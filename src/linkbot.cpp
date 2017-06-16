// Copyright (c) 2013-2017 Barobo, Inc.
//
// This file is part of liblinkbot.
//
// liblinkbot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// liblinkbot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with liblinkbot.  If not, see <http://www.gnu.org/licenses/>.

#include <linkbot/linkbot.hpp>
#include <linkbot/error.hpp>

#include <daemon.pb.hpp>
#include <robot.pb.hpp>

#include "robot-write-stream.hpp"

#include <util/asio/iothread.hpp>

#include <util/overload.hpp>
#include <util/math.hpp>
#include <util/log.hpp>
#include <util/global.hpp>

#include <composed/rpc_stream.hpp>
#include <composed/rpc_client.hpp>

#include <beast/websocket.hpp>

#include <boost/asio/use_future.hpp>

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <boost/program_options/parsers.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace barobo {

namespace {

using namespace std::literals;

auto use_future = boost::asio::use_future_t<std::allocator<char>>{};

std::string daemonHostName () {
    return "127.0.0.1";
}

uint16_t daemonPortNo () {
    return 42000;
}

std::chrono::milliseconds requestTimeout () {
    return std::chrono::seconds{10};
}

void initializeLoggingCore () {
    static std::once_flag flag;
    std::call_once(flag, [] {
        namespace po = boost::program_options;

        auto optsDesc = util::log::optionsDescription(util::log::ConsoleDefault::OFF);
        auto options = po::variables_map{};

        // We're in library code, so interpret environment variables as command line arguments.
        // Example: LINKBOT_LOG_FILE=<arg> would be interpreted as --log-file=<arg>.
        auto transformEnvVar = [] (std::string var) {
            if (boost::starts_with(var, "LINKBOT_")) {
                boost::erase_first(var, "LINKBOT_");
                boost::replace_all(var, "_", "-");
                boost::to_lower(var);
                return var;
            }
            else {
                return std::string{};
            }
        };

        po::store(po::parse_environment(optsDesc, transformEnvVar), options);
        po::notify(options);
    });
}

} // <anonymous>

struct Linkbot::Impl {
private:
    explicit Impl (const std::string& sid)
        : io(util::global<util::asio::IoThread>())
        , stream(io->context())
        , wsStream(stream)
        , msgStream(wsStream)
        , rpcStream(msgStream)
        , daemonRpcClient(rpcStream)
        , daemonRpc(daemonRpcClient)
        , robotWriteStream(daemonRpc, sid)
        , robotRpcClient(robotWriteStream)
        , robotRpc(robotRpcClient)
        , serialId(sid)
    {
        if (serialId.size() > 4) {
            throw boost::system::system_error{make_error_code(DaemonStatus::INVALID_SERIALID)};
        }

        const auto daemonEp = boost::asio::ip::tcp::endpoint{
                boost::asio::ip::address::from_string(daemonHostName()), daemonPortNo()};

        stream.connect(daemonEp);
        wsStream.handshake("foo.com", "/");
        wsStream.binary(true);
        robotRunDone = rpcStream.async_run_read_loop(
                util::overload([this](const auto& e, auto&& h) {
                            this->event(e);
                            io->context().post(std::forward<decltype(h)>(h));
                        },
                        [this](const linkbot_daemon_RpcReply& e, auto&& h) {
                            daemonRpcClient.event(e);
                            io->context().post(std::forward<decltype(h)>(h));
                        }),
                use_future);
        linkbot_daemon_addRobotRefs_In refRequest{};
        refRequest.serialIds_count = 1;
        strcpy(refRequest.serialIds[0].value, serialId.c_str());
        daemonRpc.async_do_request(refRequest, linkbot_daemon_RpcReply_addRobotRefs_tag,
                5s, use_future).get();
        if (daemonRpc.reply().addRobotRefs.has_status && daemonRpc.reply().addRobotRefs.status) {
            throw boost::system::system_error{
                    make_error_code(DaemonStatus(daemonRpc.reply().addRobotRefs.status))};
        }
        isMoving = 0;
    }

    // ===================================================================================
    // Daemon events

    void event(const linkbot_daemon_ReceiveTransmission& e) {
        if (e.has_serialId && serialId == std::string(e.serialId.value)) {
            auto visitor = util::overload([this](const auto& robotEvent) {
                        this->event(robotEvent);
                    }, [this](const linkbot_robot_RpcReply& robotEvent) {
                        robotRpcClient.event(robotEvent);
                    });
            if (!nanopb::visit(visitor, e.payload.arg)) {
                BOOST_LOG(log) << "Unrecognized robot message";
            }
        }
    }

    void event(const linkbot_daemon_DongleEvent& e) {
        BOOST_LOG(log) << "DongleEvent unimplemented";
    }

    // ===================================================================================
    // Robot events

    void event(const linkbot_robot_AccelerometerEvent& e) {
        if (accelerometerEventCallback && e.has_x && e.has_y && e.has_z && e.has_timestamp) {
            accelerometerEventCallback(e.x, e.y, e.z, e.timestamp);
        }
    }

    void event(const linkbot_robot_ButtonEvent& e) {
        if (buttonEventCallback && e.has_button && e.has_state && e.has_timestamp) {
            buttonEventCallback(static_cast<LinkbotButton>(e.button),
                                static_cast<LinkbotButtonState>(e.state),
                                e.timestamp);
        }
    }

    void event(const linkbot_robot_EncoderEvent& e) {
        if (encoderEventCallback && e.has_encoder && e.has_value && e.has_timestamp) {
            encoderEventCallback(e.encoder, util::radToDeg(e.value), e.timestamp);
        }
    }

    void event(const linkbot_robot_JointEvent& e) {
        if (e.has_joint && e.has_event && e.has_timestamp) {
            _onJointEvent(e.joint, static_cast<LinkbotJointState>(e.event), e.timestamp);
        }
    }

    void _onJointEvent(int joint, LinkbotJointState event, int timestamp)
    {
        if (jointEventCallback) {
            jointEventCallback(joint, event, timestamp);
            return;
        }
        switch(event) {
            case LINKBOT_JOINT_STATE_COAST:
                // intentional fall-through
            case LINKBOT_JOINT_STATE_HOLD:
                isMoving &= ~(1<<joint);
                break;
            default:
                break;
        }
        if ( ! (isMoving & moveWaitMask) ) {
            try {
                moveWaitPromise.set_value();
            } catch ( const std::future_error& ) {
            }
        }
    }

    void event(const linkbot_robot_DebugMessageEvent& e) {
        if (e.has_bytestring) {
            BOOST_LOG(log) << "Debug message from robot: " << e.bytestring;
        }
    }

    void event(const linkbot_robot_ConnectionTerminated& e) {
        if (e.has_timestamp) {
            BOOST_LOG(log) << "Connection terminated at " << e.timestamp;
            if (connectionTerminatedCallback) {
                connectionTerminatedCallback(e.timestamp);
            }
        }
    }

    void event(const linkbot_robot_PowerOnEvent& e) {
        BOOST_LOG(log) << "PowerOnEvent unimplemented";
    }

public:
    // The caller has ownership of the returned pointer.
    static Impl* fromSerialId (const std::string& serialId) {
        initializeLoggingCore();
        return new Impl{serialId};
    }

    static Impl* fromEnv() {
        // Get the serial id from the environment
        auto env_str = std::getenv("ROBOTMANAGER_IDS");
        if(!env_str) {
            throw Error("Environment variable ROBOTMANAGER_IDS not set.");
        }
        std::string env{env_str};
        std::istringstream ss{env};
        std::string token;
        _connect_n++;
        for(int i = 0; i < _connect_n; i++) {
            std::getline(ss, token, ',');
        }
        if ( token.length() != 4 ) {
            throw Error("Insufficient number of robots connected in robot manager.");
        }
        return fromSerialId(token);
    }

    ~Impl () {
        if (robotRunDone.valid()) {
            try {
                BOOST_LOG(log) << "Disconnecting robot client";
                linkbot_daemon_releaseRobotRefs_In refRequest{};
                refRequest.serialIds_count = 1;
                strcpy(refRequest.serialIds[0].value, serialId.c_str());
                daemonRpc.async_do_request(refRequest, linkbot_daemon_RpcReply_releaseRobotRefs_tag,
                        5s, use_future).get();
                wsStream.close("client destroyed");
                boost::system::error_code ec;
                daemonRpc.close(ec);
                robotRpc.close(ec);
                robotRunDone.get();
            }
            catch (std::exception& e) {
                BOOST_LOG(log) << "Exception during disconnect: " << e.what();
            }
        }
    }

    void setMoving(int mask) {
        switch (formFactor) {
            case LINKBOT_FORM_FACTOR_I:
                mask &= 0x05;
                break;
            case LINKBOT_FORM_FACTOR_L:
                mask &= 0x03;
                break;
            case LINKBOT_FORM_FACTOR_T:
                mask &= 0x07;
                break;
        }
        isMoving |= mask;
    }

    std::shared_ptr<util::asio::IoThread> io;

    boost::asio::ip::tcp::socket stream;
    beast::websocket::stream<boost::asio::ip::tcp::socket&> wsStream;
    composed::websocket msgStream;

    using DaemonTxMsg = linkbot_daemon_ClientToDaemon;
    using DaemonRxMsg = linkbot_daemon_DaemonToClient;
    using DaemonTxRpc = linkbot_daemon_RpcRequest;
    using DaemonRxRpc = linkbot_daemon_RpcReply;
    using RobotTxRpc = linkbot_robot_RpcRequest;
    using RobotRxRpc = linkbot_robot_RpcReply;

    composed::rpc_stream<composed::websocket&, DaemonTxMsg, DaemonRxMsg> rpcStream;
    composed::rpc_client<decltype(rpcStream)&, DaemonTxRpc, DaemonRxRpc> daemonRpcClient;
    typename decltype(daemonRpcClient)::transactor daemonRpc;
    RobotWriteStream<decltype(daemonRpc)> robotWriteStream;
    composed::rpc_client<decltype(robotWriteStream)&, RobotTxRpc, RobotRxRpc> robotRpcClient;
    typename decltype(robotRpcClient)::transactor robotRpc;

    std::string serialId;

    std::future<void> robotRunDone;

    std::function<void(LinkbotButton, LinkbotButtonState, int)> buttonEventCallback;
    std::function<void(int,double, int)> encoderEventCallback;
    std::function<void(int,LinkbotJointState, int)> jointEventCallback;
    std::function<void(double,double,double,int)> accelerometerEventCallback;
    std::function<void(int)> connectionTerminatedCallback;

    mutable util::log::Logger log;

    int isMoving;
    int moveWaitMask;
    LinkbotFormFactor formFactor;
    std::promise<void> moveWaitPromise;
    static int _connect_n; // Number of robots connected using the ID-less "connect" function
};

int Linkbot::Impl::_connect_n = 0;

#if 0
Linkbot::Linkbot (const std::string& host, const std::string& service) try
    : m(Linkbot::Impl::fromWebSocketEndpoint(host, service))
{}
catch (std::exception& e) {
    throw Error(e.what());
}
#endif

Linkbot::Linkbot (const std::string& id) try
    : m(Linkbot::Impl::fromSerialId(id))
{
    // Get the form factor
    LinkbotFormFactor form;
    getFormFactor(form);
    m->formFactor = form;
    initJointEventCallback();
}
catch (std::exception& e) {
    throw Error(id + ": " + e.what());
}

Linkbot::Linkbot () try
    : m(Linkbot::Impl::fromEnv())
{
    // Get the form factor
    LinkbotFormFactor form;
    getFormFactor(form);
    m->formFactor = form;
    initJointEventCallback();
}
catch (std::exception& e) {
    throw Error(e.what());
}

Linkbot::~Linkbot () {
    delete m;
}

using namespace std::placeholders; // _1, _2, etc.

#define REQUEST(x, ...) linkbot_robot_##x##_In{__VA_ARGS__}, linkbot_robot_RpcReply_##x##_tag

void Linkbot::initJointEventCallback () {
    try {
        m->robotRpc.async_do_request(REQUEST(enableJointEvent, true, true), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

/* GETTERS */

void Linkbot::getAccelerometer (int& timestamp, double&x, double&y, double&z)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getAccelerometerData), requestTimeout(), use_future).get();
        auto& value = m->robotRpc.reply().getAccelerometerData;
        x = value.x;
        y = value.y;
        z = value.z;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

std::vector<int> Linkbot::getAdcRaw()
{
    try {
        std::vector<int> rvalues;
        m->robotRpc.async_do_request(REQUEST(getAdcRaw),
                                requestTimeout(),
                                use_future).get();
        auto& value = m->robotRpc.reply().getAdcRaw;
        for(auto i = 0; i < value.values_count; i++) {
            rvalues.push_back(value.values[i]);
        }
        return rvalues;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getBatteryVoltage(double& volts)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getBatteryVoltage), requestTimeout(), use_future).get();
        volts = m->robotRpc.reply().getBatteryVoltage.v;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getFormFactor(LinkbotFormFactor& form)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getFormFactor), requestTimeout(), use_future).get();
        form = LinkbotFormFactor(m->robotRpc.reply().getFormFactor.value);
        BOOST_LOG(m->log) << "Got form factor: " << form;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointAngles (int& timestamp, double& a0, double& a1, double& a2) {
    try {
        m->robotRpc.async_do_request(REQUEST(getEncoderValues), requestTimeout(), use_future).get();
        auto& values = m->robotRpc.reply().getEncoderValues;
        assert(values.values_count >= 3);
        a0 = util::radToDeg(values.values[0]);
        a1 = util::radToDeg(values.values[1]);
        a2 = util::radToDeg(values.values[2]);
        timestamp = values.timestamp;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointSpeeds(double& s1, double& s2, double& s3)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getMotorControllerOmega), requestTimeout(), use_future).get();
        auto& values = m->robotRpc.reply().getMotorControllerOmega;
        assert(values.values_count >= 3);
        s1 = util::radToDeg(values.values[0]);
        s2 = util::radToDeg(values.values[1]);
        s3 = util::radToDeg(values.values[2]);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointStates(int& timestamp,
                             LinkbotJointState& s1,
                             LinkbotJointState& s2,
                             LinkbotJointState& s3)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getJointStates), requestTimeout(), use_future).get();
        auto& values = m->robotRpc.reply().getJointStates;
        assert(values.values_count >= 3);
        s1 = static_cast<LinkbotJointState>(values.values[0]);
        s2 = static_cast<LinkbotJointState>(values.values[1]);
        s3 = static_cast<LinkbotJointState>(values.values[2]);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getLedColor (int& r, int& g, int& b) {
    try {
        m->robotRpc.async_do_request(REQUEST(getLedColor), requestTimeout(), use_future).get();
        auto color = m->robotRpc.reply().getLedColor;
        r = 0xff & color.value >> 16;
        g = 0xff & color.value >> 8;
        b = 0xff & color.value;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getVersionString (std::string& v) {
    try {
        m->robotRpc.async_do_request(REQUEST(getFirmwareVersionString), requestTimeout(), use_future).get();
        v = m->robotRpc.reply().getFirmwareVersionString.value;
        BOOST_LOG(m->log) << "Firmware version " << v;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointSafetyThresholds(int& t1, int& t2, int& t3)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getMotorControllerSafetyThreshold), requestTimeout(), use_future).get();
        auto& value = m->robotRpc.reply().getMotorControllerSafetyThreshold;
        t1 = value.values[0];
        t2 = value.values[1];
        t3 = value.values[2];
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointSafetyAngles(double& t1, double& t2, double& t3)
{
    try {
        m->robotRpc.async_do_request(REQUEST(getMotorControllerSafetyAngle), requestTimeout(), use_future).get();
        auto& value = m->robotRpc.reply().getMotorControllerSafetyAngle;
        t1 = util::radToDeg(double(value.values[0]));
        t2 = util::radToDeg(double(value.values[1]));
        t3 = util::radToDeg(double(value.values[2]));
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getSerialId(std::string& serialId)
{
    char buf[5];
    readEeprom(0x412, 4, (uint8_t*)buf);
    buf[4] = '\0';
    serialId = std::string(buf);
}

/* SETTERS */
void Linkbot::resetEncoderRevs() {
    try {
        m->robotRpc.async_do_request(REQUEST(resetEncoderRevs), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setBuzzerFrequency (double freq) {
    try {
        m->robotRpc.async_do_request(REQUEST(setBuzzerFrequency, true, float(freq)), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointSpeeds (int mask, double s0, double s1, double s2) {
    try {
        linkbot_robot_setMotorControllerOmega_In arg;
        arg.has_mask = true;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { s0, s1, s2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(util::degToRad(s));
            }
            jointFlag <<= 1;
        }
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_setMotorControllerOmega_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointStates(
        int mask,
        LinkbotJointState s1, double d1,
        LinkbotJointState s2, double d2,
        LinkbotJointState s3, double d3
        )
{
    linkbot_robot_Goal_Type goalType[3];
    linkbot_robot_Goal_Controller controllerType[3];
    LinkbotJointState jointStates[3];
    float coefficients[3];
    jointStates[0] = s1;
    jointStates[1] = s2;
    jointStates[2] = s3;
    coefficients[0] = d1;
    coefficients[1] = d2;
    coefficients[2] = d3;
    for(int i = 0; i < 3; i++) {
        switch(jointStates[i]) {
            case LINKBOT_JOINT_STATE_COAST:
                goalType[i] = linkbot_robot_Goal_Type_INFINITE;
                controllerType[i] = linkbot_robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_HOLD:
                goalType[i] = linkbot_robot_Goal_Type_RELATIVE;
                controllerType[i] = linkbot_robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_MOVING:
                goalType[i] = linkbot_robot_Goal_Type_INFINITE;
                controllerType[i] = linkbot_robot_Goal_Controller_CONSTVEL;
                m->setMoving(1<<i);
                break;
            case LINKBOT_JOINT_STATE_POWER:
                goalType[i] = linkbot_robot_Goal_Type_INFINITE;
                controllerType[i] = linkbot_robot_Goal_Controller_PID;
                m->setMoving(1<<i);
                break;
            default:
                break;
        }
    }
    try {
        m->robotRpc.async_do_request(REQUEST(move,
                bool(mask&0x01), { true, goalType[0], true, coefficients[0], true, controllerType[0] },
                bool(mask&0x02), { true, goalType[1], true, coefficients[1], true, controllerType[1] },
                bool(mask&0x04), { true, goalType[2], true, coefficients[2], true, controllerType[2] }),
                requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointStates(
        int mask,
        LinkbotJointState s1, double d1, double timeout1, LinkbotJointState end1,
        LinkbotJointState s2, double d2, double timeout2, LinkbotJointState end2,
        LinkbotJointState s3, double d3, double timeout3, LinkbotJointState end3
        )
{
    linkbot_robot_Goal_Type goalType[3];
    linkbot_robot_Goal_Controller controllerType[3];
    LinkbotJointState jointStates[3];
    float coefficients[3];
    jointStates[0] = s1;
    jointStates[1] = s2;
    jointStates[2] = s3;
    coefficients[0] = d1;
    coefficients[1] = d2;
    coefficients[2] = d3;
    bool hasTimeouts[3];
    hasTimeouts[0] = (timeout1 != 0.0);
    hasTimeouts[1] = (timeout2 != 0.0);
    hasTimeouts[2] = (timeout3 != 0.0);

    for(int i = 0; i < 3; i++) {
        switch(jointStates[i]) {
            case LINKBOT_JOINT_STATE_COAST:
                goalType[i] = linkbot_robot_Goal_Type_INFINITE;
                controllerType[i] = linkbot_robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_HOLD:
                goalType[i] = linkbot_robot_Goal_Type_RELATIVE;
                controllerType[i] = linkbot_robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_MOVING:
                goalType[i] = linkbot_robot_Goal_Type_INFINITE;
                controllerType[i] = linkbot_robot_Goal_Controller_CONSTVEL;
                m->setMoving(1<<i);
                break;
            case LINKBOT_JOINT_STATE_POWER:
                goalType[i] = linkbot_robot_Goal_Type_INFINITE;
                controllerType[i] = linkbot_robot_Goal_Controller_PID;
                m->setMoving(1<<i);
                break;
            default:
                break;
        }
    }
    try {
        auto js_to_int = [] (LinkbotJointState js) {
            switch(js) {
                case LINKBOT_JOINT_STATE_COAST:
                    return linkbot_robot_JointState_COAST;
                case LINKBOT_JOINT_STATE_HOLD:
                    return linkbot_robot_JointState_HOLD;
                case LINKBOT_JOINT_STATE_MOVING:
                    return linkbot_robot_JointState_MOVING;
                default:
                    return linkbot_robot_JointState_COAST;
            }
        };

        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01),
            { true, goalType[0], true, coefficients[0], true, controllerType[0],
                hasTimeouts[0], float(timeout1), hasTimeouts[0], js_to_int(end1)},
            bool(mask&0x02),
            { true, goalType[1], true, coefficients[1], true, controllerType[1],
                hasTimeouts[1], float(timeout2), hasTimeouts[1], js_to_int(end2)},
            bool(mask&0x04),
            { true, goalType[2], true, coefficients[2], true, controllerType[2],
                hasTimeouts[2], float(timeout3), hasTimeouts[2], js_to_int(end3)}),
            requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setLedColor (int r, int g, int b) {
    try {
        m->robotRpc.async_do_request(REQUEST(setLedColor, true, uint32_t(r << 16 | g << 8 | b)), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointSafetyThresholds(int mask, int t0, int t1, int t2) {
    try {
        linkbot_robot_setMotorControllerSafetyThreshold_In arg;
        arg.has_mask = true;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& t : { t0, t1, t2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = t;
            }
            jointFlag <<= 1;
        }
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_setMotorControllerSafetyThreshold_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointSafetyAngles(int mask, double t0, double t1, double t2) {
    try {
        linkbot_robot_setMotorControllerSafetyAngle_In arg;
        arg.has_mask = true;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& t : { t0, t1, t2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(util::degToRad(t));
            }
            jointFlag <<= 1;
        }
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_setMotorControllerSafetyAngle_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointAccelI(
    int mask,
    double a0, double a1, double a2)
{
    try {
        linkbot_robot_setMotorControllerAlphaI_In arg;
        arg.has_mask = true;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { a0, a1, a2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(util::degToRad(s));
            }
            jointFlag <<= 1;
        }
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_setMotorControllerAlphaI_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointAccelF(
    int mask,
    double a0, double a1, double a2)
{
    try {
        linkbot_robot_setMotorControllerAlphaF_In arg;
        arg.has_mask = true;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { a0, a1, a2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(util::degToRad(s));
            }
            jointFlag <<= 1;
        }
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_setMotorControllerAlphaF_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}
/* MOVEMENT */

void Linkbot::drive (int mask, double a0, double a1, double a2)
{
    try {
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), { true, linkbot_robot_Goal_Type_RELATIVE,
                               true, float(util::degToRad(a0)),
                               true, linkbot_robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { true, linkbot_robot_Goal_Type_RELATIVE,
                               true, float(util::degToRad(a1)),
                               true, linkbot_robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { true, linkbot_robot_Goal_Type_RELATIVE,
                               true, float(util::degToRad(a2)),
                               true, linkbot_robot_Goal_Controller_PID
                             }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::driveTo (int mask, double a0, double a1, double a2)
{
    try {
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), { true, linkbot_robot_Goal_Type_ABSOLUTE,
                               true, float(util::degToRad(a0)),
                               true, linkbot_robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { true, linkbot_robot_Goal_Type_ABSOLUTE,
                               true, float(util::degToRad(a1)),
                               true, linkbot_robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { true, linkbot_robot_Goal_Type_ABSOLUTE,
                               true, float(util::degToRad(a2)),
                               true, linkbot_robot_Goal_Controller_PID
                             }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::move (int mask, double a0, double a1, double a2) {
    try {
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), { true, linkbot_robot_Goal_Type_RELATIVE,
                               true, float(util::degToRad(a0)),
                               false},
            bool(mask&0x02), { true, linkbot_robot_Goal_Type_RELATIVE,
                               true, float(util::degToRad(a1)),
                               false},
            bool(mask&0x04), { true, linkbot_robot_Goal_Type_RELATIVE,
                               true, float(util::degToRad(a2)),
                               false}
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveContinuous (int mask, double c0, double c1, double c2) {
    try {
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), { true, linkbot_robot_Goal_Type_INFINITE, true, float(c0), false },
            bool(mask&0x02), { true, linkbot_robot_Goal_Type_INFINITE, true, float(c1), false },
            bool(mask&0x04), { true, linkbot_robot_Goal_Type_INFINITE, true, float(c2), false }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveAccel(int mask, int relativeMask,
    double omega0_i, double timeout0, LinkbotJointState endstate0,
    double omega1_i, double timeout1, LinkbotJointState endstate1,
    double omega2_i, double timeout2, LinkbotJointState endstate2)
{
    bool hasTimeouts[3];
    hasTimeouts[0] = (timeout0 != 0.0);
    hasTimeouts[1] = (timeout1 != 0.0);
    hasTimeouts[2] = (timeout2 != 0.0);
    linkbot_robot_Goal_Type motionType[3];
    for(int i = 0; i < 3; i++) {
        if(relativeMask & (1<<i)) {
            motionType[i] = linkbot_robot_Goal_Type_RELATIVE;
        } else {
            motionType[i] = linkbot_robot_Goal_Type_ABSOLUTE;
        }
    }
    try {
        auto js_to_int = [] (LinkbotJointState js) {
            switch(js) {
                case LINKBOT_JOINT_STATE_COAST:
                    return linkbot_robot_JointState_COAST;
                case LINKBOT_JOINT_STATE_HOLD:
                    return linkbot_robot_JointState_HOLD;
                case LINKBOT_JOINT_STATE_MOVING:
                    return linkbot_robot_JointState_MOVING;
                default:
                    return linkbot_robot_JointState_COAST;
            }
        };
        
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), {
                true, motionType[0],
                true, float(util::degToRad(omega0_i)),
                true, linkbot_robot_Goal_Controller_ACCEL,
                hasTimeouts[0], float(timeout0), hasTimeouts[0], js_to_int(endstate0)
                },
            bool(mask&0x02), {
                true, motionType[1],
                true, float(util::degToRad(omega1_i)),
                true, linkbot_robot_Goal_Controller_ACCEL,
                hasTimeouts[1], float(timeout1), hasTimeouts[1], js_to_int(endstate1)
                },
            bool(mask&0x04), {
                true, motionType[2],
                true, float(util::degToRad(omega2_i)),
                true, linkbot_robot_Goal_Controller_ACCEL,
                hasTimeouts[2], float(timeout2), hasTimeouts[2], js_to_int(endstate2)
                }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveSmooth(int mask, int relativeMask, double a0, double a1, double a2)
{
    linkbot_robot_Goal_Type motionType[3];
    for(int i = 0; i < 3; i++) {
        if(relativeMask & (1<<i)) {
            motionType[i] = linkbot_robot_Goal_Type_RELATIVE;
        } else {
            motionType[i] = linkbot_robot_Goal_Type_ABSOLUTE;
        }
    }

    try {
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), {
                true, motionType[0],
                true, float(util::degToRad(a0)),
                true, linkbot_robot_Goal_Controller_SMOOTH
                },
            bool(mask&0x02), {
                true, motionType[1],
                true, float(util::degToRad(a1)),
                true, linkbot_robot_Goal_Controller_SMOOTH
                },
            bool(mask&0x04), {
                true, motionType[2],
                true, float(util::degToRad(a2)),
                true, linkbot_robot_Goal_Controller_SMOOTH
                }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveTo (int mask, double a0, double a1, double a2) {
    try {
        m->setMoving(mask);
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), { true, linkbot_robot_Goal_Type_ABSOLUTE, true, float(util::degToRad(a0)) },
            bool(mask&0x02), { true, linkbot_robot_Goal_Type_ABSOLUTE, true, float(util::degToRad(a1)) },
            bool(mask&0x04), { true, linkbot_robot_Goal_Type_ABSOLUTE, true, float(util::degToRad(a2)) }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveWait(int mask) {
    mask &= 0x07;
    if(! (m->isMoving & mask) ) {
        return;
    }
    m->moveWaitMask = mask;
    m->moveWaitPromise = std::promise<void>{};
    auto future = m->moveWaitPromise.get_future();
    future.wait();
}

void Linkbot::motorPower(int mask, int m1, int m2, int m3)
{
    try {
        m->robotRpc.async_do_request(REQUEST(move,
            bool(mask&0x01), { true, linkbot_robot_Goal_Type_INFINITE,
                               true, float(m1),
                               true, linkbot_robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { true, linkbot_robot_Goal_Type_INFINITE,
                               true, float(m2),
                               true, linkbot_robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { true, linkbot_robot_Goal_Type_INFINITE,
                               true, float(m3),
                               true, linkbot_robot_Goal_Controller_PID
                             }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::stop (int mask) {
    try {
        m->robotRpc.async_do_request(REQUEST(stop, true, static_cast<uint32_t>(mask)), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

/* CALLBACKS */

void Linkbot::setAccelerometerEventCallback (LinkbotAccelerometerEventCallback cb, void* userData) {
    const bool enable = !!cb;
    auto granularity = float(enable ? 0.05 : 0);

    try {
        m->robotRpc.async_do_request(REQUEST(enableAccelerometerEvent,
            true, enable, true, granularity
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->accelerometerEventCallback = std::bind(cb, _1, _2, _3, _4, userData);
    }
    else {
        m->accelerometerEventCallback = nullptr;
    }
}

void Linkbot::setAccelerometerEventCallback (std::function<void(double, double, double, int)> cb) {
    const bool enable = !!cb;
    auto granularity = float(enable ? 0.05 : 0);

    try {
        m->robotRpc.async_do_request(REQUEST(enableAccelerometerEvent,
            true, enable, true, granularity
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->accelerometerEventCallback = cb;
    }
    else {
        m->accelerometerEventCallback = nullptr;
    }
}


void Linkbot::setButtonEventCallback (LinkbotButtonEventCallback cb, void* userData) {
    const bool enable = !!cb;

    try {
        m->robotRpc.async_do_request(REQUEST(enableButtonEvent, enable), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->buttonEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->buttonEventCallback = nullptr;
    }
}

void Linkbot::setButtonEventCallback (std::function<void(LinkbotButton, LinkbotButtonState, int)> cb) {
    const bool enable = !!cb;

    try {
        m->robotRpc.async_do_request(REQUEST(enableButtonEvent, enable), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->buttonEventCallback = cb;
    }
    else {
        m->buttonEventCallback = nullptr;
    }
}

void Linkbot::setEncoderEventCallback (LinkbotEncoderEventCallback cb,
                                       double granularity, void* userData)
{
    const bool enable = !!cb;
    granularity = util::degToRad(granularity);

    try {
        m->robotRpc.async_do_request(REQUEST(enableEncoderEvent,
            true, { true, enable, true, float(granularity) },
            true, { true, enable, true, float(granularity) },
            true, { true, enable, true, float(granularity) }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->encoderEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->encoderEventCallback = nullptr;
    }
}

void Linkbot::setEncoderEventCallback (std::function<void(int, double, int)> cb, double granularity)
{
    const bool enable = !!cb;
    granularity = util::degToRad(granularity);

    try {
        m->robotRpc.async_do_request(REQUEST(enableEncoderEvent,
            true, { true, enable, true, float(granularity) },
            true, { true, enable, true, float(granularity) },
            true, { true, enable, true, float(granularity) }
        ), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->encoderEventCallback = cb;
    }
    else {
        m->encoderEventCallback = nullptr;
    }
}

void Linkbot::setJointEventCallback (LinkbotJointEventCallback cb, void* userData) {
    const bool enable = !!cb;

    try {
        m->robotRpc.async_do_request(REQUEST(enableJointEvent, enable), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->jointEventCallback = std::bind(cb, _1, _2, _3, userData);
    }
    else {
        m->jointEventCallback = nullptr;
    }
}

void Linkbot::setJointEventCallback (std::function<void(int, LinkbotJointState, int)> cb) {
    const bool enable = !!cb;

    try {
        m->robotRpc.async_do_request(REQUEST(enableJointEvent, enable), requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }

    if (enable) {
        m->jointEventCallback = cb;
    }
    else {
        m->jointEventCallback = nullptr;
    }
}

void Linkbot::setConnectionTerminatedCallback (LinkbotConnectionTerminatedCallback cb, void* userData) {
    m->connectionTerminatedCallback = std::bind(cb, _1, userData);
}


void Linkbot::writeEeprom(uint32_t address, const uint8_t *data, size_t size)
{
    if(size > 128) {
        throw Error("Payload size too large");
    }
    try {
        linkbot_robot_writeEeprom_In arg;
        arg.has_address = true;
        arg.has_data = true;
        arg.address = address;
        memcpy(arg.data.bytes, data, size);
        arg.data.size = size;
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_writeEeprom_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::readEeprom(uint32_t address, size_t recvsize, uint8_t *buffer)
{
    if(recvsize > 128) {
        throw Error("Payload size too large");
    }
    try {
        linkbot_robot_readEeprom_In arg;
        arg.has_address = true;
        arg.has_size = true;
        arg.address = address;
        arg.size = recvsize;
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_readEeprom_tag, requestTimeout(), use_future).get();
        memcpy(buffer, m->robotRpc.reply().readEeprom.data.bytes, m->robotRpc.reply().readEeprom.data.size);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::writeTwi(uint32_t address, const uint8_t *data, size_t size)
{
    if(size > 128) {
        throw Error("Payload size too large");
    }
    try {
        linkbot_robot_writeTwi_In arg;
        arg.has_address = true;
        arg.has_data = true;
        arg.address = address;
        memcpy(arg.data.bytes, data, size);
        arg.data.size = size;
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_writeTwi_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::readTwi(uint32_t address, size_t recvsize, uint8_t *buffer)
{
    if(recvsize > 128) {
        throw Error("Payload size too large");
    }
    try {
        linkbot_robot_readTwi_In arg;
        arg.has_address = true;
        arg.has_recvsize = true;
        arg.address = address;
        arg.recvsize = recvsize;
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_readTwi_tag, requestTimeout(), use_future).get();
        memcpy(buffer, m->robotRpc.reply().readTwi.data.bytes, m->robotRpc.reply().readTwi.data.size);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::writeReadTwi(
    uint32_t address,
    const uint8_t *sendbuf,
    size_t sendsize,
    uint8_t* recvbuf,
    size_t recvsize)
{
    if((recvsize > 128) || (sendsize > 128)) {
        throw Error("Payload size too large");
    }
    try {
        linkbot_robot_writeReadTwi_In arg;
        arg.has_address = true;
        arg.has_recvsize = true;
        arg.has_data = true;
        arg.address = address;
        arg.recvsize = recvsize;
        memcpy(arg.data.bytes, sendbuf, sendsize);
        arg.data.size = sendsize;
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_writeReadTwi_tag, requestTimeout(), use_future).get();
        memcpy(recvbuf, m->robotRpc.reply().readTwi.data.bytes, m->robotRpc.reply().readTwi.data.size);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setPeripheralResetMask(int mask, int resetMask)
{
    try {
        linkbot_robot_setResetOnDisconnect_In arg;
        arg.has_mask = true;
        arg.has_peripheralResetMask = true;
        arg.mask = mask;
        arg.peripheralResetMask = resetMask;
        m->robotRpc.async_do_request(arg, linkbot_robot_RpcReply_setResetOnDisconnect_tag, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

} // namespace
