// Copyright (c) 2013-2016 Barobo, Inc.
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

#include "gen-robot.pb.hpp"
#include "gen-daemon.pb.hpp"
#include <baromesh/system_error.hpp>

#include <rpc/asio/client.hpp>

#include <util/asio/iothread.hpp>
#include <util/asio/operation.hpp>
//#include <util/asio/use_future.hpp>
#include <util/asio/ws/connector.hpp>

#include <util/log.hpp>
#include <util/global.hpp>

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

#include <boost/asio/yield.hpp>

#undef M_PI
#define M_PI 3.14159265358979323846

namespace barobo {

using WebSocketClient = rpc::asio::Client<util::asio::ws::Connector::MessageQueue>;
using rpc::asio::asyncFire;

namespace {

auto use_future = boost::asio::use_future_t<std::allocator<char>>{};

template <class T>
T degToRad (T x) { return T(double(x) * M_PI / 180.0); }

template <class T>
T radToDeg (T x) { return T(double(x) * 180.0 / M_PI); }

std::string daemonHostName () {
    return "127.0.0.1";
}

std::string daemonServiceName () {
    return "42000";
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

template <class Duration, class CompletionToken>
auto asyncResolveSerialId (WebSocketClient& daemon, const std::string& serialId,
        Duration&& timeout, CompletionToken&& token) {
    using StringPair = std::pair<std::string, std::string>;

    if (serialId.size() != 4) {
        throw Error{"Linkbot serial IDs must be 4 characters"};
    }

    auto coroutine =
    [ &daemon
    , serialId
    , timeout = std::forward<Duration>(timeout)
    , port = uint16_t{}
    ]
    (auto&& op, boost::system::error_code ec = {},
            rpc::MethodResult<barobo::Daemon>::resolveSerialId arg = {}) mutable {
        reenter (op) {
            yield {
                assert(serialId.size() == 4);
                auto arg = rpc::MethodIn<barobo::Daemon>::resolveSerialId{};
                strncpy(arg.serialId.value, serialId.data(), 4);
                arg.serialId.value[4] = 0;
                asyncFire(daemon, arg, timeout, std::move(op));
            }
            if (ec) { op.complete(ec, StringPair{}); return; }

            if (arg.status) {
                op.complete(make_error_code(baromesh::Status(arg.status)), StringPair{});
                return;
            }

            if ((port = arg.endpoint.port) != arg.endpoint.port) {
                op.complete(make_error_code(baromesh::Status::PORT_OUT_OF_RANGE), StringPair{});
                return;
            }

            using std::to_string;
            op.complete(ec, std::make_pair(daemonHostName(), to_string(port)));
            // arg.endpoint.host is deprecated: use daemonHostName()
        }
    };

    return util::asio::asyncDispatch(
        daemon.get_io_service(),
        std::make_tuple(make_error_code(boost::asio::error::operation_aborted), StringPair{}),
        std::move(coroutine),
        std::forward<CompletionToken>(token)
    );
}

} // <anonymous>

using MethodIn = rpc::MethodIn<barobo::Robot>;
using MethodResult = rpc::MethodResult<barobo::Robot>;
using Broadcast = rpc::Broadcast<barobo::Robot>;

struct Linkbot::Impl {
private:
    explicit Impl (const std::string& host, const std::string& service)
        : io(util::global<util::asio::IoThread>())
        , wsConnector(io->context())
        , robot(io->context())
    {
        BOOST_LOG(log) << "Connecting to Linkbot proxy at " << host << ":" << service;
        wsConnector.asyncConnect(robot.messageQueue(), host, service, use_future).get();
        rpc::asio::asyncConnect<barobo::Robot>(robot, requestTimeout(), use_future).get();
        robotRunDone = rpc::asio::asyncRunClient<barobo::Robot>(robot, *this, use_future);
    }

public:
#if 0
    static Impl* fromWebSocketEndpoint (const std::string& host, const std::string& service) {
        initializeLoggingCore();
        return new Impl{host, service};
    }
#endif

    // Use the daemon to resolve a serial ID to WebSocket URI and
    // construct a Linkbot::Impl backed by this host:service. The caller has
    // ownership of the returned pointer.
    static Impl* fromSerialId (const std::string& serialId) {
        initializeLoggingCore();

        auto io = util::global<util::asio::IoThread>();
        util::log::Logger lg;

        BOOST_LOG(lg) << "Connecting to the daemon ...";

        util::asio::ws::Connector dConnector {io->context()};
        WebSocketClient daemon {io->context()};
        dConnector.asyncConnect(daemon.messageQueue(),
                daemonHostName(), daemonServiceName(), use_future).get();

        std::string host, service;
        std::tie(host, service) = asyncResolveSerialId(
                daemon, serialId, requestTimeout(), use_future).get();

        daemon.close();

        BOOST_LOG(lg) << "Got robot URI: " << host << ":" << service;
        return new Impl{host, service};
    }

    ~Impl () {
        if (robotRunDone.valid()) {
            try {
                BOOST_LOG(log) << "Disconnecting robot client";
                asyncDisconnect(robot, requestTimeout(), use_future).get();
                robot.close();
                robotRunDone.get();
            }
            catch (std::exception& e) {
                BOOST_LOG(log) << "Exception during disconnect: " << e.what();
            }
        }
    }

    void onBroadcast (Broadcast::buttonEvent b) {
        if (buttonEventCallback) {
            buttonEventCallback(static_cast<LinkbotButton>(b.button),
                                static_cast<LinkbotButtonState>(b.state),
                                b.timestamp);
        }
    }

    void onBroadcast (Broadcast::encoderEvent b) {
        if (encoderEventCallback) {
            encoderEventCallback(b.encoder, radToDeg(b.value), b.timestamp);
        }
    }

    void onBroadcast (Broadcast::accelerometerEvent b) {
        if (accelerometerEventCallback) {
            accelerometerEventCallback(b.x, b.y, b.z, b.timestamp);
        }
    }

    void onBroadcast (Broadcast::jointEvent b) {
        if (jointEventCallback) {
            jointEventCallback(b.joint, static_cast<LinkbotJointState>(b.event), b.timestamp);
        }
    }

    void onBroadcast (Broadcast::debugMessageEvent e) {
        BOOST_LOG(log) << "Debug message from robot: " << e.bytestring;
    }

    void onBroadcast (Broadcast::connectionTerminated b) {
        BOOST_LOG(log) << "Connection terminated at " << b.timestamp;
        if (connectionTerminatedCallback) {
            connectionTerminatedCallback(b.timestamp);
        }
    }

    std::shared_ptr<util::asio::IoThread> io;
    util::asio::ws::Connector wsConnector;

    WebSocketClient robot;  // RPC client
    std::future<void> robotRunDone;

    std::function<void(LinkbotButton, LinkbotButtonState, int)> buttonEventCallback;
    std::function<void(int,double, int)> encoderEventCallback;
    std::function<void(int,LinkbotJointState, int)> jointEventCallback;
    std::function<void(double,double,double,int)> accelerometerEventCallback;
    std::function<void(int)> connectionTerminatedCallback;

    mutable util::log::Logger log;
};

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
{}
catch (std::exception& e) {
    throw Error(id + ": " + e.what());
}

Linkbot::~Linkbot () {
    delete m;
}

using namespace std::placeholders; // _1, _2, etc.

/* GETTERS */

void Linkbot::getAccelerometer (int& timestamp, double&x, double&y, double&z)
{
    try {
        auto value = asyncFire(m->robot, MethodIn::getAccelerometerData{}, requestTimeout(), use_future).get();
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
        auto value = asyncFire( m->robot,
                                MethodIn::getAdcRaw{},
                                requestTimeout(),
                                use_future).get();
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
        auto value = asyncFire(m->robot, MethodIn::getBatteryVoltage{}, requestTimeout(), use_future).get();
        volts = value.v;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getFormFactor(LinkbotFormFactor& form)
{
    try {
        auto value = asyncFire(m->robot, MethodIn::getFormFactor{}, requestTimeout(), use_future).get();
        form = LinkbotFormFactor(value.value);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointAngles (int& timestamp, double& a0, double& a1, double& a2) {
    try {
        auto values = asyncFire(m->robot, MethodIn::getEncoderValues{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        a0 = radToDeg(values.values[0]);
        a1 = radToDeg(values.values[1]);
        a2 = radToDeg(values.values[2]);
        timestamp = values.timestamp;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointSpeeds(double& s1, double& s2, double& s3)
{
    try {
        auto values = asyncFire(m->robot, MethodIn::getMotorControllerOmega{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        s1 = radToDeg(values.values[0]);
        s2 = radToDeg(values.values[1]);
        s3 = radToDeg(values.values[2]);
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
        auto values = asyncFire(m->robot, MethodIn::getJointStates{}, requestTimeout(), use_future).get();
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
        auto color = asyncFire(m->robot, MethodIn::getLedColor{}, requestTimeout(), use_future).get();
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
        v = asyncFire(m->robot,
                MethodIn::getFirmwareVersionString{}, requestTimeout(), use_future).get().value;
        BOOST_LOG(m->log) << "Firmware version " << v;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointSafetyThresholds(int& t1, int& t2, int& t3)
{
    try {
        auto value = asyncFire(m->robot,
            MethodIn::getMotorControllerSafetyThreshold{},
            requestTimeout(), use_future).get();
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
        auto value = asyncFire(m->robot,
            MethodIn::getMotorControllerSafetyAngle{},
            requestTimeout(), use_future).get();
        t1 = radToDeg(double(value.values[0]));
        t2 = radToDeg(double(value.values[1]));
        t3 = radToDeg(double(value.values[2]));
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
        asyncFire(m->robot, MethodIn::resetEncoderRevs{}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setBuzzerFrequency (double freq) {
    try {
        asyncFire(m->robot, MethodIn::setBuzzerFrequency{float(freq)}, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointSpeeds (int mask, double s0, double s1, double s2) {
    try {
        MethodIn::setMotorControllerOmega arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { s0, s1, s2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(degToRad(s));
            }
            jointFlag <<= 1;
        }
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
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
    barobo_Robot_Goal_Type goalType[3];
    barobo_Robot_Goal_Controller controllerType[3];
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
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_HOLD:
                goalType[i] = barobo_Robot_Goal_Type_RELATIVE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_MOVING:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_CONSTVEL;
                break;
            default:
                break;
        }
    }
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { goalType[0], coefficients[0], true, controllerType[0] },
            bool(mask&0x02), { goalType[1], coefficients[1], true, controllerType[1] },
            bool(mask&0x04), { goalType[2], coefficients[2], true, controllerType[2] }
        }, requestTimeout(), use_future).get();
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
    barobo_Robot_Goal_Type goalType[3];
    barobo_Robot_Goal_Controller controllerType[3];
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
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_HOLD:
                goalType[i] = barobo_Robot_Goal_Type_RELATIVE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case LINKBOT_JOINT_STATE_MOVING:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_CONSTVEL;
                break;
            default:
                break;
        }
    }
    try {
        auto js_to_int = [] (LinkbotJointState js) {
            switch(js) {
                case LINKBOT_JOINT_STATE_COAST:
                    return barobo_Robot_JointState_COAST;
                case LINKBOT_JOINT_STATE_HOLD:
                    return barobo_Robot_JointState_HOLD;
                case LINKBOT_JOINT_STATE_MOVING:
                    return barobo_Robot_JointState_MOVING;
                default:
                    return barobo_Robot_JointState_COAST;
            }
        };

        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01),
            { goalType[0], coefficients[0], true, controllerType[0],
                hasTimeouts[0], float(timeout1), hasTimeouts[0], js_to_int(end1)},
            bool(mask&0x02),
            { goalType[1], coefficients[1], true, controllerType[1],
                hasTimeouts[1], float(timeout2), hasTimeouts[1], js_to_int(end2)},
            bool(mask&0x04),
            { goalType[2], coefficients[2], true, controllerType[2],
                hasTimeouts[2], float(timeout3), hasTimeouts[2], js_to_int(end3)}
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setLedColor (int r, int g, int b) {
    try {
        asyncFire(m->robot, MethodIn::setLedColor{
            uint32_t(r << 16 | g << 8 | b)
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointSafetyThresholds(int mask, int t0, int t1, int t2) {
    try {
        MethodIn::setMotorControllerSafetyThreshold arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& t : { t0, t1, t2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = t;
            }
            jointFlag <<= 1;
        }
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::setJointSafetyAngles(int mask, double t0, double t1, double t2) {
    try {
        MethodIn::setMotorControllerSafetyAngle arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& t : { t0, t1, t2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(degToRad(t));
            }
            jointFlag <<= 1;
        }
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
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
        MethodIn::setMotorControllerAlphaI arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { a0, a1, a2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(degToRad(s));
            }
            jointFlag <<= 1;
        }
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
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
        MethodIn::setMotorControllerAlphaF arg;
        arg.mask = mask;
        arg.values_count = 0;
        int jointFlag = 0x01;
        for (auto& s : { a0, a1, a2 }) {
            if (jointFlag & mask) {
                arg.values[arg.values_count++] = float(degToRad(s));
            }
            jointFlag <<= 1;
        }
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}
/* MOVEMENT */

void Linkbot::drive (int mask, double a0, double a1, double a2)
{
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE,
                               float(degToRad(a0)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE,
                               float(degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE,
                               float(degToRad(a2)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::driveTo (int mask, double a0, double a1, double a2)
{
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE,
                               float(degToRad(a0)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE,
                               float(degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE,
                               float(degToRad(a2)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::move (int mask, double a0, double a1, double a2) {
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_RELATIVE,
                               float(degToRad(a0)),
                               false},
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE,
                               float(degToRad(a1)),
                               false},
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE,
                               float(degToRad(a2)),
                               false}
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveContinuous (int mask, double c0, double c1, double c2) {
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_INFINITE, float(c0), false },
            bool(mask&0x02), { barobo_Robot_Goal_Type_INFINITE, float(c1), false },
            bool(mask&0x04), { barobo_Robot_Goal_Type_INFINITE, float(c2), false }
        }, requestTimeout(), use_future).get();
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
    barobo_Robot_Goal_Type motionType[3];
    for(int i = 0; i < 3; i++) {
        if(relativeMask & (1<<i)) {
            motionType[i] = barobo_Robot_Goal_Type_RELATIVE;
        } else {
            motionType[i] = barobo_Robot_Goal_Type_ABSOLUTE;
        }
    }
    try {
        auto js_to_int = [] (LinkbotJointState js) {
            switch(js) {
                case LINKBOT_JOINT_STATE_COAST:
                    return barobo_Robot_JointState_COAST;
                case LINKBOT_JOINT_STATE_HOLD:
                    return barobo_Robot_JointState_HOLD;
                case LINKBOT_JOINT_STATE_MOVING:
                    return barobo_Robot_JointState_MOVING;
                default:
                    return barobo_Robot_JointState_COAST;
            }
        };

        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), {
                motionType[0],
                float(degToRad(omega0_i)),
                true,
                barobo_Robot_Goal_Controller_ACCEL,
                hasTimeouts[0], float(timeout0), hasTimeouts[0], js_to_int(endstate0)
                },
            bool(mask&0x02), {
                motionType[1],
                float(degToRad(omega1_i)),
                true,
                barobo_Robot_Goal_Controller_ACCEL,
                hasTimeouts[1], float(timeout1), hasTimeouts[1], js_to_int(endstate1)
                },
            bool(mask&0x04), {
                motionType[2],
                float(degToRad(omega2_i)),
                true,
                barobo_Robot_Goal_Controller_ACCEL,
                hasTimeouts[2], float(timeout2), hasTimeouts[2], js_to_int(endstate2)
                }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveSmooth(int mask, int relativeMask, double a0, double a1, double a2)
{
    barobo_Robot_Goal_Type motionType[3];
    for(int i = 0; i < 3; i++) {
        if(relativeMask & (1<<i)) {
            motionType[i] = barobo_Robot_Goal_Type_RELATIVE;
        } else {
            motionType[i] = barobo_Robot_Goal_Type_ABSOLUTE;
        }
    }

    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), {
                motionType[0],
                float(degToRad(a0)),
                true,
                barobo_Robot_Goal_Controller_SMOOTH
                },
            bool(mask&0x02), {
                motionType[1],
                float(degToRad(a1)),
                true,
                barobo_Robot_Goal_Controller_SMOOTH
                },
            bool(mask&0x04), {
                motionType[2],
                float(degToRad(a2)),
                true,
                barobo_Robot_Goal_Controller_SMOOTH
                }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::moveTo (int mask, double a0, double a1, double a2) {
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a0)) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a1)) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, float(degToRad(a2)) }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::motorPower(int mask, int m1, int m2, int m3)
{
    try {
        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), { barobo_Robot_Goal_Type_INFINITE,
                               float(m1),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_INFINITE,
                               float(m2),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_INFINITE,
                               float(m3),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             }
        }, requestTimeout(), use_future).get();
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::stop (int mask) {
    try {
        asyncFire(m->robot, MethodIn::stop{true, static_cast<uint32_t>(mask)}, requestTimeout(), use_future).get();
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
        asyncFire(m->robot, MethodIn::enableAccelerometerEvent {
            enable, granularity
        }, requestTimeout(), use_future).get();
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

void Linkbot::setButtonEventCallback (LinkbotButtonEventCallback cb, void* userData) {
    const bool enable = !!cb;

    try {
        asyncFire(m->robot, MethodIn::enableButtonEvent{enable}, requestTimeout(), use_future).get();
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

void Linkbot::setEncoderEventCallback (LinkbotEncoderEventCallback cb,
                                       double granularity, void* userData)
{
    const bool enable = !!cb;
    granularity = degToRad(granularity);

    try {
        asyncFire(m->robot, MethodIn::enableEncoderEvent {
            true, { enable, float(granularity) },
            true, { enable, float(granularity) },
            true, { enable, float(granularity) }
        }, requestTimeout(), use_future).get();
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

void Linkbot::setJointEventCallback (LinkbotJointEventCallback cb, void* userData) {
    const bool enable = !!cb;
    try {
        asyncFire(m->robot, MethodIn::enableJointEvent {
            enable
        }, requestTimeout(), use_future).get();
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

void Linkbot::setConnectionTerminatedCallback (LinkbotConnectionTerminatedCallback cb, void* userData) {
    m->connectionTerminatedCallback = std::bind(cb, _1, userData);
}


void Linkbot::writeEeprom(uint32_t address, const uint8_t *data, size_t size)
{
    if(size > 128) {
        throw Error("Payload size too large");
    }
    try {
        MethodIn::writeEeprom arg;
        arg.address = address;
        memcpy(arg.data.bytes, data, size);
        arg.data.size = size;
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
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
        MethodIn::readEeprom arg;
        arg.address = address;
        arg.size = recvsize;
        auto result = asyncFire(m->robot, arg, requestTimeout(), use_future).get();
        memcpy(buffer, result.data.bytes, result.data.size);
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
        MethodIn::writeTwi arg;
        arg.address = address;
        memcpy(arg.data.bytes, data, size);
        arg.data.size = size;
        asyncFire(m->robot, arg, requestTimeout(), use_future).get();
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
        MethodIn::readTwi arg;
        arg.address = address;
        arg.recvsize = recvsize;
        auto result = asyncFire(m->robot, arg, requestTimeout(), use_future).get();
        memcpy(buffer, result.data.bytes, result.data.size);
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
        MethodIn::writeReadTwi arg;
        arg.address = address;
        arg.recvsize = recvsize;
        memcpy(arg.data.bytes, sendbuf, sendsize);
        arg.data.size = sendsize;
        auto result = asyncFire(m->robot, arg, requestTimeout(), use_future).get();
        memcpy(recvbuf, result.data.bytes, result.data.size);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

} // namespace

#include <boost/asio/unyield.hpp>
