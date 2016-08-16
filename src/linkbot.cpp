#include "daemon.hpp"

#include <baromesh/linkbot.hpp>
#include <baromesh/error.hpp>
#include <baromesh/log.hpp>

#include "gen-robot.pb.hpp"

#include "websocketclient.hpp"
#include <baromesh/websocketconnector.hpp>

#include <util/asio/iothread.hpp>

#include <boost/asio/use_future.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <boost/program_options/parsers.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace barobo {

namespace {

std::chrono::milliseconds requestTimeout () {
    return std::chrono::milliseconds{1000};
}

} // file namespace

using MethodIn = rpc::MethodIn<barobo::Robot>;
using MethodResult = rpc::MethodResult<barobo::Robot>;
using Broadcast = rpc::Broadcast<barobo::Robot>;
using rpc::asio::asyncFire;

using boost::asio::use_future;

struct Linkbot::Impl {
private:
    explicit Impl (const std::string& host, const std::string& service)
        : io(util::asio::IoThread::getGlobal())
        , wsConnector(io->context())
        , robot(io->context())
    {
#if 0
        auto uri = std::make_shared<websocketpp::uri>(false, host, service, "");
        BOOST_LOG(log) << "Connecting to Linkbot proxy at " << uri->str();
        connector.init_asio(&io->context());
        auto ec = boost::system::error_code{};
        auto con = connector.get_connection(uri, ec);
        if (ec) { throw boost::system::system_error{ec}; }
        auto openPromise = std::promise<void>{};
        auto openFuture = openPromise.get_future();
        con->set_open_handler([con, &openPromise](websocketpp::connection_hdl) {
            openPromise.set_value();
        });
        con->set_fail_handler([con, &openPromise](websocketpp::connection_hdl) {
            auto e = boost::system::system_error{con->get_transport_ec()};
            openPromise.set_exception(std::make_exception_ptr(e));
        });
        connector.connect(con);
        openFuture.get();
        robot.messageQueue().setConnection(con);
#endif
        BOOST_LOG(log) << "Connecting to Linkbot proxy at " << host << ":" << service;
        wsConnector.asyncConnect(robot.messageQueue(), host, service, use_future).get();
        rpc::asio::asyncConnect<barobo::Robot>(robot, requestTimeout(), use_future).get();
        robotRunDone = rpc::asio::asyncRunClient<barobo::Robot>(robot, *this, use_future);
    }

    static void initializeLoggingCore () {
        static std::once_flag flag;
        std::call_once(flag, [] {
            namespace po = boost::program_options;

            auto optsDesc = baromesh::log::optionsDescription(boost::none);

            auto options = boost::program_options::variables_map{};
            po::store(po::parse_environment(optsDesc, [] (std::string var) {
                if (boost::starts_with(var, "BAROMESH_")) {
                    boost::erase_first(var, "BAROMESH_");
                    boost::replace_all(var, "_", "-");
                    boost::to_lower(var);
                    return var;
                }
                else {
                    return std::string{};
                }
            }), options);
            po::notify(options);

            baromesh::log::initialize("baromesh", options);
        });
    }

public:
    static Impl* fromWebSocketEndpoint (const std::string& host, const std::string& service) {
        initializeLoggingCore();
        return new Impl{host, service};
    }

    // Use the daemon to resolve a serial ID to WebSocket URI and
    // construct a Linkbot::Impl backed by this host:service. The caller has
    // ownership of the returned pointer.
    static Impl* fromSerialId (const std::string& serialId) {
        initializeLoggingCore();
        auto io = util::asio::IoThread::getGlobal();
        boost::log::sources::logger log;
        baromesh::WebSocketClient daemon {io->context()};

#if 0
        auto daemonUri = std::make_shared<websocketpp::uri>(
            false, baromesh::daemonHostName(), baromesh::daemonServiceName(), "");
        BOOST_LOG(log) << "Connecting to the daemon at " << daemonUri->str();
        websocketpp::client<websocketpp::config::asio_client> dConnector;  // transport creator
        dConnector.init_asio(&io->context());
        auto ec = boost::system::error_code{};
        auto con = dConnector.get_connection(daemonUri, ec);
        if (ec) { throw boost::system::system_error{ec}; }
        auto openPromise = std::promise<void>{};
        auto openFuture = openPromise.get_future();
        con->set_open_handler([con, &openPromise](websocketpp::connection_hdl) {
            openPromise.set_value();
        });
        con->set_fail_handler([con, &openPromise](websocketpp::connection_hdl) {
            auto e = boost::system::system_error{con->get_transport_ec()};
            openPromise.set_exception(std::make_exception_ptr(e));
        });
        dConnector.connect(con);
        openFuture.get();
        daemon.messageQueue().setConnection(con);
#endif
        baromesh::websocket::Connector dConnector{io->context()};
        dConnector.asyncConnect(daemon.messageQueue(),
            baromesh::daemonHostName(), baromesh::daemonServiceName(), use_future).get();
        rpc::asio::asyncConnect<barobo::Daemon>(daemon, requestTimeout(), use_future).get();

        std::string host, service;
        std::tie(host, service) = baromesh::asyncResolveSerialId(daemon,
            serialId, requestTimeout(), use_future).get();

        BOOST_LOG(log) << "Disconnecting daemon client";
        asyncDisconnect(daemon, requestTimeout(), use_future).get();
        daemon.close();

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
            buttonEventCallback(static_cast<Button::Type>(b.button),
                                static_cast<ButtonState::Type>(b.state),
                                b.timestamp);
        }
    }

    void onBroadcast (Broadcast::encoderEvent b) {
        if (encoderEventCallback) {
            encoderEventCallback(b.encoder, baromesh::radToDeg(b.value), b.timestamp);
        }
    }

    void onBroadcast (Broadcast::accelerometerEvent b) {
        if (accelerometerEventCallback) {
            accelerometerEventCallback(b.x, b.y, b.z, b.timestamp);
        }
    }

    void onBroadcast (Broadcast::jointEvent b) {
        if (jointEventCallback) {
            jointEventCallback(b.joint, static_cast<JointState::Type>(b.event), b.timestamp);
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
    mutable boost::log::sources::logger log;

    std::shared_ptr<util::asio::IoThread> io;
    baromesh::websocket::Connector wsConnector;

    baromesh::WebSocketClient robot;  // RPC client
    std::future<void> robotRunDone;

    std::function<void(Button::Type, ButtonState::Type, int)> buttonEventCallback;
    std::function<void(int,double, int)> encoderEventCallback;
    std::function<void(int,JointState::Type, int)> jointEventCallback;
    std::function<void(double,double,double,int)> accelerometerEventCallback;
    std::function<void(int)> connectionTerminatedCallback;
};

Linkbot::Linkbot (const std::string& host, const std::string& service) try
    : m(Linkbot::Impl::fromWebSocketEndpoint(host, service))
{}
catch (std::exception& e) {
    throw Error(e.what());
}

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

void Linkbot::getBatteryVoltage(double &volts)
{
    try {
        auto value = asyncFire(m->robot, MethodIn::getBatteryVoltage{}, requestTimeout(), use_future).get();
        volts = value.v;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getFormFactor(FormFactor::Type& form)
{
    try {
        auto value = asyncFire(m->robot, MethodIn::getFormFactor{}, requestTimeout(), use_future).get();
        form = FormFactor::Type(value.value);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointAngles (int& timestamp, double& a0, double& a1, double& a2) {
    try {
        auto values = asyncFire(m->robot, MethodIn::getEncoderValues{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        a0 = baromesh::radToDeg(values.values[0]);
        a1 = baromesh::radToDeg(values.values[1]);
        a2 = baromesh::radToDeg(values.values[2]);
        timestamp = values.timestamp;
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointSpeeds(double&s1, double&s2, double&s3)
{
    try {
        auto values = asyncFire(m->robot, MethodIn::getMotorControllerOmega{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        s1 = baromesh::radToDeg(values.values[0]);
        s2 = baromesh::radToDeg(values.values[1]);
        s3 = baromesh::radToDeg(values.values[2]);
    }
    catch (std::exception& e) {
        throw Error(e.what());
    }
}

void Linkbot::getJointStates(int& timestamp,
                             JointState::Type& s1,
                             JointState::Type& s2,
                             JointState::Type& s3)
{
    try {
        auto values = asyncFire(m->robot, MethodIn::getJointStates{}, requestTimeout(), use_future).get();
        assert(values.values_count >= 3);
        s1 = static_cast<JointState::Type>(values.values[0]);
        s2 = static_cast<JointState::Type>(values.values[1]);
        s3 = static_cast<JointState::Type>(values.values[2]);
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

void Linkbot::getVersions (uint32_t& major, uint32_t& minor, uint32_t& patch) {
    try {
        auto version = asyncFire(m->robot, MethodIn::getFirmwareVersion{}, requestTimeout(), use_future).get();
        major = version.major;
        minor = version.minor;
        patch = version.patch;
        BOOST_LOG(m->log) << "Firmware version "
                           << major << '.' << minor << '.' << patch;
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
        auto rad2deg = [] (double x) -> double {return x*180.0/M_PI;};
        auto value = asyncFire(m->robot,
            MethodIn::getMotorControllerSafetyAngle{},
            requestTimeout(), use_future).get();
        t1 = rad2deg(value.values[0]);
        t2 = rad2deg(value.values[1]);
        t3 = rad2deg(value.values[2]);
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
                arg.values[arg.values_count++] = float(baromesh::degToRad(s));
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
        JointState::Type s1, double d1,
        JointState::Type s2, double d2,
        JointState::Type s3, double d3
        )
{
    barobo_Robot_Goal_Type goalType[3];
    barobo_Robot_Goal_Controller controllerType[3];
    JointState::Type jointStates[3];
    float coefficients[3];
    jointStates[0] = s1;
    jointStates[1] = s2;
    jointStates[2] = s3;
    coefficients[0] = d1;
    coefficients[1] = d2;
    coefficients[2] = d3;
    for(int i = 0; i < 3; i++) {
        switch(jointStates[i]) {
            case JointState::COAST:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case JointState::HOLD:
                goalType[i] = barobo_Robot_Goal_Type_RELATIVE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case JointState::MOVING:
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
        JointState::Type s1, double d1, double timeout1, JointState::Type end1,
        JointState::Type s2, double d2, double timeout2, JointState::Type end2,
        JointState::Type s3, double d3, double timeout3, JointState::Type end3
        )
{
    barobo_Robot_Goal_Type goalType[3];
    barobo_Robot_Goal_Controller controllerType[3];
    JointState::Type jointStates[3];
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
            case JointState::COAST:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
                break;
            case JointState::HOLD:
                goalType[i] = barobo_Robot_Goal_Type_RELATIVE;
                controllerType[i] = barobo_Robot_Goal_Controller_PID;
                coefficients[i] = 0;
            case JointState::MOVING:
                goalType[i] = barobo_Robot_Goal_Type_INFINITE;
                controllerType[i] = barobo_Robot_Goal_Controller_CONSTVEL;
                break;
            default:
                break;
        }
    }
    try {
        auto js_to_int = [] (JointState::Type js) {
            switch(js) {
                case JointState::COAST:
                    return barobo_Robot_JointState_COAST;
                case JointState::HOLD:
                    return barobo_Robot_JointState_HOLD;
                case JointState::MOVING:
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
                arg.values[arg.values_count++] = float(baromesh::degToRad(t));
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
                arg.values[arg.values_count++] = float(baromesh::degToRad(s));
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
                arg.values[arg.values_count++] = float(baromesh::degToRad(s));
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
                               float(baromesh::degToRad(a0)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE,
                               float(baromesh::degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE,
                               float(baromesh::degToRad(a2)),
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
                               float(baromesh::degToRad(a0)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE,
                               float(baromesh::degToRad(a1)),
                               true,
                               barobo_Robot_Goal_Controller_PID
                             },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE,
                               float(baromesh::degToRad(a2)),
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
                               float(baromesh::degToRad(a0)),
                               false},
            bool(mask&0x02), { barobo_Robot_Goal_Type_RELATIVE,
                               float(baromesh::degToRad(a1)),
                               false},
            bool(mask&0x04), { barobo_Robot_Goal_Type_RELATIVE,
                               float(baromesh::degToRad(a2)),
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
    double omega0_i, double timeout0, JointState::Type endstate0,
    double omega1_i, double timeout1, JointState::Type endstate1,
    double omega2_i, double timeout2, JointState::Type endstate2)
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
        auto js_to_int = [] (JointState::Type js) {
            switch(js) {
                case JointState::COAST:
                    return barobo_Robot_JointState_COAST;
                case JointState::HOLD:
                    return barobo_Robot_JointState_HOLD;
                case JointState::MOVING:
                    return barobo_Robot_JointState_MOVING;
                default:
                    return barobo_Robot_JointState_COAST;
            }
        };

        asyncFire(m->robot, MethodIn::move {
            bool(mask&0x01), {
                motionType[0],
                float(baromesh::degToRad(omega0_i)),
                true,
                barobo_Robot_Goal_Controller_ACCEL,
                hasTimeouts[0], float(timeout0), hasTimeouts[0], js_to_int(endstate0)
                },
            bool(mask&0x02), {
                motionType[1],
                float(baromesh::degToRad(omega1_i)),
                true,
                barobo_Robot_Goal_Controller_ACCEL,
                hasTimeouts[1], float(timeout1), hasTimeouts[1], js_to_int(endstate1)
                },
            bool(mask&0x04), {
                motionType[2],
                float(baromesh::degToRad(omega2_i)),
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
                float(baromesh::degToRad(a0)),
                true,
                barobo_Robot_Goal_Controller_SMOOTH
                },
            bool(mask&0x02), {
                motionType[1],
                float(baromesh::degToRad(a1)),
                true,
                barobo_Robot_Goal_Controller_SMOOTH
                },
            bool(mask&0x04), {
                motionType[2],
                float(baromesh::degToRad(a2)),
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
            bool(mask&0x01), { barobo_Robot_Goal_Type_ABSOLUTE, float(baromesh::degToRad(a0)) },
            bool(mask&0x02), { barobo_Robot_Goal_Type_ABSOLUTE, float(baromesh::degToRad(a1)) },
            bool(mask&0x04), { barobo_Robot_Goal_Type_ABSOLUTE, float(baromesh::degToRad(a2)) }
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

void Linkbot::setAccelerometerEventCallback (AccelerometerEventCallback cb, void* userData) {
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

void Linkbot::setButtonEventCallback (ButtonEventCallback cb, void* userData) {
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

void Linkbot::setEncoderEventCallback (EncoderEventCallback cb,
                                       double granularity, void* userData)
{
    const bool enable = !!cb;
    granularity = baromesh::degToRad(granularity);

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

void Linkbot::setJointEventCallback (JointEventCallback cb, void* userData) {
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

void Linkbot::setConnectionTerminatedCallback (ConnectionTerminatedCallback cb, void* userData) {
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
