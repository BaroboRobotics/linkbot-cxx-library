#include <linkbot/linkbot.hpp>

#include <thread>

#define LINKBOT_MAX_SPEED 200

using namespace barobo;
using std::this_thread::sleep_for;
using std::chrono::seconds;
using std::chrono::milliseconds;

CLinkbot::CLinkbot(const std::string& serialId) {
    _l = new Linkbot(serialId);
}

CLinkbot::~CLinkbot() {
    delete _l;
}

void CLinkbot::getAccelerometerData(double &x, double &y, double &z) {
    int timestamp;
    _l->getAccelerometer(timestamp, x, y, z);
}

void CLinkbot::getBatteryVoltage(double &voltage) {
    _l->getBatteryVoltage(voltage);
}

void CLinkbot::getFormFactor(LinkbotFormFactor &form) {
    _l->getFormFactor(form);
}

void CLinkbot::getJointAngle(LinkbotJoint joint, double &angle) {
    double angles[3];
    int timestamp;
    _l->getJointAngles(timestamp, angles[0], angles[1], angles[2]);
    angle = angles[joint];
}

void CLinkbot::getJointAngles(double &angle1, double &angle2, double &angle3) {
    int timestamp;
    _l->getJointAngles(timestamp, angle1, angle2, angle3);
}

void CLinkbot::getJointSpeed(LinkbotJoint id, double &speed) {
    double speeds[3];
    _l->getJointSpeeds(speeds[0], speeds[1], speeds[2]);
    speed = speeds[id];
}

void CLinkbot::getJointSpeedRatio(LinkbotJoint id, double &ratio) {
    double speed;
    getJointSpeed(id, speed);
    ratio = speed/LINKBOT_MAX_SPEED;
}

void CLinkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
    _l->getJointSpeeds(speed1, speed2, speed3);
}

void CLinkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
    getJointSpeeds(ratio1, ratio2, ratio3);
    ratio1 /= LINKBOT_MAX_SPEED;
    ratio2 /= LINKBOT_MAX_SPEED;
    ratio3 /= LINKBOT_MAX_SPEED;
}

void CLinkbot::getLEDColorRGB(int &r, int &g, int &b) {
    _l->getLedColor(r, g, b);
}

// SETTERS

void CLinkbot::setBuzzerFrequency(int frequency, double time) {
    _l->setBuzzerFrequency(frequency);
    sleep_for(milliseconds(int(time*1000)));
    _l->setBuzzerFrequency(0);
}

void CLinkbot::setBuzzerFrequencyOn(int frequency) {
    _l->setBuzzerFrequency(frequency);
}

void CLinkbot::setBuzzerFrequencyOff() {
    _l->setBuzzerFrequency(0);
}

