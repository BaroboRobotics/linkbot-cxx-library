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

#include <algorithm>
#include <cstdlib>
#include <thread>


#include <util/asio/iothread.hpp>
#include <util/asio/ws/acceptor.hpp>
#include <util/asio/ws/connector.hpp>
#include <util/global.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/asio/use_future.hpp>

#include "message.pb.h"


#define LINKBOT_MAX_SPEED 200

using std::this_thread::sleep_for;
using std::chrono::seconds;
using std::chrono::milliseconds;

namespace barobo {

CLinkbot::CLinkbot(const std::string& serialId) : 
    _l(boost::to_upper_copy<std::string>(serialId)), 
    mSerialId(boost::to_upper_copy<std::string>(serialId))  
{ }

void CLinkbot::getAccelerometerData(double &x, double &y, double &z) {
    int timestamp;
    _l.getAccelerometer(timestamp, x, y, z);
}

void CLinkbot::getBatteryVoltage(double &voltage) {
    _l.getBatteryVoltage(voltage);
}

void CLinkbot::getFormFactor(LinkbotFormFactor &form) {
    _l.getFormFactor(form);
}

void CLinkbot::getJointAngle(LinkbotJoint joint, double &angle) {
    double angles[3];
    int timestamp;
    _l.getJointAngles(timestamp, angles[0], angles[1], angles[2]);
    angle = angles[joint];
}

void CLinkbot::getJointAngles(double &angle1, double &angle2, double &angle3) {
    int timestamp;
    _l.getJointAngles(timestamp, angle1, angle2, angle3);
}

void CLinkbot::getJointSpeed(LinkbotJoint id, double &speed) {
    double speeds[3];
    _l.getJointSpeeds(speeds[0], speeds[1], speeds[2]);
    speed = speeds[id];
}

void CLinkbot::getJointSpeedRatio(LinkbotJoint id, double &ratio) {
    double speed;
    getJointSpeed(id, speed);
    ratio = speed/LINKBOT_MAX_SPEED;
}

void CLinkbot::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
    _l.getJointSpeeds(speed1, speed2, speed3);
}

void CLinkbot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
    getJointSpeeds(ratio1, ratio2, ratio3);
    ratio1 /= LINKBOT_MAX_SPEED;
    ratio2 /= LINKBOT_MAX_SPEED;
    ratio3 /= LINKBOT_MAX_SPEED;
}

void CLinkbot::getLEDColorRGB(int &r, int &g, int &b) {
    _l.getLedColor(r, g, b);
}

// SETTERS

void CLinkbot::setBuzzerFrequency(int frequency, double time) {
    _l.setBuzzerFrequency(frequency);
    sleep_for(milliseconds(int(time*1000)));
    _l.setBuzzerFrequency(0);
}

void CLinkbot::setBuzzerFrequencyOn(int frequency) {
    _l.setBuzzerFrequency(frequency);
}

void CLinkbot::setBuzzerFrequencyOff() {
    _l.setBuzzerFrequency(0);
}

void CLinkbot::setJointMovementStateNB(LinkbotJoint id, LinkbotDirection dir)
{
    auto coefficient = 0.0;
    auto state = LINKBOT_JOINT_STATE_COAST;
    switch(dir) {
        case LINKBOT_POSITIVE:
            coefficient = 1;
            break;
        case LINKBOT_NEGATIVE:
            coefficient = -1;
            break;
        case LINKBOT_FORWARD:
            coefficient = (id == LINKBOT_JOINT_THREE)? -1 : 1;
            break;
        case LINKBOT_BACKWARD:
            coefficient = (id == LINKBOT_JOINT_THREE)? 1 : -1;
            break;
        default:
            break;
    }
    switch(dir) {
        case LINKBOT_POSITIVE:
        case LINKBOT_NEGATIVE:
        case LINKBOT_FORWARD:
        case LINKBOT_BACKWARD:
            state = LINKBOT_JOINT_STATE_MOVING;
            break;
        default:
            break;
    }
    _l.setJointStates(1<<id, 
        state, coefficient,
        state, coefficient,
        state, coefficient);
}

void CLinkbot::setJointMovementStateTime(LinkbotJoint id, LinkbotDirection dir, double seconds)
{
    setJointMovementStateTimeNB(id, dir, seconds);
    moveWait(1<<id);
}

void CLinkbot::setJointMovementStateTimeNB(LinkbotJoint id, LinkbotDirection dir, double seconds)
{
    auto coefficient = 0.0;
    auto state = LINKBOT_JOINT_STATE_COAST;
    switch(dir) {
        case LINKBOT_POSITIVE:
            coefficient = 1;
            break;
        case LINKBOT_NEGATIVE:
            coefficient = -1;
            break;
        case LINKBOT_FORWARD:
            coefficient = (id == LINKBOT_JOINT_THREE)? -1 : 1;
            break;
        case LINKBOT_BACKWARD:
            coefficient = (id == LINKBOT_JOINT_THREE)? 1 : -1;
            break;
        default:
            break;
    }
    switch(dir) {
        case LINKBOT_POSITIVE:
        case LINKBOT_NEGATIVE:
        case LINKBOT_FORWARD:
        case LINKBOT_BACKWARD:
            state = LINKBOT_JOINT_STATE_MOVING;
            break;
        default:
            break;
    }

    _l.setJointStates(1<<id,
        state, coefficient, seconds, LINKBOT_JOINT_STATE_HOLD,
        state, coefficient, seconds, LINKBOT_JOINT_STATE_HOLD,
        state, coefficient, seconds, LINKBOT_JOINT_STATE_HOLD);
}

void CLinkbot::setJointSpeed(LinkbotJoint id, double speed) {
    _l.setJointSpeeds(1<<id, speed, speed, speed);
}

void CLinkbot::setJointSpeeds(double speed1, double speed2, double speed3) {
    _l.setJointSpeeds(7, speed1, speed2, speed3);
}

void CLinkbot::setJointSpeedRatio(LinkbotJoint id, double ratio) {
    setJointSpeed(id, ratio*LINKBOT_MAX_SPEED);
}

void CLinkbot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
    setJointSpeeds(
        ratio1*LINKBOT_MAX_SPEED,
        ratio2*LINKBOT_MAX_SPEED,
        ratio3*LINKBOT_MAX_SPEED );
}

void CLinkbot::setJointPower(LinkbotJoint id, double power) {
    _l.motorPower(1<<id, power*255, power*255, power*255);
}

void CLinkbot::setLEDColorRGB(int r, int g, int b) {
    _l.setLedColor(r, g, b);
}

void CLinkbot::setMotorPowers(double p1, double p2, double p3) {
    _l.motorPower(7, p1*255, p2*255, p3*255);
}

void CLinkbot::setMovementStateNB(LinkbotDirection dir1,
            LinkbotDirection dir2,
            LinkbotDirection dir3)
{
    LinkbotJointState states[3];
    double c[3];
    std::vector<LinkbotDirection> dirs = {dir1, dir2, dir3};
    for(auto i = 0; i < 3; i++) {
        switch(dirs[i]) { 
            case LINKBOT_NEGATIVE:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = -1;
                break;
            case LINKBOT_NEUTRAL:
                states[i] = LINKBOT_JOINT_STATE_COAST;
                c[i] = 0;
                break;
            case LINKBOT_POSITIVE:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = 1;
                break;
            case LINKBOT_FORWARD:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = 1;
                if(i == 2) {
                    c[i] *= -1;
                }
                break;
            case LINKBOT_BACKWARD:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = -1;
                if(i == 2) {
                    c[i] *= -1;
                }
                break;
        }
    }   
    _l.setJointStates(0x07, 
        states[0], c[0],
        states[1], c[1],
        states[2], c[2]);
}


void CLinkbot::setMovementStateTime( LinkbotDirection dir1,
        LinkbotDirection dir2,
        LinkbotDirection dir3,
        double seconds)
{
    setMovementStateTimeNB(dir1, dir2, dir3, seconds);
    moveWait();
}

void CLinkbot::setMovementStateTimeNB( LinkbotDirection dir1,
        LinkbotDirection dir2,
        LinkbotDirection dir3,
        double seconds)
{
    LinkbotJointState states[3];
    double c[3];
    std::vector<LinkbotDirection> dirs = {dir1, dir2, dir3};
    for(auto i = 0; i < 3; i++) {
        switch(dirs[i]) { 
            case LINKBOT_NEGATIVE:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = -1;
                break;
            case LINKBOT_NEUTRAL:
                states[i] = LINKBOT_JOINT_STATE_COAST;
                c[i] = 0;
                break;
            case LINKBOT_POSITIVE:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = 1;
                break;
            case LINKBOT_FORWARD:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = 1;
                if(i == 2) {
                    c[i] *= -1;
                }
                break;
            case LINKBOT_BACKWARD:
                states[i] = LINKBOT_JOINT_STATE_MOVING;
                c[i] = -1;
                if(i == 2) {
                    c[i] *= -1;
                }
                break;
        }
    }   
    _l.setJointStates(0x07,
        states[0], c[0], seconds, LINKBOT_JOINT_STATE_HOLD,
        states[1], c[1], seconds, LINKBOT_JOINT_STATE_HOLD,
        states[2], c[2], seconds, LINKBOT_JOINT_STATE_HOLD);
}

void CLinkbot::setSpeed(double speed, double radius) {
    auto omega = speed / radius;
    omega *= 180/M_PI;
    setJointSpeeds(omega, omega, omega);
}

void CLinkbot::resetToZero() {
    _l.resetEncoderRevs();
    moveTo(0, 0, 0);
}

void CLinkbot::resetToZeroNB() {
    _l.resetEncoderRevs();
    moveToNB(0, 0, 0);
}

// MOVEMENT

void CLinkbot::move(double j1, double j2, double j3) {
    moveNB(j1, j2, j3);
    moveWait();
}

void CLinkbot::moveNB(double j1, double j2, double j3) {
    _l.move(7, j1, j2, j3);
}

void CLinkbot::moveWait(int mask) {
    _l.moveWait(mask);
}

void CLinkbot::moveJoint(LinkbotJoint id, double angle) {
    moveJointNB(id, angle);
    moveJointWait(id);
}

void CLinkbot::moveJointNB(LinkbotJoint id, double angle) {
    _l.move(1<<id, angle, angle, angle);
}

void CLinkbot::moveJointWait(LinkbotJoint id) {
    _l.moveWait(1<<id);
}

void CLinkbot::moveTo(double angle1, double angle2, double angle3) {
    moveToNB(angle1, angle2, angle3);
    moveWait(7);
}

void CLinkbot::moveToNB(double angle1, double angle2, double angle3) {
    _l.moveTo(7, angle1, angle2, angle3);
}

void CLinkbot::stop(int mask) {
    _l.stop(mask);
}

void CLinkbot::setButtonEventCallback( LinkbotButtonEventCallback cb, void* userData) {
    _l.setButtonEventCallback(cb, userData);
}

void CLinkbot::setButtonEventCallback (std::function<void(LinkbotButton, LinkbotButtonState, int)> cb) {
    _l.setButtonEventCallback(cb);
}

void CLinkbot::setEncoderEventCallback (LinkbotEncoderEventCallback cb, double granularity, void* userData) {
    _l.setEncoderEventCallback(cb, granularity, userData);
}

void CLinkbot::setEncoderEventCallback (std::function<void(int, double, int)> cb, double granularity) {
    _l.setEncoderEventCallback(cb, granularity);
}

void CLinkbot::setAccelerometerEventCallback (LinkbotAccelerometerEventCallback cb, void* userData) {
    _l.setAccelerometerEventCallback(cb, userData);
}

void CLinkbot::setAccelerometerEventCallback (std::function<void(double, double, double, int)> cb) {
    _l.setAccelerometerEventCallback(cb);
}

void CLinkbot::delaySeconds(double seconds) {
    sleep_for(milliseconds(int(seconds*1000)));
}

CLinkbotGroup::CLinkbotGroup() {}

void CLinkbotGroup::addRobot(CLinkbot& robot) {
    mRobots.insert( std::pair<std::string, CLinkbot*>( robot._serialId(), &robot ) );
}

void CLinkbotGroup::setBuzzerFrequencyOn(int frequency) {
    for ( auto& kv : mRobots ) {
        kv.second->setBuzzerFrequencyOn(frequency);
    }
}

void CLinkbotGroup::setBuzzerFrequencyOff() {
    for ( auto& kv : mRobots ) {
        kv.second->setBuzzerFrequencyOff();
    }
}

void CLinkbotGroup::setJointSpeed(LinkbotJoint id, double speed)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeed(id, speed);
    }
}

void CLinkbotGroup::setJointSpeeds(double speed1, double speed2, double speed3)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeeds(speed1, speed2, speed3);
    }
}

void CLinkbotGroup::setJointSpeedRatio(LinkbotJoint id, double ratio)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeedRatio(id, ratio);
    }
}

void CLinkbotGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeedRatios(ratio1, ratio2, ratio3);
    }
}

void CLinkbotGroup::setJointPower(LinkbotJoint id, double power)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointPower(id, power);
    }
}

void CLinkbotGroup::setLEDColorRGB(int r, int g, int b)
{
    for ( auto& kv : mRobots ) {
        kv.second->setLEDColorRGB(r, g, b);
    }
}

void CLinkbotGroup::setMotorPowers(double p1, double p2, double p3)
{
    for ( auto& kv : mRobots ) {
        kv.second->setMotorPowers(p1, p2, p3);
    }
}

void CLinkbotGroup::setSpeed(double speed, double radius)
{
    for ( auto& kv : mRobots ) {
        kv.second->setSpeed(speed, radius);
    }
}

// Group Movement

void CLinkbotGroup::move(double j1, double j2, double j3)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveNB(j1, j2, j3);
    }
    for ( auto& kv : mRobots ) {
        kv.second->moveWait();
    }
}

void CLinkbotGroup::moveNB(double j1, double j2, double j3)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveNB(j1, j2, j3);
    }
}

void CLinkbotGroup::moveWait(int mask)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveWait(mask);
    }
}

void CLinkbotGroup::moveJoint(LinkbotJoint id, double angle)
{
    moveJointNB(id, angle);
    moveWait(1<<id);
}

void CLinkbotGroup::moveJointNB(LinkbotJoint id, double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointNB(id, angle);
    }
}

void CLinkbotGroup::moveJointWait(LinkbotJoint id)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointWait(id);
    }
}

void CLinkbotGroup::moveTo(double angle1, double angle2, double angle3)
{
    moveToNB(angle1, angle2, angle3);
    moveWait();
}

void CLinkbotGroup::moveToNB(double angle1, double angle2, double angle3)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveToNB(angle1, angle2, angle3);
    }
}

void CLinkbotGroup::resetToZero()
{
    resetToZeroNB();
    moveWait();
}

void CLinkbotGroup::resetToZeroNB()
{
    for ( auto& kv : mRobots ) {
        kv.second->resetToZeroNB();
    }
}

void CLinkbotGroup::stop(int mask)
{
    for ( auto& kv : mRobots ) {
        kv.second->stop(mask);
    }
}

void sendToPrex(std::string json) {
    auto ioThread = util::global<util::asio::IoThread>();
    auto host = "localhost";
    auto service = std::getenv("PREX_IPC_PORT");

    /* Pack the json string into a protobuf message */
    PrexMessage msg;
    Image image;
    image.set_payload(json);
    image.set_format("JSON");
    std::ostringstream image_buffer;
    image.SerializeToOstream(&image_buffer);
    msg.set_payload(image_buffer.str());
    msg.set_type(PrexMessage_MessageType_IMAGE);
    std::ostringstream msg_buffer;
    msg.SerializeToOstream(&msg_buffer);

    auto connector = util::asio::ws::Connector{ioThread->context()};
    // a `ws::Connector` wraps a `websocketpp::client`
    auto clientMq = util::asio::ws::Connector::MessageQueue{ioThread->context()};
    // a `ws::Connector::MessageQueue` wraps a `websocketpp::client::connection_ptr`

    auto use_future = boost::asio::use_future_t<std::allocator<char>>{};
    // We need this special use_future to work around an Asio bug on gcc 5+.

    connector.asyncConnect(clientMq, host, service, use_future).get();

    clientMq.asyncSend(boost::asio::buffer(msg_buffer.str()), use_future).get();

    util::log::Logger lg;

    auto ec = boost::system::error_code{};
    clientMq.close(ec);
    if (ec) { BOOST_LOG(lg) << "client message queue close: " << ec.message(); }
    connector.close(ec);
    if (ec) { BOOST_LOG(lg) << "connector close: " << ec.message(); }
}

} // namespace barobo
