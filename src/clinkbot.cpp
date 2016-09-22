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
#include <thread>

#define LINKBOT_MAX_SPEED 200

using namespace barobo;
using std::this_thread::sleep_for;
using std::chrono::seconds;
using std::chrono::milliseconds;

CLinkbot::CLinkbot(const std::string& serialId) {
    auto _id = serialId;
    std::transform(_id.begin(), _id.end(), _id.begin(), ::toupper);
    _l = new Linkbot(_id);
    mSerialId = _id;
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

void CLinkbot::setJointSpeed(LinkbotJoint id, double speed) {
    _l->setJointSpeeds(1<<id, speed, speed, speed);
}

void CLinkbot::setJointSpeeds(double speed1, double speed2, double speed3) {
    _l->setJointSpeeds(7, speed1, speed2, speed3);
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
    _l->motorPower(1<<id, power*255, power*255, power*255);
}

void CLinkbot::setLEDColorRGB(int r, int g, int b) {
    _l->setLedColor(r, g, b);
}

void CLinkbot::setMotorPowers(double p1, double p2, double p3) {
    _l->motorPower(7, p1*255, p2*255, p3*255);
}

void CLinkbot::setSpeed(double speed, double radius) {
    auto omega = speed / radius;
    omega *= 180/M_PI;
    setJointSpeeds(omega, omega, omega);
}

void CLinkbot::resetToZero() {
    _l->resetEncoderRevs();
    moveTo(0, 0, 0);
}

void CLinkbot::resetToZeroNB() {
    _l->resetEncoderRevs();
    moveToNB(0, 0, 0);
}

// MOVEMENT

void CLinkbot::move(double j1, double j2, double j3) {
    moveNB(j1, j2, j3);
    moveWait();
}

void CLinkbot::moveNB(double j1, double j2, double j3) {
    _l->move(7, j1, j2, j3);
}

void CLinkbot::moveWait(int mask) {
    _l->moveWait(mask);
}

void CLinkbot::moveJoint(LinkbotJoint id, double angle) {
    moveJointNB(id, angle);
    moveJointWait(id);
}

void CLinkbot::moveJointNB(LinkbotJoint id, double angle) {
    _l->move(1<<id, angle, angle, angle);
}

void CLinkbot::moveJointWait(LinkbotJoint id) {
    _l->moveWait(1<<id);
}

void CLinkbot::moveTo(double angle1, double angle2, double angle3) {
    moveToNB(angle1, angle2, angle3);
    moveWait(7);
}

void CLinkbot::moveToNB(double angle1, double angle2, double angle3) {
    _l->moveTo(7, angle1, angle2, angle3);
}

void CLinkbot::stop(int mask) {
    _l->stop(mask);
}

void CLinkbot::delaySeconds(double seconds) {
    sleep_for(milliseconds(int(seconds*1000)));
}

CLinkbotGroup::CLinkbotGroup() {}

CLinkbotGroup::~CLinkbotGroup() {}

void CLinkbotGroup::addRobots(CLinkbot& robot) {
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