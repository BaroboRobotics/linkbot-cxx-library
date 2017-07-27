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
#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

namespace barobo {

CLinkbotI::CLinkbotI(const std::string& serialId)
: CLinkbot(serialId)
{ 
    // Make sure we are a Linkbot-I 
    LinkbotFormFactor form;
    getFormFactor(form);
    if ( form != LINKBOT_FORM_FACTOR_I ) {
        throw Error("Connected Linkbot is not a Linkbot-I.");
    }
}

CLinkbotI::CLinkbotI()
: CLinkbot()
{ 
    // Make sure we are a Linkbot-I 
    LinkbotFormFactor form;
    getFormFactor(form);
    if ( form != LINKBOT_FORM_FACTOR_I ) {
        throw Error("Connected Linkbot is not a Linkbot-I.");
    }
}

void CLinkbotI::closeGripper() {
    closeGripperNB();
    moveWait();
}

void CLinkbotI::closeGripperNB() {
    Linkbot::setJointStates( 0x05,
        LINKBOT_JOINT_STATE_POWER, 64, 2, LINKBOT_JOINT_STATE_HOLD,
        LINKBOT_JOINT_STATE_POWER, 64, 2, LINKBOT_JOINT_STATE_HOLD,
        LINKBOT_JOINT_STATE_POWER, 64, 2, LINKBOT_JOINT_STATE_HOLD);
}

void CLinkbotI::openGripper(double angle) {
    openGripperNB(angle);
    moveWait();
}

void CLinkbotI::openGripperNB(double angle) {
    moveNB(-angle/2.0, 0, -angle/2.0);
}

void CLinkbotI::driveAngle(double angle) {
    driveAngleNB(angle);
    moveWait(0x05);
}

void CLinkbotI::driveAngleNB(double angle) {
    moveNB(angle, 0, -angle);
}

void CLinkbotI::driveBackward(double angle) {
    driveBackwardNB(angle);
    moveWait(0x05);
}

void CLinkbotI::driveBackwardNB(double angle) {
    moveNB(-angle, 0, angle);
}

void CLinkbotI::driveDistance(double distance, double radius) {
    driveDistanceNB(distance, radius);
    moveWait(0x05);
}

void CLinkbotI::driveDistanceNB(double distance, double radius) {
    double angle = (distance/radius)*180/M_PI;
    moveNB(angle, 0, -angle);
}

void CLinkbotI::driveForeverNB() {
    Linkbot::moveContinuous(0x05, 1, 0, -1);
}

void CLinkbotI::driveForward(double angle) {
    driveForwardNB(angle);
    moveWait(0x05);
}

void CLinkbotI::driveForwardNB(double angle) {
    moveNB(angle, 0, -angle);
}

void CLinkbotI::driveTime(double time) {
    driveTimeNB(time);
    moveWait(0x05);
}

void CLinkbotI::driveTimeNB(double time) {
    Linkbot::setJointStates(0x05,
        LINKBOT_JOINT_STATE_MOVING, 1, time, LINKBOT_JOINT_STATE_HOLD,
        LINKBOT_JOINT_STATE_MOVING, 1, time, LINKBOT_JOINT_STATE_HOLD,
        LINKBOT_JOINT_STATE_MOVING, -1, time, LINKBOT_JOINT_STATE_HOLD);
}

void CLinkbotI::turnLeft(double angle, double radius, double tracklength)
{
    turnLeftNB(angle, radius, tracklength);
    moveWait(0x05);
}

void CLinkbotI::turnLeftNB(double angle, double radius, double tracklength) {
    auto theta = angle*M_PI/180.0;
    auto phi = tracklength * theta / (2*radius);
    phi *= 180/M_PI;
    moveNB(-phi, 0, -phi);
}

void CLinkbotI::turnRight(double angle, double radius, double tracklength) {
    turnRightNB(angle, radius, tracklength);
    moveWait(0x05);
}

void CLinkbotI::turnRightNB(double angle, double radius, double tracklength) {
    turnLeftNB(-angle, radius, tracklength);
}

} // namespace barobo
