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

#ifndef LINKBOT_LINKBOTGROUP_HPP
#define LINKBOT_LINKBOTGROUP_HPP

#include "linkbot.hpp"

namespace barobo {

template<class T> void Group<T>::setBuzzerFrequencyOn(int frequency) {
    for ( auto& kv : mRobots ) {
        kv.second->setBuzzerFrequencyOn(frequency);
    }
}

template<class T> void Group<T>::setBuzzerFrequencyOff() {
    for ( auto& kv : mRobots ) {
        kv.second->setBuzzerFrequencyOff();
    }
}

template<class T> void Group<T>::setJointSpeed(LinkbotJoint id, double speed)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeed(id, speed);
    }
}

template<class T> void Group<T>::setJointSpeeds(double speed1, double speed2, double speed3)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeeds(speed1, speed2, speed3);
    }
}

template<class T> void Group<T>::setJointSpeedRatio(LinkbotJoint id, double ratio)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeedRatio(id, ratio);
    }
}

template<class T> void Group<T>::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointSpeedRatios(ratio1, ratio2, ratio3);
    }
}

template<class T> void Group<T>::setJointPower(LinkbotJoint id, double power)
{
    for ( auto& kv : mRobots ) {
        kv.second->setJointPower(id, power);
    }
}

template<class T> void Group<T>::setLEDColorRGB(int r, int g, int b)
{
    for ( auto& kv : mRobots ) {
        kv.second->setLEDColorRGB(r, g, b);
    }
}

template<class T> void Group<T>::setMotorPowers(double p1, double p2, double p3)
{
    for ( auto& kv : mRobots ) {
        kv.second->setMotorPowers(p1, p2, p3);
    }
}

template<class T> void Group<T>::setSpeed(double speed, double radius)
{
    for ( auto& kv : mRobots ) {
        kv.second->setSpeed(speed, radius);
    }
}

// Group Movement

template<class T> void Group<T>::holdJoint(LinkbotJoint id)
{
    for ( auto& kv : mRobots ) {
        kv.second->holdJoint(id);
    }
}

template<class T> void Group<T>::holdJoints()
{
    for ( auto& kv : mRobots ) {
        kv.second->holdJoints();
    }
}

template<class T> void Group<T>::holdJointsAtExit()
{
    for ( auto& kv : mRobots ) {
        kv.second->holdJointsAtExit();
    }
}

template<class T> void Group<T>::move(double j1, double j2, double j3)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveNB(j1, j2, j3);
    }
    for ( auto& kv : mRobots ) {
        kv.second->moveWait();
    }
}

template<class T> void Group<T>::moveNB(double j1, double j2, double j3)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveNB(j1, j2, j3);
    }
}

template<class T> void Group<T>::moveForeverNB()
{
    for ( auto& kv : mRobots ) {
        kv.second->moveForeverNB();
    }
}

template<class T> void Group<T>::moveJointForeverNB(LinkbotJoint id)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointForeverNB(id);
    }
}

template<class T> void Group<T>::moveJointTime(LinkbotJoint id, double time)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointTimeNB(id, time);
    }
    moveWait(1<<id);
}

template<class T> void Group<T>::moveJointTimeNB(LinkbotJoint id, double time)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointTimeNB(id, time);
    }
}

template<class T> void Group<T>::moveJointTo(LinkbotJoint id, double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointToNB(id, angle);
    }
    moveWait();
}

template<class T> void Group<T>::moveJointToNB(LinkbotJoint id, double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointToNB(id, angle);
    }
}

template<class T> void Group<T>::moveWait(int mask)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveWait(mask);
    }
}

template<class T> void Group<T>::moveJoint(LinkbotJoint id, double angle)
{
    moveJointNB(id, angle);
    moveWait(1<<id);
}

template<class T> void Group<T>::moveJointNB(LinkbotJoint id, double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointNB(id, angle);
    }
}

template<class T> void Group<T>::moveJointWait(LinkbotJoint id)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveJointWait(id);
    }
}

template<class T> void Group<T>::moveTime(double time)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveTimeNB(time);
    }
    moveWait();
}

template<class T> void Group<T>::moveTimeNB(double time)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveTimeNB(time);
    }
}

template<class T> void Group<T>::moveTo(double angle1, double angle2, double angle3)
{
    moveToNB(angle1, angle2, angle3);
    moveWait();
}

template<class T> void Group<T>::moveToNB(double angle1, double angle2, double angle3)
{
    for ( auto& kv : mRobots ) {
        kv.second->moveToNB(angle1, angle2, angle3);
    }
}

template<class T> void Group<T>::resetToZero()
{
    resetToZeroNB();
    moveWait();
}

template<class T> void Group<T>::resetToZeroNB()
{
    for ( auto& kv : mRobots ) {
        kv.second->resetToZeroNB();
    }
}

template<class T> void Group<T>::stop(int mask)
{
    for ( auto& kv : mRobots ) {
        kv.second->stop(mask);
    }
}

} // barobo

#endif
