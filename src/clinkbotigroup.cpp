
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

namespace barobo {

CLinkbotIGroup::CLinkbotIGroup()
{ }

void CLinkbotIGroup::closeGripper()
{
    for ( auto& kv : mRobots ) {
        kv.second->closeGripperNB();
    }
    moveWait();
}

void CLinkbotIGroup::closeGripperNB()
{
    for ( auto& kv : mRobots ) {
        kv.second->closeGripperNB();
    }
}

void CLinkbotIGroup::openGripper(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->openGripperNB(angle);
    }
    moveWait();
}

void CLinkbotIGroup::openGripperNB(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->openGripperNB(angle);
    }
}

void CLinkbotIGroup::driveAngle(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveAngleNB(angle);
    }
    moveWait();
}

void CLinkbotIGroup::driveAngleNB(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveAngleNB(angle);
    }
}

void CLinkbotIGroup::driveBackward(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveBackwardNB(angle);
    }
    moveWait();
}

void CLinkbotIGroup::driveBackwardNB(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveBackwardNB(angle);
    }
}

void CLinkbotIGroup::driveDistance(double distance, double radius)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveDistanceNB(distance, radius);
    }
    moveWait();
}

void CLinkbotIGroup::driveDistanceNB(double distance, double radius)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveDistanceNB(distance, radius);
    }
}

void CLinkbotIGroup::driveForeverNB()
{
    for ( auto& kv : mRobots ) {
        kv.second->driveForeverNB();
    }
}

void CLinkbotIGroup::driveForward(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveForwardNB(angle);
    }
    moveWait();
}

void CLinkbotIGroup::driveForwardNB(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveForwardNB(angle);
    }
}

void CLinkbotIGroup::driveTime(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveTimeNB(angle);
    }
    moveWait();
}

void CLinkbotIGroup::driveTimeNB(double angle)
{
    for ( auto& kv : mRobots ) {
        kv.second->driveTimeNB(angle);
    }
}

void CLinkbotIGroup::turnLeft(double angle, double radius, double tracklength)
{
    for ( auto& kv : mRobots ) {
        kv.second->turnLeftNB(angle, radius, tracklength);
    }
    moveWait();
}

void CLinkbotIGroup::turnLeftNB(double angle, double radius, double tracklength)
{
    for ( auto& kv : mRobots ) {
        kv.second->turnLeftNB(angle, radius, tracklength);
    }
}

void CLinkbotIGroup::turnRight(double angle, double radius, double tracklength)
{
    for ( auto& kv : mRobots ) {
        kv.second->turnRightNB(angle, radius, tracklength);
    }
    moveWait();
}

void CLinkbotIGroup::turnRightNB(double angle, double radius, double tracklength)
{
    for ( auto& kv : mRobots ) {
        kv.second->turnRightNB(angle, radius, tracklength);
    }
}

} // namespace barobo
