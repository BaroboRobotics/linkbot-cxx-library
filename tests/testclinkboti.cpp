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

#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <serial-id>\n", argv[0]);
        return 1;
    }
    
    barobo::CLinkbotI linkbot {argv[1]};

    std::cout << "Drive Angle: 90 degrees..." << std::endl;
    linkbot.driveAngle(90);

    std::cout << "Drive backward 90 degrees..." << std::endl;
    linkbot.driveBackward(90);

    std::cout << "Drive forward 6 inches..." << std::endl;
    linkbot.driveDistance(6, 3.5/2.0);

    std::cout << "Drive for 3 seconds..." << std::endl;
    linkbot.driveTime(3);

    std::cout << "Driving for 3 seconds and sleeping for 5..." << std::endl;
    linkbot.driveTimeNB(3);

    using std::this_thread::sleep_for;
    using std::chrono::seconds;
    sleep_for(seconds(5));

    std::cout << "Turn left 90 degrees..." << std::endl;
    linkbot.turnLeft(90, 3.5/2, 3.7);

    std::cout << "Turn right 90 degrees..." << std::endl;
    linkbot.turnRight(90, 3.5/2, 3.7);

    std::cout << "accelJointAngleNB(), 360 degrees, 30 deg/s/s\n";
    linkbot.accelJointAngleNB(LINKBOT_JOINT_ONE, 30, 360);
    linkbot.moveWait();

    std::cout<< "accelJointTimeNB(), 30 deg/s/s, 4 seconds\n";
    linkbot.accelJointTimeNB(LINKBOT_JOINT_ONE, 30, 4);
    linkbot.moveWait();

    std::cout<< "accelJointToVelocityNB(), 30 deg/s/s, 180deg/s\n";
    linkbot.accelJointToVelocityNB(LINKBOT_JOINT_ONE, 30, 90);
    sleep_for(seconds(5));

    return 0;
}
