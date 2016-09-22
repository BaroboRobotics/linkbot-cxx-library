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

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <serial-id>*\n", argv[0]);
        return 1;
    }

    auto group = barobo::CLinkbotGroup{};
    std::vector<barobo::CLinkbot> linkbots;

    for(auto i = 1; i < argc; i++) {
        linkbots.emplace_back(argv[i]);
        group.addRobots(linkbots.back());
    }

    // Scale the frequency from 220 to 440
    std::cout << "Slurring buzzer from 220 Hz to 440 Hz..." << std::endl;
    for(int i = 220; i < 440; i+=10) {
        group.setBuzzerFrequencyOn(i);
    }
    group.setBuzzerFrequencyOff();

    // Move slow-
    std::cout << "Moving 90 degrees slow...\n";
    group.setJointSpeeds(30, 30, 30);
    group.move(90, 90, 90);

    std::cout << "Moving 90 degrees fast...\n";
    group.setJointSpeeds(180, 180, 180);
    group.move(90, 90, 90);

    group.setJointSpeeds(90, 90, 90);

    std::cout << "Moving joint 1...\n";
    group.moveJoint(LINKBOT_JOINT_ONE, 90);

    std::cout << "Moving joint 2...\n";
    group.moveJoint(LINKBOT_JOINT_ONE, 90);

    std::cout << "Moving joint 3...\n";
    group.moveJoint(LINKBOT_JOINT_ONE, 90);

    std::cout << "Reset to zero...\n";
    group.resetToZero();

    std::cout << "Stopping all motors...\n";
    group.stop();
    
    return 0;
}
