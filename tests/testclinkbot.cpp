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

int main() {
    auto linkbot = barobo::CLinkbot("ZRG6");

    // Get the accelerometer data
    double x, y, z;
    linkbot.getAccelerometerData(x, y, z);
    std::cout << "Accelerometer data: " << x << " " << y << " " <<z << std::endl;

    // Get the battery voltage
    double v;
    linkbot.getBatteryVoltage(v);
    std::cout << "Battery Voltage: " << v << std::endl;

    // Get the form factor
    LinkbotFormFactor form;
    linkbot.getFormFactor(form);
    std::cout << "Form factor: " << form << std::endl;

    // Get joint angles
    for ( LinkbotJoint n : {LINKBOT_JOINT_ONE, LINKBOT_JOINT_TWO, LINKBOT_JOINT_THREE} ) {
        double angle;
        linkbot.getJointAngle(n, angle);
        std::cout << "Joint angle: " << n << ": " << angle << std::endl;
    }

    return 0;
}
