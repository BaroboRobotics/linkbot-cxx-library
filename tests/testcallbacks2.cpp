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
        printf("Usage: %s <serial-id>\n", argv[0]);
        return 1;
    }
    
    auto linkbot = barobo::CLinkbot(argv[1]);

    // Test buttons 
    linkbot.setButtonEventCallback( [&] (LinkbotButton button, LinkbotButtonState state, int timestamp) {
        std::cout << button <<" "<< state <<" "<< timestamp << std::endl;
    });
    std::cout << "Button callbacks enabled. Press ENTER to continue:" << std::endl;
    getchar();
    linkbot.setButtonEventCallback(nullptr);

    // Test accelerometer
    linkbot.setAccelerometerEventCallback( [&] (double x, double y, double z, int timestamp) {
        std::cout << x <<" "<< y <<" "<< z <<" "<< timestamp << std::endl;
    });
    std::cout << "Accel callbacks enabled. Press ENTER to continue:" << std::endl;
    getchar();
    linkbot.setAccelerometerEventCallback(nullptr);

    // Test encoders
    linkbot.setEncoderEventCallback( [&] (int joint, double angle, int timestamp) {
        std::cout << joint <<" "<< angle <<" "<< timestamp << std::endl;
    }, 2.0);
    std::cout << "Encoder callbacks enabled. Press ENTER to continue:" << std::endl;
    getchar();
    linkbot.setEncoderEventCallback(nullptr, 0);

    return 0;
}
