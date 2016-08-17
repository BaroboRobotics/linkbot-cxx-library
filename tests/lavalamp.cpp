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
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <cassert>
#include <cmath>

#undef M_PI
#define M_PI 3.14159265358979323846

void sendNewColor(barobo::Linkbot& linkbot, double tim) {
    uint32_t red, green, blue;
    red = (sin(tim) + 1) * 127;
    green = (sin(tim + 2 * M_PI / 3) + 1) * 127;
    blue = (sin(tim + 4 * M_PI / 4) + 1) * 127;
    linkbot.setLedColor(red, green, blue);
}

void lavaLamp (std::string serialId) {
    double t = 0;
    try {
        barobo::Linkbot linkbot { serialId };
        std::cout << serialId << ": connected\n";

        while (1) {
            sendNewColor(linkbot, t);
            t += 0.05;
        }
    }
    catch (std::exception& e) {
        std::cout << std::hex;
        // FIXME: This serial ID should be information baked into e.what() in
        // some cases.
        std::cout << "(" << serialId << ") error setting color(" << t << "): "
                  << e.what() << '\n';
    }
}


int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <serial-id> [<serial-id> ...]\n", argv[0]);
        return 1;
    }

    // Get list of serial IDs from command line.
    std::vector<std::string> serialIds { argv + 1, argv + argc };

    // Ensure they all sorta look like serial IDs.
    assert(std::all_of(serialIds.cbegin(), serialIds.cend(),
                [] (const std::string& s) { return 4 == s.size(); }));

    std::vector<std::thread> lavaLampThreads;

    for (auto s : serialIds) {
        lavaLampThreads.emplace_back(lavaLamp, s);
    }

    for (auto& t: lavaLampThreads) {
        t.join();
    }

    return 0;
}
