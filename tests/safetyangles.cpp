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
#include <iostream>

#include <cassert>

void testSafetyAngles (std::string serialId) {
    double t = 0;
    try {
        barobo::Linkbot linkbot { serialId };

        std::cout << "Linkbot " << serialId << " constructed\n";

        double a[] = {-0.5, 0.5, 5.75};
        double b[] = {0, 0, 0};

        std::cout << "Setting safety angles: " << a[0] << ", " << a[1] << ", " << a[2] << '\n';
        linkbot.setJointSafetyAngles(0x07, a[0], a[1], a[2]);
        linkbot.getJointSafetyAngles(b[0], b[1], b[2]);
        std::cout << "Safety angles are now: " << b[0] << ", " << b[1] << ", " << b[2] << '\n';

        assert(std::equal(a, a + 3, b));
    }
    catch (std::exception& e) {
        std::cout << std::hex;
        // FIXME: This serial ID should be information baked into e.what() in
        // some cases.
        std::cout << "(" << serialId << ") error running test routine: "
                  << e.what() << '\n';
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <serial-id>\n", argv[0]);
        return 1;
    }

    auto serialId = std::string{argv[1]};
    assert(4 == serialId.size());

    testSafetyAngles(serialId);

    return 0;
}
