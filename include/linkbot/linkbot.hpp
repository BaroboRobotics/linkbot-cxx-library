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

#ifndef LINKBOT_LINKBOT_HPP
#define LINKBOT_LINKBOT_HPP

#include <linkbot/linkbot.h>

#include <string>
#include <vector>
#include <stdint.h>

namespace barobo {

/* A C++03-compatible Linkbot API. */
class Linkbot {
public:
#if 0
    // Construct a Linkbot backed by a given WebSocket host and service. For
    // example, Linkbot{"127.0.0.1", "42010"} would attempt to start
    // communicating with a robot interface at ws://localhost:42010/.
    Linkbot (const std::string& host, const std::string& service);
#endif

    // Ask the daemon to resolve the given serial ID to a WebSocket host:service,
    // and construct a Linkbot backed by this WebSocket endpoint.
    explicit Linkbot (const std::string& serialId);

    ~Linkbot ();

private:
    // noncopyable
    Linkbot (const Linkbot&);
    Linkbot& operator= (const Linkbot&);
    void initJointEventCallback ();

public:
    // All member functions may throw a barobo::Error exception on failure.

    /* GETTERS */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void getAccelerometer (int& timestamp, double&, double&, double&);
    std::vector<int> getAdcRaw();
    void getBatteryVoltage(double& voltage);
    void getFormFactor(LinkbotFormFactor& form);
    void getJointAngles (int& timestamp, double&, double&, double&);
    void getJointSpeeds(double&, double&, double&);
    void getJointStates(int& timestamp,
                        LinkbotJointState& s1,
                        LinkbotJointState& s2,
                        LinkbotJointState& s3);
    void getLedColor (int&, int&, int&);
    void getVersionString (std::string& v);
    void getSerialId(std::string& serialId);
    void getJointSafetyThresholds(int&, int&, int&);
    void getJointSafetyAngles(double&, double&, double&);

    /* SETTERS */
    void resetEncoderRevs();
    void setBuzzerFrequency (double);
    void setJointAccelI(int mask, double, double, double);
    void setJointAccelF(int mask, double, double, double);
    void setJointSpeeds (int mask, double, double, double);
    void setJointStates(
        int mask,
        LinkbotJointState s1, double d1,
        LinkbotJointState s2, double d2,
        LinkbotJointState s3, double d3);
    void setJointStates(
        int mask,
        LinkbotJointState s1, double d1, double timeout1, LinkbotJointState end1,
        LinkbotJointState s2, double d2, double timeout2, LinkbotJointState end2,
        LinkbotJointState s3, double d3, double timeout3, LinkbotJointState end3
        );
    void setLedColor (int, int, int);
    void setJointSafetyThresholds(int mask, int t1=100, int t2=100, int t3=100);
    void setJointSafetyAngles(int mask, double t1=10, double t2=10, double t3=10);

    /* MOVEMENT */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    void drive (int mask, double, double, double);
    void driveTo (int mask, double, double, double);
    void move (int mask, double, double, double);
    // moveContinuous takes three angular speed coefficients. Use -1 to move
    // a motor backward, +1 to move it forward.
    void moveAccel(int mask, int relativeMask,
        double omega0_i, double timeout0, LinkbotJointState endstate0,
        double omega1_i, double timeout1, LinkbotJointState endstate1,
        double omega2_i, double timeout2, LinkbotJointState endstate2);
    void moveContinuous (int mask, double, double, double);
    void moveTo (int mask, double, double, double);
    void moveSmooth(int mask, int relativeMask, double a0, double a1, double a2);
    void moveWait(int mask);
    void motorPower(int mask, int m1, int m2, int m3);
    void stop (int mask = 0x07);

    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    void setButtonEventCallback (LinkbotButtonEventCallback, void* userData);
    void setEncoderEventCallback (LinkbotEncoderEventCallback, double granularity, void* userData);
    void setAccelerometerEventCallback (LinkbotAccelerometerEventCallback, void* userData);
    void setConnectionTerminatedCallback (LinkbotConnectionTerminatedCallback, void* userData);

    /* MISC */
    void writeEeprom(uint32_t address, const uint8_t* data, size_t size);
    void readEeprom(uint32_t address, size_t recvsize, uint8_t* buffer);
    void writeTwi(uint32_t address, const uint8_t* data, size_t size);
    void readTwi(uint32_t address, size_t recvsize, uint8_t* buffer);
    void writeReadTwi(
        uint32_t address,
        const uint8_t* sendbuf,
        size_t sendsize,
        uint8_t* recvbuf,
        size_t recvsize);

private:
    struct Impl;
    Impl* m;
};

} // barobo

#endif
