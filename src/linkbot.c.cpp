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

#include <linkbot/linkbot.h>
#include <linkbot/linkbot.hpp>

#include <iostream>
#include <string>

#include <cstdio>
#include <cstring>

struct Linkbot {
#if 0
    Linkbot (const char* host, const char* service)
        : impl(std::string(host), std::string(service))
    {}
#endif

    Linkbot (const char* serialId)
        : impl(std::string(serialId))
    {}

    barobo::Linkbot impl;
};

#if 0
Linkbot* linkbotFromTcpEndpoint(const char* host, const char* service)
{
    try {
        return new Linkbot(host, service);
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return nullptr;
    }
}
#endif

Linkbot* linkbotFromSerialId(const char* serialId)
{
    try {
        return new Linkbot(serialId);
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return nullptr;
    }
}

void linkbotDelete(Linkbot* l)
{
    delete l;
}

#define LINKBOT_C_WRAPPER_FUNC_IMPL(cpp_name, ...) \
do \
{ \
    if (!l) { \
        return -1; \
    } \
    try { \
        l->impl. cpp_name (__VA_ARGS__); \
        return 0; \
    } \
    catch (std::exception& e) { \
        fprintf(stderr, "Runtime exception: %s\n", e.what()); \
        return -1; \
    } \
} while(0)

/* GETTERS */

int linkbotGetAccelerometer(Linkbot* l, int *timestamp, double *x, double *y,
                           double *z)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getAccelerometer, *timestamp, *x, *y, *z);
}

int linkbotGetBatteryVoltage(Linkbot* l, double* voltage)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getBatteryVoltage, *voltage);
}

int linkbotGetFormFactor(Linkbot* l, LinkbotFormFactor *form)
{
    *form = LINKBOT_FORM_FACTOR_I;
    LINKBOT_C_WRAPPER_FUNC_IMPL(getFormFactor, *form);
}

int linkbotGetJointAngles(Linkbot* l, int *timestamp, double *j1, double *j2, double *j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointAngles, *timestamp, *j1, *j2, *j3);
}

int linkbotGetJointSpeeds(Linkbot* l, double *s1, double *s2, double *s3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointSpeeds, *s1, *s2, *s3);
}

int linkbotGetJointStates(Linkbot* l, int *timestamp,
                          LinkbotJointState* j1,
                          LinkbotJointState* j2,
                          LinkbotJointState* j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointStates, *timestamp, *j1, *j2, *j3);
}

int linkbotGetLedColor(Linkbot* l, int *r, int *g, int *b)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getLedColor, *r, *g, *b);
}

int linkbotGetVersionString (Linkbot* l, char* v, unsigned n)
{
    if (!l || !n) {
        return -1;
    }
    std::string versionString;
    try {
        l->impl.getVersionString(versionString);
        strncpy(v, versionString.c_str(), n);
        v[n-1] = '\0';
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}

int linkbotGetSerialId(Linkbot* l, char* serialId)
{
    if (!l) {
        return -1;
    }
    std::string id;
    try {
        l->impl.getSerialId(id);
        memcpy(serialId, id.c_str(), 4);
        serialId[4] = 0;
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}

int linkbotGetJointSafetyThresholds(Linkbot* l, int* t1, int* t2, int* t3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointSafetyThresholds, *t1, *t2, *t3);
}

int linkbotGetJointSafetyAngles(Linkbot* l, double* t1, double* t2, double* t3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(getJointSafetyAngles, *t1, *t2, *t3);
}

/* SETTERS */

int linkbotSetAlphaI(Linkbot* l, int mask, double a1, double a2, double a3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointAccelI, mask, a1, a2, a3);
}

int linkbotSetAlphaF(Linkbot* l, int mask, double a1, double a2, double a3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointAccelF, mask, a1, a2, a3);
}

int linkbotResetEncoderRevs(Linkbot* l)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(resetEncoderRevs);
}

int linkbotSetBuzzerFrequency(Linkbot* l, float freq)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setBuzzerFrequency, freq);
}

int linkbotSetJointSpeeds(Linkbot* l, int mask, double j1, double j2,
                          double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointSpeeds, mask, j1, j2, j3);
}

int linkbotSetJointStates(Linkbot* l, int mask,
        LinkbotJointState s1, double d1,
        LinkbotJointState s2, double d2,
        LinkbotJointState s3, double d3
        )
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointStates, mask,
        s1, d1,
        s2, d2,
        s3, d3);
}

int linkbotSetJointStatesTimed(Linkbot* l, int mask,
        LinkbotJointState s1, double d1, double timeout1, LinkbotJointState end1,
        LinkbotJointState s2, double d2, double timeout2, LinkbotJointState end2,
        LinkbotJointState s3, double d3, double timeout3, LinkbotJointState end3
        )
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointStates, mask,
        s1, d1, timeout1, end1,
        s2, d2, timeout2, end2,
        s3, d3, timeout3, end3);
}

int linkbotSetLedColor(Linkbot* l, int r, int g, int b)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setLedColor, r, g, b);
}

int linkbotSetJointSafetyThresholds(Linkbot* l, int mask, int t1, int t2, int t3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointSafetyThresholds, mask, t1, t2, t3);
}

int linkbotSetJointSafetyAngles(Linkbot* l, int mask, double t1, double t2, double t3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(setJointSafetyAngles, mask, t1, t2, t3);
}

/* MOVEMENT */

int linkbotMoveAccel(Linkbot* l, int mask, int relativeMask,
    double a0, double timeout0, LinkbotJointState endstate0,
    double a1, double timeout1, LinkbotJointState endstate1,
    double a2, double timeout2, LinkbotJointState endstate2)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveAccel, mask, relativeMask,
            a0, timeout0, endstate0,
            a1, timeout1, endstate1,
            a2, timeout2, endstate2);
}

int linkbotMoveSmooth(Linkbot* l, int mask, int relativeMask,
    double a0, double a1, double a2)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveSmooth, mask, relativeMask, a0, a1, a2);
}

int linkbotMoveContinuous(Linkbot* l, int mask,
                          double d1,
                          double d2,
                          double d3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveContinuous, mask, d1, d2, d3);
}

int linkbotWriteEeprom(Linkbot* l, unsigned int address, const char *data, unsigned int size)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(writeEeprom, uint32_t(address), (uint8_t*)(data), size_t(size));
}

int linkbotDrive(Linkbot* l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(drive, mask, j1, j2, j3);
}

int linkbotDriveTo(Linkbot* l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(driveTo, mask, j1, j2, j3);
}

int linkbotMove(Linkbot* l, int mask, double j1, double j2, double j3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(move, mask, j1, j2, j3);
}

int linkbotMoveTo(Linkbot* l, int mask, double j1, double j2, double j3) {
    LINKBOT_C_WRAPPER_FUNC_IMPL(moveTo, mask, j1, j2, j3);
}

int linkbotMotorPower(Linkbot* l, int mask, int m1, int m2, int m3)
{
    LINKBOT_C_WRAPPER_FUNC_IMPL(motorPower, mask, m1, m2, m3);
}

int linkbotStop(Linkbot* l, int mask) {
    LINKBOT_C_WRAPPER_FUNC_IMPL(stop, mask);
}

/* CALLBACKS */

#define LINKBOT_DEF_EVENT_CALLBACK(cbname) \
int linkbotSet##cbname(Linkbot* l, Linkbot##cbname cb, void* userData) \
{ \
    if (!l) { \
        return -1; \
    } \
    try { \
        l->impl.set##cbname(cb, userData); \
        return 0; \
    } \
    catch (std::exception& e) { \
        fprintf(stderr, "Runtime exception: %s\n", e.what()); \
        return -1; \
    } \
}

LINKBOT_DEF_EVENT_CALLBACK(ButtonEventCallback)
//LINKBOT_DEF_EVENT_CALLBACK(EncoderEventCallback)
LINKBOT_DEF_EVENT_CALLBACK(JointEventCallback)
LINKBOT_DEF_EVENT_CALLBACK(AccelerometerEventCallback)
LINKBOT_DEF_EVENT_CALLBACK(ConnectionTerminatedCallback)

int linkbotSetEncoderEventCallback(Linkbot* l,
                                   LinkbotEncoderEventCallback cb,
                                   float granularity,
                                   void* userData)
{
    if (!l) {
        return -1;
    }
    try {
        l->impl.setEncoderEventCallback(cb, granularity, userData);
        return 0;
    }
    catch (std::exception& e) {
        fprintf(stderr, "Runtime exception: %s\n", e.what());
        return -1;
    }
}

#undef LINKBOT_DEF_EVENT_CALLBACK
