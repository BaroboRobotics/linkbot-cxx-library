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

#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <stdint.h>

namespace barobo {

    using PlotData = std::array< std::vector<double>, 6 >;
    
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
    virtual void getAccelerometer (int& timestamp, double&, double&, double&);
    virtual std::vector<int> getAdcRaw();
    virtual void getBatteryVoltage(double& voltage);
    virtual void getFormFactor(LinkbotFormFactor& form);
    virtual void getJointAngles (int& timestamp, double&, double&, double&);
    virtual void getJointSpeeds(double&, double&, double&);
    virtual void getJointStates(int& timestamp,
                        LinkbotJointState& s1,
                        LinkbotJointState& s2,
                        LinkbotJointState& s3);
    virtual void getLedColor (int&, int&, int&);
    virtual void getVersionString (std::string& v);
    virtual void getSerialId(std::string& serialId);
    virtual void getJointSafetyThresholds(int&, int&, int&);
    virtual void getJointSafetyAngles(double&, double&, double&);

    /* SETTERS */
    virtual void resetEncoderRevs();
    virtual void setBuzzerFrequency (double);
    virtual void setJointAccelI(int mask, double, double, double);
    virtual void setJointAccelF(int mask, double, double, double);
    virtual void setJointSpeeds (int mask, double, double, double);
    virtual void setJointStates(
        int mask,
        LinkbotJointState s1, double d1,
        LinkbotJointState s2, double d2,
        LinkbotJointState s3, double d3);
    virtual void setJointStates(
        int mask,
        LinkbotJointState s1, double d1, double timeout1, LinkbotJointState end1,
        LinkbotJointState s2, double d2, double timeout2, LinkbotJointState end2,
        LinkbotJointState s3, double d3, double timeout3, LinkbotJointState end3
        );
    virtual void setLedColor (int, int, int);
    virtual void setJointSafetyThresholds(int mask, int t1=100, int t2=100, int t3=100);
    virtual void setJointSafetyAngles(int mask, double t1=10, double t2=10, double t3=10);

    /* MOVEMENT */
    // Member functions take angles in degrees.
    // All functions are non-blocking. Use moveWait() to wait for non-blocking
    // movement functions.
    virtual void drive (int mask, double, double, double);
    virtual void driveTo (int mask, double, double, double);
    virtual void move (int mask, double, double, double);
    // moveContinuous takes three angular speed coefficients. Use -1 to move
    // a motor backward, +1 to move it forward.
    virtual void moveAccel(int mask, int relativeMask,
        double omega0_i, double timeout0, LinkbotJointState endstate0,
        double omega1_i, double timeout1, LinkbotJointState endstate1,
        double omega2_i, double timeout2, LinkbotJointState endstate2);
    virtual void moveContinuous (int mask, double, double, double);
    virtual void moveTo (int mask, double, double, double);
    virtual void moveSmooth(int mask, int relativeMask, double a0, double a1, double a2);
    virtual void moveWait(int mask);
    virtual void motorPower(int mask, int m1, int m2, int m3);
    virtual void stop (int mask = 0x07);

    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    virtual void setButtonEventCallback (LinkbotButtonEventCallback, void* userData);
    // cb function params: button, button state, timestamp(millis)
    virtual void setButtonEventCallback (std::function<void(LinkbotButton, LinkbotButtonState, int)>);

    virtual void setEncoderEventCallback (LinkbotEncoderEventCallback, double granularity, void* userData);
    // cb function params: joint number, angle, timestamp
    virtual void setEncoderEventCallback (std::function<void(int, double, int)>, double granularity);

    virtual void setAccelerometerEventCallback (LinkbotAccelerometerEventCallback, void* userData);
    // cb function params: x, y, z, timestamp
    virtual void setAccelerometerEventCallback (std::function<void(double, double, double, int)>);

    virtual void setConnectionTerminatedCallback (LinkbotConnectionTerminatedCallback, void* userData);

    /* MISC */
    virtual void writeEeprom(uint32_t address, const uint8_t* data, size_t size);
    virtual void readEeprom(uint32_t address, size_t recvsize, uint8_t* buffer);
    virtual void writeTwi(uint32_t address, const uint8_t* data, size_t size);
    virtual void readTwi(uint32_t address, size_t recvsize, uint8_t* buffer);
    virtual void writeReadTwi(
        uint32_t address,
        const uint8_t* sendbuf,
        size_t sendsize,
        uint8_t* recvbuf,
        size_t recvsize);

private:
    struct Impl;
    Impl* m;
};

class CLinkbot : public Linkbot {
public:
    explicit CLinkbot (const std::string& serialId = "LOCL");

    /* GETTERS */

    virtual void getAccelerometerData(double &x, double &y, double &z);
    virtual void getBatteryVoltage(double &voltage);
    //void getDistance(double &distance, double radius);
    virtual void getFormFactor(LinkbotFormFactor& form);
    virtual void getJointAngle(LinkbotJoint id, double &angle);
    virtual void getJointAngles(double &angle1, double &angle2, double &angle3);
    //void getJointAngleInstant(LinkbotJoint id, double &angle);
    //void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
    //void getJointSafetyAngle(double &angle);
    //void getJointSafetyAngleTimeout(double &timeout);
    virtual void getJointSpeed(LinkbotJoint id, double &speed);
    virtual void getJointSpeedRatio(LinkbotJoint id, double &ratio);
    virtual void getJointSpeeds(double &speed1, double &speed2, double &speed3);
    virtual void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    virtual void getLEDColorRGB(int &r, int &g, int &b);
    //void getLEDColor(char color[]);

    /* SETTERS */
    virtual void setBuzzerFrequency(int frequency, double time);
    virtual void setBuzzerFrequencyOn(int frequency);
    virtual void setBuzzerFrequencyOff();
    virtual void setJointMovementStateNB(LinkbotJoint id, LinkbotDirection dir);
    virtual void setJointMovementStateTime(LinkbotJoint id, LinkbotDirection dir, double seconds);
    virtual void setJointMovementStateTimeNB(LinkbotJoint id, LinkbotDirection dir, double seconds);
    #if 0 //TODO 
    virtual void setJointSafetyAngle(double angle);
    virtual void setJointSafetyAngleTimeout(double timeout);
    #endif
    virtual void setJointSpeed(LinkbotJoint id, double speed);
    virtual void setJointSpeeds(double speed1, double speed2, double speed3);
    virtual void setJointSpeedRatio(LinkbotJoint id, double ratio);
    virtual void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    virtual void setJointPower(LinkbotJoint id, double power);
    virtual void setLEDColorRGB(int r, int g, int b);
    //void setLEDColor(char *color);
    virtual void setMotorPowers(double p1, double p2, double p3);
    virtual void setMovementStateNB( LinkbotDirection dir1,
            LinkbotDirection dir2,
            LinkbotDirection dir3);
    virtual void setMovementStateTime( LinkbotDirection dir1,
            LinkbotDirection dir2,
            LinkbotDirection dir3,
            double seconds);
    virtual void setMovementStateTimeNB( LinkbotDirection dir1,
            LinkbotDirection dir2,
            LinkbotDirection dir3,
            double seconds);
    virtual void setSpeed(double speed, double radius);

    /* MOVEMENT */
    virtual void accelJointAngleNB(LinkbotJoint id, double acceleration, double angle);
    virtual void accelJointTimeNB(LinkbotJoint id, double acceleration, double time);
    virtual void accelJointToVelocityNB(LinkbotJoint id, double acceleration, double speed);
    virtual void accelJointToMaxSpeedNB(LinkbotJoint id, double acceleration);
    virtual void driveAccelJointTimeNB(double radius, double acceleration,
            double time);
    virtual void driveAccelToVelocityNB(double radius, double acceleration,
            double velocity);
    virtual void driveAccelToMaxSpeedNB(double radius, double acceleration);
    virtual void driveAccelDistanceNB(double radius, double acceleration, 
            double distance);
#if 0 // TODO
    virtual void closeGripper();
    virtual void closeGripperNB();
    virtual void holdJoint(LinkbotJoint id);
    virtual void holdJoints();
    virtual void holdJointsAtExit();
    int isMoving(int mask=0x07);
    int isConnected();
#endif
    virtual void move(double j1, double j2, double j3);
    virtual void moveNB(double j1, double j2, double j3);
    virtual void moveWait(int mask=0x07);
    virtual void moveForeverNB();
    virtual void moveJoint(LinkbotJoint id, double angle);
    virtual void moveJointNB(LinkbotJoint id, double angle);
    virtual void moveJointForeverNB(LinkbotJoint id);
    virtual void moveJointTime(LinkbotJoint id, double time);
    virtual void moveJointTimeNB(LinkbotJoint id, double time);
    virtual void moveJointTo(LinkbotJoint id, double angle);
    virtual void moveJointToNB(LinkbotJoint id, double angle);
#if 0
    virtual void moveJointToByTrackPos(LinkbotJoint id, double angle);
    virtual void moveJointToByTrackPosNB(LinkbotJoint id, double angle);
#endif
    virtual void moveJointWait(LinkbotJoint id);
    virtual void moveTime(double time);
    virtual void moveTimeNB(double time);
    virtual void moveTo(double angle1, double angle2, double angle3);
    virtual void moveToNB(double angle1, double angle2, double angle3);
#if 0 // TODO
    virtual void moveToByTrackPos(double angle1, double angle2, double angle3);
    virtual void moveToByTrackPosNB(double angle1, double angle2, double angle3);
#endif
    virtual void moveToZero();
    virtual void moveToZeroNB();
#if 0
    virtual void openGripper(double angle);
    virtual void openGripperNB(double angle);
#endif
    virtual void relaxJoint(LinkbotJoint id);
    virtual void relaxJoints();
    virtual void resetToZero();
    virtual void resetToZeroNB();
    virtual void stop(int mask = 0x07);
    virtual void stopOneJoint(LinkbotJoint id);

    /* CALLBACKS */
    // Passing a null pointer as the first parameter of those three functions
    // will disable its respective events.
    virtual void setButtonEventCallback (LinkbotButtonEventCallback, void* userData);
    virtual void setButtonEventCallback (std::function<void(LinkbotButton, LinkbotButtonState, int)>);

    virtual void setEncoderEventCallback (LinkbotEncoderEventCallback, double granularity, void* userData);
    virtual void setEncoderEventCallback (std::function<void(int, double, int)>, double granularity);

    virtual void setAccelerometerEventCallback (LinkbotAccelerometerEventCallback, void* userData);
    virtual void setAccelerometerEventCallback (std::function<void(double, double, double, int)>);

    /* MISC */
    virtual void delaySeconds(double seconds);

#if 0
    /*
    virtual void enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
    virtual void disableButtonCallback();
    */
    virtual void blinkLED(double delay, int numBlinks);
#endif
    virtual void recordAnglesBegin(
    //  double timeInterval = 0.1,
    //  int mask = 0x07,
    //  int shiftData = 1
    );
    PlotData recordAnglesEnd();
#if 0
    virtual void recordDistanceBegin(
        LinkbotJoint id,
        robotRecordData_t &time,
        robotRecordData_t &distance,
        double radius,
        double timeInterval = 0.1,
        int shiftData = 1);
    virtual void recordDistanceEnd(LinkbotJoint id, int &num);

    virtual void recordAnglesBegin2(
        robotRecordData_t &time,
        robotRecordData_t &angle1,
        robotRecordData_t &angle2,
        robotRecordData_t &angle3,
        int shiftData = 1);
    virtual void recordAnglesEnd2(int &num);
    virtual void recordDistanceBegin2(
        LinkbotJoint id,
        robotRecordData_t &time,
        robotRecordData_t &distance,
        double radius,
        int shiftData = 1);
    virtual void recordDistanceEnd2(LinkbotJoint id, int &num);
    virtual void recordDistanceOffset(double distance);
    virtual void recordNoDataShift();
    virtual void enableRecordDataShift();
    virtual void disableRecordDataShift();
    virtual void delaySeconds(int seconds);
    virtual void systemTime(double &time);

#endif // TODO

    std::string _serialId() { return mSerialId; }

    PlotData _plotData;

protected:
    std::string mSerialId;
};

class CLinkbotGroup {
public:
    explicit CLinkbotGroup();

    virtual void addRobot(CLinkbot& robot);

    // SETTERS 
    virtual void setBuzzerFrequencyOn(int frequency);
    virtual void setBuzzerFrequencyOff();
    virtual void setJointSpeed(LinkbotJoint id, double speed);
    virtual void setJointSpeeds(double speed1, double speed2, double speed3);
    virtual void setJointSpeedRatio(LinkbotJoint id, double ratio);
    virtual void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    virtual void setJointPower(LinkbotJoint id, double power);
    virtual void setLEDColorRGB(int r, int g, int b);
    virtual void setMotorPowers(double p1, double p2, double p3);
    virtual void setSpeed(double speed, double radius);

    // MOVEMENT
    virtual void move(double j1, double j2, double j3);
    virtual void moveNB(double j1, double j2, double j3);
    virtual void moveWait(int mask=0x07);
    virtual void moveJoint(LinkbotJoint id, double angle);
    virtual void moveJointNB(LinkbotJoint id, double angle);
    virtual void moveJointWait(LinkbotJoint id);
    virtual void moveTo(double angle1, double angle2, double angle3);
    virtual void moveToNB(double angle1, double angle2, double angle3);
    virtual void resetToZero();
    virtual void resetToZeroNB();
    virtual void stop(int mask = 0x07);

private:
    std::map<std::string, CLinkbot*> mRobots;
};

class CLinkbotI : public CLinkbot {
public:
    explicit CLinkbotI(const std::string& serialId = "LOCL");

    virtual void driveAngle(double angle);
    virtual void driveAngleNB(double angle);
    virtual void driveBackward(double angle);
    virtual void driveBackwardNB(double angle);
    virtual void driveDistance(double distance, double radius);
    virtual void driveDistanceNB(double distance, double radius);
    virtual void driveForeverNB();
    virtual void driveForward(double angle);
    virtual void driveForwardNB(double angle);
    virtual void driveTime(double time);
    virtual void driveTimeNB(double time);
    virtual void turnLeft(double angle, double radius, double tracklength);
    virtual void turnLeftNB(double angle, double radius, double tracklength);
    virtual void turnRight(double angle, double radius, double tracklength);
    virtual void turnRightNB(double angle, double radius, double tracklength);

};

class CLinkbotL: public CLinkbot {
public:
    explicit CLinkbotL(const std::string& serialId = "LOCL");
};

template <typename T>
std::vector<std::tuple<T, T>> pair(T xs, T ys) {
    return {std::make_tuple(xs, ys)};
}

template <typename T, typename... Args>
std::vector<std::tuple<T, T>> pair(T xs, T ys, Args... args) {
    auto base = pair(xs, ys);
    auto tail = pair(args...);
    base.insert(base.end(), tail.begin(), tail.end());
    return base;
}

void sendToPrex(std::string json);

template <typename T>
void scatterPlot(T xs, T ys) {
    std::ostringstream buffer;
    buffer << '[';
    buffer << "{\n";
    buffer << "  \"x\": [";
    for (auto j = xs.begin(); j != xs.end(); ++j) {
        if( j != xs.begin() ) {
            buffer << ", ";
        }
        buffer << *j;
    }
    buffer << "],\n";
    buffer << "  \"y\": [";
    for (auto j = ys.begin(); j != ys.end(); ++j) {
        if ( j != ys.begin() ) {
            buffer << ", ";
        }
        buffer << *j;
    }
    buffer << "],\n";
    buffer << "  \"type\": \"scatter\"\n}";
    buffer << "]\n";
    std::cout << buffer.str();
    sendToPrex(buffer.str());
}

template <typename T, typename... Args>
void scatterPlot(T xs, T ys, Args... args) {
    auto data = pair(xs, ys, args...);

    std::ostringstream buffer;
    buffer << '[';
    for(auto i = data.begin(); i!= data.end(); ++i) {
        if ( i != data.begin() ) {
            buffer << ",\n";
        }
        buffer << "{\n";
        auto xs = std::get<0>(*i);
        auto ys = std::get<1>(*i);
        buffer << "  \"x\": [";
        for (auto j = xs.begin(); j != xs.end(); ++j) {
            if( j != xs.begin() ) {
                buffer << ", ";
            }
            buffer << *j;
        }
        buffer << "],\n";
        buffer << "  \"y\": [";
        for (auto j = ys.begin(); j != ys.end(); ++j) {
            if ( j != ys.begin() ) {
                buffer << ", ";
            }
            buffer << *j;
        }
        buffer << "],\n";
        buffer << "  \"type\": \"scatter\"\n}";
    }
    buffer << "]\n";
    std::cout << buffer.str();
    sendToPrex(buffer.str());
}

void scatterPlot(PlotData data);
 
} // barobo

#endif
