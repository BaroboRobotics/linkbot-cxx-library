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

#include <map>
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

class CLinkbot {
public:
    explicit CLinkbot (const std::string& serialId = "LOCL");

    ~CLinkbot ();

    /* GETTERS */

    void getAccelerometerData(double &x, double &y, double &z);
    void getBatteryVoltage(double &voltage);
    //void getDistance(double &distance, double radius);
    void getFormFactor(LinkbotFormFactor& form);
    void getJointAngle(LinkbotJoint id, double &angle);
    void getJointAngles(double &angle1, double &angle2, double &angle3);
    //void getJointAngleInstant(LinkbotJoint id, double &angle);
    //void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
    //void getJointSafetyAngle(double &angle);
    //void getJointSafetyAngleTimeout(double &timeout);
    void getJointSpeed(LinkbotJoint id, double &speed);
    void getJointSpeedRatio(LinkbotJoint id, double &ratio);
    void getJointSpeeds(double &speed1, double &speed2, double &speed3);
    void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    void getLEDColorRGB(int &r, int &g, int &b);
    //void getLEDColor(char color[]);

    /* SETTERS */
    void setBuzzerFrequency(int frequency, double time);
    void setBuzzerFrequencyOn(int frequency);
    void setBuzzerFrequencyOff();
    #if 0 //TODO 
    void setJointMovementStateNB(LinkbotJoint id, robotJointState_t dir);
    void setJointMovementStateTime(LinkbotJoint id, robotJointState_t dir, double seconds);
    void setJointMovementStateTimeNB(LinkbotJoint id, robotJointState_t dir, double seconds);
    void setJointSafetyAngle(double angle);
    void setJointSafetyAngleTimeout(double timeout);
    #endif
    void setJointSpeed(LinkbotJoint id, double speed);
    void setJointSpeeds(double speed1, double speed2, double speed3);
    void setJointSpeedRatio(LinkbotJoint id, double ratio);
    void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    void setJointPower(LinkbotJoint id, double power);
    void setLEDColorRGB(int r, int g, int b);
    //void setLEDColor(char *color);
    void setMotorPowers(double p1, double p2, double p3);
#if 0 // TODO
    void setMovementStateNB( robotJointState_t dir1,
            robotJointState_t dir2,
            robotJointState_t dir3);
    void setMovementStateTime( robotJointState_t dir1,
            robotJointState_t dir2,
            robotJointState_t dir3,
            double seconds);
    void setMovementStateTimeNB( robotJointState_t dir1,
            robotJointState_t dir2,
            robotJointState_t dir3,
            double seconds);
#endif
    void setSpeed(double speed, double radius);

    /* MOVEMENT */
#if 0 // TODO
    void accelJointAngleNB(LinkbotJoint id, double acceleration, double angle);
    void accelJointTimeNB(LinkbotJoint id, double acceleration, double time);
    void accelJointToVelocityNB(LinkbotJoint id, double acceleration, double speed);
    void accelJointToMaxSpeedNB(LinkbotJoint id, double acceleration);
    void driveAccelJointTimeNB(double radius, double acceleration,
            double time);
    void driveAccelToVelocityNB(double radius, double acceleration,
            double velocity);
    void driveAccelToMaxSpeedNB(double radius, double acceleration);
    void driveAccelDistanceNB(double radius, double acceleration, 
            double distance);
    void closeGripper();
    void closeGripperNB();
    void holdJoint(LinkbotJoint id);
    void holdJoints();
    void holdJointsAtExit();
    int isMoving(int mask=0x07);
    int isConnected();
#endif
    void move(double j1, double j2, double j3);
    void moveNB(double j1, double j2, double j3);
    void moveWait(int mask=0x07);
#if 0 // TODO
    void moveForeverNB();
#endif
    void moveJoint(LinkbotJoint id, double angle);
    void moveJointNB(LinkbotJoint id, double angle);
#if 0
    void moveJointForeverNB(LinkbotJoint id);
    void moveJointTime(LinkbotJoint id, double time);
    void moveJointTimeNB(LinkbotJoint id, double time);
    void moveJointTo(LinkbotJoint id, double angle);
    void moveJointToNB(LinkbotJoint id, double angle);
    void moveJointToByTrackPos(LinkbotJoint id, double angle);
    void moveJointToByTrackPosNB(LinkbotJoint id, double angle);
#endif
    void moveJointWait(LinkbotJoint id);
#if 0
    void moveTime(double time);
    void moveTimeNB(double time);
#endif
    void moveTo(double angle1, double angle2, double angle3);
    void moveToNB(double angle1, double angle2, double angle3);
#if 0 // TODO
    void moveToByTrackPos(double angle1, double angle2, double angle3);
    void moveToByTrackPosNB(double angle1, double angle2, double angle3);
    void moveToZero();
    void moveToZeroNB();
    void openGripper(double angle);
    void openGripperNB(double angle);
    void relaxJoint(LinkbotJoint id);
    void relaxJoints();
#endif
    void resetToZero();
    void resetToZeroNB();
    void stop(int mask = 0x07);
#if 0
    void stopOneJoint(LinkbotJoint id);
#endif
    /* MISC */
    void delaySeconds(double seconds);

#if 0
    /*
    void enableButtonCallback(void* userdata, void (*buttonCallback)(void* data, int button, int buttonDown));
    void disableButtonCallback();
    */
    void blinkLED(double delay, int numBlinks);
    void recordAnglesBegin(
        robotRecordData_t &time,
        robotRecordData_t &angle1,
        robotRecordData_t &angle2,
        robotRecordData_t &angle3,
        double timeInterval = 0.1,
        int mask = 0x07,
        int shiftData = 1);
    void recordAnglesEnd(int &num);
    void recordDistanceBegin(
        LinkbotJoint id,
        robotRecordData_t &time,
        robotRecordData_t &distance,
        double radius,
        double timeInterval = 0.1,
        int shiftData = 1);
    void recordDistanceEnd(LinkbotJoint id, int &num);

    void recordAnglesBegin2(
        robotRecordData_t &time,
        robotRecordData_t &angle1,
        robotRecordData_t &angle2,
        robotRecordData_t &angle3,
        int shiftData = 1);
    void recordAnglesEnd2(int &num);
    void recordDistanceBegin2(
        LinkbotJoint id,
        robotRecordData_t &time,
        robotRecordData_t &distance,
        double radius,
        int shiftData = 1);
    void recordDistanceEnd2(LinkbotJoint id, int &num);
    void recordDistanceOffset(double distance);
    void recordNoDataShift();
    void enableRecordDataShift();
    void disableRecordDataShift();
    void delaySeconds(int seconds);
    void systemTime(double &time);

#endif // TODO

    std::string _serialId() { return mSerialId; }

protected:
    Linkbot *_l;
    std::string mSerialId;
};

class CLinkbotGroup {
public:
    explicit CLinkbotGroup();
    ~CLinkbotGroup();

    void addRobot(CLinkbot& robot);

    // SETTERS 
    void setBuzzerFrequencyOn(int frequency);
    void setBuzzerFrequencyOff();
    void setJointSpeed(LinkbotJoint id, double speed);
    void setJointSpeeds(double speed1, double speed2, double speed3);
    void setJointSpeedRatio(LinkbotJoint id, double ratio);
    void setJointSpeedRatios(double ratios1, double ratios2, double ratios3);
    void setJointPower(LinkbotJoint id, double power);
    void setLEDColorRGB(int r, int g, int b);
    void setMotorPowers(double p1, double p2, double p3);
    void setSpeed(double speed, double radius);

    // MOVEMENT
    void move(double j1, double j2, double j3);
    void moveNB(double j1, double j2, double j3);
    void moveWait(int mask=0x07);
    void moveJoint(LinkbotJoint id, double angle);
    void moveJointNB(LinkbotJoint id, double angle);
    void moveJointWait(LinkbotJoint id);
    void moveTo(double angle1, double angle2, double angle3);
    void moveToNB(double angle1, double angle2, double angle3);
    void resetToZero();
    void resetToZeroNB();
    void stop(int mask = 0x07);

private:
    std::map<std::string, CLinkbot*> mRobots;
};

class CLinkbotI : public CLinkbot {
public:
    explicit CLinkbotI(const std::string& serialId = "LOCL");
    ~CLinkbotI();

    void driveAngle(double angle);
    void driveAngleNB(double angle);
    void driveBackward(double angle);
    void driveBackwardNB(double angle);
    void driveDistance(double distance, double radius);
    void driveDistanceNB(double distance, double radius);
    void driveForeverNB();
    void driveForward(double angle);
    void driveForwardNB(double angle);
    void driveTime(double time);
    void driveTimeNB(double time);
    void turnLeft(double angle, double radius, double tracklength);
    void turnLeftNB(double angle, double radius, double tracklength);
    void turnRight(double angle, double radius, double tracklength);
    void turnRightNB(double angle, double radius, double tracklength);

};

class CLinkbotL: public CLinkbot {
public:
    explicit CLinkbotL(const std::string& serialId = "LOCL");
    ~CLinkbotL();
};

} // barobo

#endif
