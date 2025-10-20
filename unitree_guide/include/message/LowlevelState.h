/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "interface/CmdPanel.h"
#include <iostream>

struct MotorState {
  unsigned int mode; //实际电机模式
  float q;
  float dq;
  float ddq;
  float tauEst;

  MotorState() {
    q = 0;
    dq = 0;
    ddq = 0;
    tauEst = 0;
  }
};

struct IMU {
  float quaternion[4]; // w, x, y, z
  float gyroscope[3];
  float accelerometer[3];

  IMU() {
    for (int i = 0; i < 3; i++) {
      quaternion[i] = 0;
      gyroscope[i] = 0;
      accelerometer[i] = 0;
    }
    quaternion[3] = 0;
  }

  RotMat getRotMat() {
    Quat quat;
    quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
    return quatToRotMat(quat);
  }

  Vec3 getAcc() {
    Vec3 acc;
    acc << accelerometer[0], accelerometer[1], accelerometer[2];
    return acc;
  }

  Vec3 getGyro() {
    Vec3 gyro;
    gyro << gyroscope[0], gyroscope[1], gyroscope[2];
    return gyro;
  }

  Quat getQuat() {
    Quat q;
    q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
    return q;
  }
};

struct LowlevelState {
  IMU imu;
  MotorState motorState[12];
  UserCommand userCmd;
  UserValue userValue;

  Vec34 getQ() {
    Vec34 qLegs;
    for (int i = 0; i < 4; ++i) {
      qLegs.col(i)(0) = motorState[3 * i].q;
      qLegs.col(i)(1) = motorState[3 * i + 1].q;
      qLegs.col(i)(2) = motorState[3 * i + 2].q;
    }
    return qLegs;
  }

  Vec34 getQd() {
    Vec34 qdLegs;
    for (int i = 0; i < 4; ++i) {
      qdLegs.col(i)(0) = motorState[3 * i].dq;
      qdLegs.col(i)(1) = motorState[3 * i + 1].dq;
      qdLegs.col(i)(2) = motorState[3 * i + 2].dq;
    }
    return qdLegs;
  }

  RotMat getRotMat() { return imu.getRotMat(); }

  Vec3 getAcc() { return imu.getAcc(); }

  Vec3 getGyro() { return imu.getGyro(); }

  Vec3 getAccGlobal() { return getRotMat() * getAcc(); }

  Vec3 getGyroGlobal() { return getRotMat() * getGyro(); }

  double getYaw() { return rotMatToRPY(getRotMat())(2); }

  double getDYaw() { return getGyroGlobal()(2); }

  void setQ(Vec12 q) {
    for (int i = 0; i < 12; ++i) {
      motorState[i].q = q(i);
    }
  }
};

struct LowlevelStateHexapod {
  IMU imu;
  MotorState motorState[18];
  UserCommand userCmd;
  UserValue userValue;
  Vec18 getQ2() {
    Vec18 q;
    q(0) = motorState[0].q;
    q(1) = motorState[1].q;
    q(2) = motorState[2].q;
    q(3) = motorState[3].q;
    q(4) = motorState[4].q;
    q(5) = motorState[5].q;
    q(6) = motorState[6].q;
    q(7) = motorState[7].q;
    q(8) = motorState[8].q;
    q(9) = motorState[9].q;
    q(10) = motorState[10].q;
    q(11) = motorState[11].q;
    q(12) = motorState[12].q;
    q(13) = motorState[13].q;
    q(14) = motorState[14].q;
    q(15) = motorState[15].q;
    q(16) = motorState[16].q;
    q(17) = motorState[17].q;

    return q;
  }
  Vec18 getQd2() {
    Vec18 qd;
    for (int i = 0; i < 18; ++i) {
      qd(i) = motorState[i].dq;
    }
    return qd;
  }

  RotMat getRotMat() { return imu.getRotMat(); }

  Vec3 getAcc() { return imu.getAcc(); }

  Vec3 getGyro() { return imu.getGyro(); }

  Vec3 getAccGlobal() { return getRotMat() * getAcc(); }

  Vec3 getGyroGlobal() { return getRotMat() * getGyro(); }

  double getYaw() { return rotMatToRPY(getRotMat())(2); }

  double getDYaw() { return getGyroGlobal()(2); }

  void setQ(Vec18 q) {
    for (int i = 0; i < 18; ++i) {
      motorState[i].q = q(i);
    }
  }
};

#endif // LOWLEVELSTATE_HPP