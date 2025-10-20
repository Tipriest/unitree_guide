/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/hexapodRobot.h"
#include <iostream>

// 获取机器人位置（这里简单地返回0号腿的足端位置作为参考）
Vec3 HexapodRobot::getX(LowlevelStateHexapod &state) {
  return getFootPosition(state, 0, FrameType::BODY);
}

// 获取所有足端相对于机器人位置参考点的向量
Vec34 HexapodRobot::getVecXP(LowlevelStateHexapod &state) {
  Vec3 x = getX(state);
  Vec34 vecXP, qLegs;
  qLegs = state.getQ();

  for (int i = 0; i < 4; ++i) {
    vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
  }
  return vecXP;
}
// 逆运动学：计算所有腿的关节角度
Vec12 HexapodRobot::getQ(const Vec34 &vecP, FrameType frame) {
  Vec12 q;
  for (int i = 0; i < 4; ++i) {
    q.segment(3 * i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
  }
  return q;
}

// 逆运动学导数：计算所有腿的关节角速度
Vec12 HexapodRobot::getQd(const Vec34 &pos, const Vec34 &vel, FrameType frame) {
  Vec12 qd;
  for (int i = 0; i < 4; ++i) {
    qd.segment(3 * i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
  }
  return qd;
}

// 逆动力学：计算所有腿的关节力矩
Vec12 HexapodRobot::getTau(const Vec12 &q, const Vec34 feetForce) {
  Vec12 tau;
  for (int i = 0; i < 4; ++i) {
    tau.segment(3 * i, 3) =
        _Legs[i]->calcTau(q.segment(3 * i, 3), feetForce.col(i));
  }
  return tau;
}

// 正运动学：获取指定腿的足端位置
Vec3 HexapodRobot::getFootPosition(LowlevelStateHexapod &state, int id,
                                   FrameType frame) {

  q = state.getQ2();
  pinocchio::forwardKinematics(model, data, q);

  if (frame == FrameType::BODY) {
    return _Legs[id]->calcPEe2B(qLegs.col(id));
  } else if (frame == FrameType::HIP) {
    return _Legs[id]->calcPEe2H(qLegs.col(id));
  } else {
    std::cout << "[ERROR] The frame of function: getFootPosition can only be "
                 "BODY or HIP."
              << std::endl;
    exit(-1);
  }
}

// 正运动学导数：获取指定腿的足端速度
Vec3 HexapodRobot::getFootVelocity(LowlevelStateHexapod &state, int id) {
  Vec34 qLegs = state.getQ();
  Vec34 qdLegs = state.getQd();
  return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// 正运动学：获取所有足端在指定坐标系下的位置
Vec34 HexapodRobot::getFeet2BPositions(LowlevelStateHexapod &state,
                                       FrameType frame) {
  Vec34 feetPos;
  if (frame == FrameType::GLOBAL) {
    for (int i = 0; i < 4; ++i) {
      feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
    }
    feetPos = state.getRotMat() * feetPos;
  } else if ((frame == FrameType::BODY) || (frame == FrameType::HIP)) {
    for (int i = 0; i < 4; ++i) {
      feetPos.col(i) = getFootPosition(state, i, frame);
    }
  } else {
    std::cout << "[ERROR] Frame error of function getFeet2BPositions"
              << std::endl;
    exit(-1);
  }
  return feetPos;
}

// 正运动学导数：获取所有足端在指定坐标系下的速度
Vec34 HexapodRobot::getFeet2BVelocities(LowlevelStateHexapod &state,
                                        FrameType frame) {
  Vec34 feetVel;
  for (int i = 0; i < 4; ++i) {
    feetVel.col(i) = getFootVelocity(state, i);
  }

  if (frame == FrameType::GLOBAL) {
    Vec34 feetPos = getFeet2BPositions(state, FrameType::BODY);
    feetVel += skew(state.getGyro()) * feetPos;
    return state.getRotMat() * feetVel;
  } else if ((frame == FrameType::BODY) || (frame == FrameType::HIP)) {
    return feetVel;
  } else {
    std::cout << "[ERROR] Frame error of function getFeet2BVelocities"
              << std::endl;
    exit(-1);
  }
}

// 获取指定腿的雅可比矩阵
Mat3 HexapodRobot::getJaco(LowlevelStateHexapod &state, int legID) {
  return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

// ELminiRobot的构造函数
ELminiRobot::ELminiRobot() {
  // 初始化四条腿，并设置其相对于机身中心的安装位置
  _Legs[0] = new ElminiLeg(0, Vec3(0.1805, -0.047, 0));
  _Legs[1] = new ElminiLeg(1, Vec3(0.1805, 0.047, 0));
  _Legs[2] = new ElminiLeg(2, Vec3(0.1805, -0.047, 0));
  _Legs[3] = new ElminiLeg(3, Vec3(-0.1805, 0.047, 0));
  _Legs[4] = new ElminiLeg(3, Vec3(-0.1805, 0.047, 0));
  _Legs[5] = new ElminiLeg(3, Vec3(-0.1805, 0.047, 0));

  pinocchio::urdf::buildModel(urdf_filename, model, false);
  data = pinocchio::Data(model);

  // 设置默认站立姿态下各关节的位置
  _feetPosNormalStand << 0.1805, 0.1805, -0.1805, -0.1805, -0.1308, 0.1308,
      -0.1308, 0.1308, -0.3180, -0.3180, -0.3180, -0.3180, -0.1308, 0.1308,
      -0.3180, -0.3180, -0.3180, -0.3180;

  // 设置机器人的速度限制
  _robVelLimitX << -0.4, 0.4;
  _robVelLimitY << -0.3, 0.3;
  _robVelLimitYaw << -0.5, 0.5;

#ifdef COMPILE_WITH_REAL_ROBOT
  // 如果是为真实机器人编译，设置其物理参数
  _mass = 12.5;
  _pcb << 0.01, 0.0, 0.0;
  _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_GAZEBO
  // 如果是为Gazebo仿真编译，设置其物理参数
  _mass = 13.4;
  _pcb << 0.0, 0.0, 0.0;
  _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif // COMPILE_WITH_GAZEBO
}