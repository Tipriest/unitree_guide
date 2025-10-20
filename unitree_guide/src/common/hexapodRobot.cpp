/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/hexapodRobot.h"
#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// 获取机器人位置（这里简单地返回0号腿的足端位置作为参考）
Vec3 HexapodRobot::getX(LowlevelStateHexapod &state) {
  return getFootPosition(state, 0, FrameType::BODY);
}

// 获取所有足端相对于机器人位置参考点的向量
Vec36 HexapodRobot::getVecXP(LowlevelStateHexapod &state) {
  Vec3 x = getX(state);
  Vec36 vecXP;
  for (int i = 0; i < 6; ++i) {
    vecXP.col(i) = getFootPosition(state, i, FrameType::BODY) - x;
  }
  return vecXP;
}
// 逆运动学：根据足端位置计算关节角度
Vec18 HexapodRobot::getQ(const Vec36 &vecP, FrameType frame) {
  Vec18 q;
  for (int i = 0; i < 6; ++i) {
    // 获取足端在指定坐标系下的位置
    Vec3 foot_position = vecP.col(i);
    if (frame == FrameType::BODY) {
      // 足端位置相对于机身坐标系
      q.segment(3 * i, 3) = _Legs[i]->calcQ(foot_position, FrameType::BODY);
    } else if (frame == FrameType::HIP) {
      // 足端位置相对于髋关节坐标系
      q.segment(3 * i, 3) = _Legs[i]->calcQ(foot_position, FrameType::HIP);
    } else {
      std::cerr
          << "[ERROR] The frame of function: getQ can only be BODY or HIP."
          << std::endl;
      exit(-1);
    }
  }
  return q;
}

// 逆运动学导数：根据足端位置和速度计算关节角速度
Vec18 HexapodRobot::getQd(const Vec36 &pos, const Vec36 &vel, FrameType frame) {
  Vec18 qd;
  for (int i = 0; i < 6; ++i) {
    // 获取足端在指定坐标系下的位置和速度
    Vec3 foot_position = pos.col(i);
    Vec3 foot_velocity = vel.col(i);
    if (frame == FrameType::BODY) {
      // 足端位置和速度相对于机身坐标系
      qd.segment(3 * i, 3) =
          _Legs[i]->calcQd(foot_position, foot_velocity, FrameType::BODY);
    } else if (frame == FrameType::HIP) {
      // 足端位置和速度相对于髋关节坐标系
      qd.segment(3 * i, 3) =
          _Legs[i]->calcQd(foot_position, foot_velocity, FrameType::HIP);
    } else {
      std::cerr
          << "[ERROR] The frame of function: getQd can only be BODY or HIP."
          << std::endl;
      exit(-1);
    }
  }
  return qd;
}

Vec18 HexapodRobot::getTau(const Vec18 &q, const Vec36 feetForce) {
  Vec18 tau;
  for (int i = 0; i < 6; ++i) {
    tau.segment(3 * i, 3) =
        _Legs[i]->calcTau(q.segment(3 * i, 3), feetForce.col(i));
  }
  return tau;
}
// 正运动学：获取指定腿的足端位置
Vec3 HexapodRobot::getFootPosition(LowlevelStateHexapod &state, int id,
                                   FrameType frame) {
  Eigen::VectorXd q = state.getQ();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  if (frame == FrameType::BODY) {
    return data.oMf[model.getFrameId(transIdtoStr(id) + "_FOOT_LINK")]
        .translation();
  } else if (frame == FrameType::HIP) {
    return data.oMf[model.getFrameId(transIdtoStr(id) + "_FOOT_LINK")]
               .translation() -
           data.oMf[model.getFrameId(transIdtoStr(id) + "_HIP")].translation();
  } else {
    std::cerr << "[ERROR] The frame of function: getFootPosition can only be "
                 "BODY or HIP."
              << std::endl;
    exit(-1);
  }
}

// 正运动学导数：获取指定腿的足端速度
Vec3 HexapodRobot::getFootVelocity(LowlevelStateHexapod &state, int id) {
  Eigen::VectorXd q = state.getQ();
  Eigen::VectorXd qd = state.getQd();

  Eigen::MatrixXd jacobian(6, model.nv);
  pinocchio::computeFrameJacobian(
      model, data, q, model.getFrameId(transIdtoStr(id) + "_FOOT_LINK"),
      pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

  return jacobian.topRows<3>() * qd;
}

// 正运动学：获取所有足端在指定坐标系下的位置
Vec36 HexapodRobot::getFeet2BPositions(LowlevelStateHexapod &state,
                                       FrameType frame) {
  Vec36 feetPos;
  for (int i = 0; i < 6; ++i) {
    feetPos.col(i) = getFootPosition(state, i, frame);
  }
  return feetPos;
}

// 正运动学导数：获取所有足端在指定坐标系下的速度
Vec36 HexapodRobot::getFeet2BVelocities(LowlevelStateHexapod &state,
                                        FrameType frame) {
  Vec36 feetVel;
  for (int i = 0; i < 6; ++i) {
    feetVel.col(i) = getFootVelocity(state, i);
  }
  return feetVel;
}

// 获取指定腿的雅可比矩阵
Mat3 HexapodRobot::getJaco(LowlevelStateHexapod &state, int legID) {
  Eigen::VectorXd q = state.getQ();

  Eigen::MatrixXd jacobian(6, model.nv);
  pinocchio::computeFrameJacobian(
      model, data, q, model.getFrameId(transIdtoStr(legID) + "_FOOT_LINK"),
      pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

  return jacobian.topRows<3>();
}

// ELminiRobot的构造函数
EL201Robot::EL201Robot() {
  // 初始化六条腿，并设置其相对于机身中心的安装位置
  _Legs[0] = new ElminiLeg(0, Vec3(0.1805, -0.047, 0));
  _Legs[1] = new ElminiLeg(1, Vec3(0.1805, 0.047, 0));
  _Legs[2] = new ElminiLeg(2, Vec3(-0.1805, -0.047, 0));
  _Legs[3] = new ElminiLeg(3, Vec3(-0.1805, 0.047, 0));
  _Legs[4] = new ElminiLeg(4, Vec3(0.0, -0.1, 0));
  _Legs[5] = new ElminiLeg(5, Vec3(0.0, 0.1, 0));

  // 使用Pinocchio加载URDF模型
  pinocchio::urdf::buildModel(urdf_filename, model, false);
  data = pinocchio::Data(model);

  // 设置默认站立姿态下各关节的位置
  _feetPosNormalStand << 0.1805, 0.1805, -0.1805, -0.1805, 0.0, 0.0, -0.1308,
      0.1308, -0.1308, 0.1308, -0.1, 0.1, -0.3180, -0.3180, -0.3180, -0.3180,
      -0.3180, -0.3180;

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