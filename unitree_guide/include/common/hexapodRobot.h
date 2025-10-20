/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/hexapodLeg.h"
#include "message/LowlevelState.h"
// 引入Pinocchio库中用于解析URDF文件的头文件。
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/kinematics.hpp>

// 六足机器人类，提供运动学和动力学计算功能
class HexapodRobot {
public:
  HexapodRobot(){};  // 构造函数
  ~HexapodRobot() {} // 析构函数

  // 获取机器人位置
  Vec3 getX(LowlevelStateHexapod &state);
  Vec34 getVecXP(LowlevelStateHexapod &state);

  // 逆运动学（Body/Hip坐标系）
  Vec12 getQ(const Vec34 &feetPosition, FrameType frame); // 计算关节角度
  Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity,
              FrameType frame);                        // 计算关节角速度
  Vec12 getTau(const Vec12 &q, const Vec34 feetForce); // 计算关节力矩

  // 正运动学
  Vec3 getFootPosition(LowlevelStateHexapod &state, int id,
                       FrameType frame);              // 获取单足位置
  Vec3 getFootVelocity(LowlevelStateHexapod &state, int id); // 获取单足速度
  Vec34 getFeet2BPositions(LowlevelStateHexapod &state,
                           FrameType frame); // 获取所有足相对身体的位置
  Vec34 getFeet2BVelocities(LowlevelStateHexapod &state,
                            FrameType frame); // 获取所有足相对身体的速度

  // 获取雅可比矩阵
  Mat3 getJaco(LowlevelStateHexapod &state, int legID);

  // 获取机器人运动限制
  Vec2 getRobVelLimitX() { return _robVelLimitX; }
  Vec2 getRobVelLimitY() { return _robVelLimitY; }
  Vec2 getRobVelLimitYaw() { return _robVelLimitYaw; }

  // 获取机器人默认站立关节位置
  Vec36 getFeetPosIdeal() { return _feetPosNormalStand; }

  // 获取机器人质量
  double getRobMass() { return _mass; }

  // 获取机器人质心位置
  Vec3 getPcb() { return _pcb; }

  // 获取机器人惯性矩阵
  Mat3 getRobInertial() { return _Ib; }

protected:
  HexapodLeg *_Legs[6];      // 六足机器人腿部对象数组
  Vec2 _robVelLimitX;        // X方向速度限制
  Vec2 _robVelLimitY;        // Y方向速度限制
  Vec2 _robVelLimitYaw;      // 偏航角速度限制
  Vec36 _feetPosNormalStand; // 默认站立位置
  double _mass;              // 机器人质量
  Vec3 _pcb;                 // 质心位置
  Mat3 _Ib;                  // 惯性矩阵
  pinocchio::Model model;
  pinocchio::Data data;
  Vec18 q;                   // 所有的关节此时的位置
};

// ELmini机器人类，继承自HexapodRobot
class ELminiRobot : public HexapodRobot {
public:
  ELminiRobot();    // 构造函数
  ~ELminiRobot() {} // 析构函数
  std::string urdf_filename =
      "/home/tipriest/Documents/legged_localization_benchmark/src/unitree_ros/"
      "robots/el_mini_description/urdf/sw2urdf.urdf";
};

#endif // UNITREEROBOT_H