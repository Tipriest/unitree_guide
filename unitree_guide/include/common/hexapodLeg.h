/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/enumClass.h"
#include "common/mathTypes.h"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

class HexapodLeg {
public:
  // 构造函数
  HexapodLeg(int legID, float abadLinkLength, float hipLinkLength,
             float kneeLinkLength, Vec3 pHip2B);
  ~HexapodLeg() {}
  // 正向运动学：计算足端相对于髋关节（基座）坐标系的位置
  Vec3 calcPEe2H(Vec3 q);
  // 正向运动学：计算足端相对于机身坐标系的位置
  Vec3 calcPEe2B(Vec3 q);
  // 正向运动学导数：计算足端速度
  Vec3 calcVEe(Vec3 q, Vec3 qd);
  // 逆向运动学：根据足端位置计算关节角度
  Vec3 calcQ(Vec3 pEe, FrameType frame);
  // 逆向运动学导数：根据关节角度和足端速度计算关节角速度
  Vec3 calcQd(Vec3 q, Vec3 vEe);
  // 逆向运动学导数：根据足端位置和速度计算关节角速度
  Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
  // 逆向动力学：根据关节角度和足端力计算关节力矩
  Vec3 calcTau(Vec3 q, Vec3 force);
  // 计算雅可比矩阵
  Mat3 calcJaco(Vec3 q);
  // 获取髋关节（基座）到机身的平移向量
  Vec3 getHip2B() { return _pHip2B; }

protected:
  // 逆运动学辅助函数：计算q1
  float q1_ik(float py, float pz, float b2y);
  // 逆运动学辅助函数：计算q3
  float q3_ik(float b3z, float b4z, float b);
  // 逆运动学辅助函数：计算q2
  float q2_ik(float q1, float q3, float px, float py, float pz, float b3z,
              float b4z);
  float _sideSign; // 侧边符号（-1或1），用于区分左右腿
  const float _abadLinkLength; // 髋关节（abduction/adduction）连杆长度
  const float _hipLinkLength;  // 大腿连杆长度
  const float _kneeLinkLength; // 小腿连杆长度
  const Vec3 _pHip2B; // 髋关节（基座）坐标系到机身坐标系的平移向量
  pinocchio::Model model_; // Pinocchio model for the leg
  pinocchio::Data data_;   // Pinocchio data for the leg
};

// Elmini机器人的腿部类，继承自HexapodLeg
class ElminiLeg : public HexapodLeg {
public:
  // 构造函数，使用Elmini机器人的特定连杆参数
  ElminiLeg(const int legID, const Vec3 pHip2B)
      : HexapodLeg(legID, 0.0838, 0.2, 0.2, pHip2B) {}
  ~ElminiLeg() {}
};

#endif // UNITREELEG_H