/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/hexapodLeg.h"
#include <iostream>

/************************/
/*******HexapodLeg*******/
/************************/
// HexapodLeg类的构造函数
HexapodLeg::HexapodLeg(int legID, float abadLinkLength, float hipLinkLength,
                       float kneeLinkLength, Vec3 pHip2B)
    : _abadLinkLength(abadLinkLength), _hipLinkLength(hipLinkLength),
      _kneeLinkLength(kneeLinkLength), _pHip2B(pHip2B) {
  // 根据腿的ID设置侧边符号，用于区分左右腿的运动学计算
  if (legID == 0 || legID == 2 || legID == 4)
    _sideSign = -1; // 左侧腿
  else if (legID == 1 || legID == 3 || legID == 5)
    _sideSign = 1; // 右侧腿
  else {
    std::cout << "Leg ID incorrect!" << std::endl;
    exit(-1);
  }

  // Initialize Pinocchio model and data for the leg
  pinocchio::urdf::buildModel("path_to_leg_urdf.urdf", model_);
  data_ = pinocchio::Data(model_);
}

// 正向运动学
/**
 * @brief 根据三个关节的角度q计算足端到髋关节（基座）坐标系原点的向量坐标
 * @param[in] q   三个关节的角度q
 * @return Vec3   足端在髋关节坐标系下的位置向量
 */
Vec3 HexapodLeg::calcPEe2H(Vec3 q) {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[model_.getFrameId("FOOT_LINK")].translation();
}

// 正向运动学
/**
 * @brief 根据三个关节的角度q计算足端到机身坐标系原点的向量坐标
 * @param[in] q   三个关节的角度q
 * @return Vec3   足端在机身坐标系下的位置向量
 */
Vec3 HexapodLeg::calcPEe2B(Vec3 q) { return _pHip2B + calcPEe2H(q); }

// 正向运动学导数
/**
 * @brief 根据三个关节的角度q和角速度qd，计算足端在机身坐标系下的速度向量
 * @param[in] q   三个关节的角度q
 * @param[in] qd  三个关节的角速度qd
 * @return Vec3   足端的速度向量
 */
Vec3 HexapodLeg::calcVEe(Vec3 q, Vec3 qd) {
  Eigen::MatrixXd jacobian(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, data_, q,
                                  model_.getFrameId("FOOT_LINK"),
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  return jacobian.topRows<3>() * qd;
}

// 逆向运动学
/**
 * @brief
 * 逆运动学，根据足端在机身坐标系或者在髋关节（基座）坐标系下的位置，求解三个关节的期望角度
 * @param[in] pEe   足端位置
 * @param[in] frame 坐标系类型 (HIP或BODY)
 * @return Vec3     计算出的三个关节的角度
 */
Vec3 HexapodLeg::calcQ(Vec3 pEe, FrameType frame) {
  // Use Pinocchio's inverse kinematics solver
  pinocchio::SE3 target_pose(pinocchio::SE3::Identity());
  target_pose.translation() = pEe;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
  pinocchio::inverseKinematics(model_, data_, model_.getFrameId("FOOT_LINK"),
                               target_pose, q);
  return q.head<3>();
}

// 逆向运动学导数
// 根据关节角度和足端速度计算关节角速度
Vec3 HexapodLeg::calcQd(Vec3 q, Vec3 vEe) {
  Eigen::MatrixXd jacobian(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, data_, q,
                                  model_.getFrameId("FOOT_LINK"),
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  return jacobian.topRows<3>().inverse() * vEe;
}

// 逆向运动学导数
// 根据足端位置和速度计算关节角速度
Vec3 HexapodLeg::calcQd(Vec3 pEe, Vec3 vEe, FrameType frame) {
  Vec3 q = calcQ(pEe, frame);
  return calcQd(q, vEe);
}

// 逆向动力学
// 根据关节角度和足端受力计算关节力矩
Vec3 HexapodLeg::calcTau(Vec3 q, Vec3 force) {
  Eigen::MatrixXd jacobian(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, data_, q,
                                  model_.getFrameId("FOOT_LINK"),
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  return jacobian.topRows<3>().transpose() * force;
}

// 计算雅可比矩阵
Mat3 HexapodLeg::calcJaco(Vec3 q) {
  Eigen::MatrixXd jacobian(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, data_, q,
                                  model_.getFrameId("FOOT_LINK"),
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  return jacobian.topRows<3>();
}

// 逆运动学辅助函数：计算q1
float HexapodLeg::q1_ik(float py, float pz, float l1) {
  float q1;
  float L = sqrt(pow(py, 2) + pow(pz, 2) - pow(l1, 2));
  q1 = atan2(pz * l1 + py * L, py * l1 - pz * L);
  return q1;
}

// 逆运动学辅助函数：计算q3
float HexapodLeg::q3_ik(float b3z, float b4z, float b) {
  float q3, temp;
  temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2)) / (2 * fabs(b3z * b4z));
  if (temp > 1)
    temp = 1;
  if (temp < -1)
    temp = -1;
  q3 = acos(temp);
  q3 = -(M_PI - q3); // 0~180
  return q3;
}

// 逆运动学辅助函数：计算q2
float HexapodLeg::q2_ik(float q1, float q3, float px, float py, float pz,
                        float b3z, float b4z) {
  float q2, a1, a2, m1, m2;

  a1 = py * sin(q1) - pz * cos(q1);
  a2 = px;
  m1 = b4z * sin(q3);
  m2 = b3z + b4z * cos(q3);
  q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);
  return q2;
}
