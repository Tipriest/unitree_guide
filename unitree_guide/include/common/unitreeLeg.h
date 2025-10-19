/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/enumClass.h"
#include "common/mathTypes.h"

class QuadrupedLeg {
public:
  QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength,
               float kneeLinkLength, Vec3 pHip2B);
  ~QuadrupedLeg() {}
  Vec3 calcPEe2H(Vec3 q);
  Vec3 calcPEe2B(Vec3 q);
  Vec3 calcVEe(Vec3 q, Vec3 qd);
  Vec3 calcQ(Vec3 pEe, FrameType frame);
  Vec3 calcQd(Vec3 q, Vec3 vEe);
  Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
  Vec3 calcTau(Vec3 q, Vec3 force);
  Mat3 calcJaco(Vec3 q);
  Vec3 getHip2B() { return _pHip2B; }

protected:
  float q1_ik(float py, float pz, float b2y);
  float q3_ik(float b3z, float b4z, float b);
  float q2_ik(float q1, float q3, float px, float py, float pz, float b3z,
              float b4z);
  float _sideSign;
  const float
      _abadLinkLength; //根(机身)关节中心至大(小)腿运动平面的距离(基座坐标系到大腿坐标系的距离)
  const float _hipLinkLength;  //大腿在大(小)腿运动平面的长度
  const float _kneeLinkLength; //小腿在大(小)腿运动平面的长度
  const Vec3 _pHip2B; // 基座坐标系到机身坐标系的平移矩阵
};

class A1Leg : public QuadrupedLeg {
public:
  A1Leg(const int legID, const Vec3 pHip2B)
      : QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B) {}
  ~A1Leg() {}
};

class Go1Leg : public QuadrupedLeg {
public:
  Go1Leg(const int legID, const Vec3 pHip2B)
      : QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B) {}
  ~Go1Leg() {}
};

#endif // UNITREELEG_H