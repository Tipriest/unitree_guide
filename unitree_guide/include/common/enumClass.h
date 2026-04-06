/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform {
  GAZEBO,
  REALROBOT,
};

enum class RobotType { A1, Go1 };

enum class UserCommand {
  // EXIT,
  NONE,
  START, // freestand
  L2_A,  // passive
  L2_B,  // move_base
  L2_X,  // fixedstand
  L2_Y,  // trotting
  L1_X,  // balanceTest
  L1_A,  // swingTest
  L1_Y,  // stepTest
};

enum class FrameType { BODY, HIP, GLOBAL };

enum class WaveStatus { STANCE_ALL, SWING_ALL, WAVE_ALL };

enum class FSMMode { NORMAL, CHANGE };

enum class FSMStateName {
  // EXIT,
  INVALID,
  PASSIVE,
  FIXEDSTAND,
  FREESTAND,
  TROTTING,
  MOVE_BASE, // move_base
  BALANCETEST,
  SWINGTEST,
  STEPTEST
};

#endif // ENUMCLASS_H