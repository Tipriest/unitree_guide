/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/JoyStick.h"
#include <iostream>

JoyStick::JoyStick(ros::NodeHandle &_nh) {
  nh = _nh;
  joystick_cmd_sub =
      nh.subscribe<sensor_msgs::Joy>("/robot_control/joystick_msgs", 1,
                                     &JoyStick::subJoystickMsgsCallback, this);

  userCmd = UserCommand::NONE;
  userValue.setZero();

  pthread_create(&_tid, nullptr, runJoyStick, (void *)this);
}

JoyStick::~JoyStick() {
  pthread_cancel(_tid);
  pthread_join(_tid, nullptr);
}

void JoyStick::subJoystickMsgsCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  // Handle joystick message here
  // Example: you can parse msg->axes and msg->buttons
  if (1 == msg->buttons[0]) {
    userCmd = UserCommand::L2_A;
  } else if (1 == msg->buttons[1]) {
    userCmd = UserCommand::START;
  }
  userValue.lx = clamp(msg->axes[0] * 1.0 / 32767, -1.0, 1.0);
  userValue.ly = clamp(msg->axes[1] * 1.0 / 32767, -1.0, 1.0);
  userValue.rx = clamp(msg->axes[2] * 1.0 / 32767, -1.0, 1.0);
  userValue.ry = clamp(msg->axes[3] * 1.0 / 32767, -1.0, 1.0);
  if (userValue.lx <= 0.005 && userValue.lx >= -0.005) {
    userValue.lx = 0.0;
  }
  if (userValue.ly <= 0.005 && userValue.ly >= -0.005) {
    userValue.ly = 0.0;
  }
  if (userValue.rx <= 0.005 && userValue.rx >= -0.005) {
    userValue.rx = 0.0;
  }
  if (userValue.ry <= 0.005 && userValue.ry >= -0.005) {
    userValue.ry = 0.0;
  }
  return;
}

void *JoyStick::runJoyStick(void *arg) {
  ((JoyStick *)arg)->run(nullptr);
  return nullptr;
}

void *JoyStick::run(void *arg) {
  while (1) {
    usleep(1000);
  }
  return nullptr;
}