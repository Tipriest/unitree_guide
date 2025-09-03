/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#pragma once

#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

class JoyStick : public CmdPanel {
public:
  JoyStick(ros::NodeHandle &_nh);
  ~JoyStick();
  void subJoystickMsgsCallback(const sensor_msgs::Joy::ConstPtr &msg);
  template <typename T> T clamp(T value, T low, T high) {
    if (value < low)
      return low;
    if (value > high)
      return high;
    return value;
  }

private:
  static void *runJoyStick(void *arg);
  void *run(void *arg);

  ros::NodeHandle nh;
  ros::Subscriber joystick_cmd_sub;
  pthread_t _tid;
};

