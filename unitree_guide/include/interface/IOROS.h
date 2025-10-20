/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef IOROS_H
#define IOROS_H

#include "interface/IOInterface.h"
#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>

class IOROS : public IOInterface {
public:
  IOROS();
  ~IOROS();
  void sendRecv(const LowlevelCmdHexapod *cmd, LowlevelStateHexapod *state);

private:
  void sendCmd(const LowlevelCmdHexapod *cmd);
  void recvState(LowlevelStateHexapod *state);
  ros::NodeHandle _nm;
  ros::Subscriber _servo_sub[18], _imu_sub;
  ros::Publisher _servo_pub[18];
  unitree_legged_msgs::LowCmd _lowCmd;
  unitree_legged_msgs::LowState _lowState;
  std::string _robot_name;

  // repeated functions for multi-thread
  void initRecv();
  void initSend();

  // Callback functions for ROS
  void imuCallback(const sensor_msgs::Imu &msg);

  void RF_HAACallback(const unitree_legged_msgs::MotorState &msg);
  void RF_HFECallback(const unitree_legged_msgs::MotorState &msg);
  void RF_KFECallback(const unitree_legged_msgs::MotorState &msg);

  void RM_HAACallback(const unitree_legged_msgs::MotorState &msg);
  void RM_HFECallback(const unitree_legged_msgs::MotorState &msg);
  void RM_KFECallback(const unitree_legged_msgs::MotorState &msg);

  void RB_HAACallback(const unitree_legged_msgs::MotorState &msg);
  void RB_HFECallback(const unitree_legged_msgs::MotorState &msg);
  void RB_KFECallback(const unitree_legged_msgs::MotorState &msg);

  void LF_HAACallback(const unitree_legged_msgs::MotorState &msg);
  void LF_HFECallback(const unitree_legged_msgs::MotorState &msg);
  void LF_KFECallback(const unitree_legged_msgs::MotorState &msg);

  void LM_HAACallback(const unitree_legged_msgs::MotorState &msg);
  void LM_HFECallback(const unitree_legged_msgs::MotorState &msg);
  void LM_KFECallback(const unitree_legged_msgs::MotorState &msg);

  void LB_HAACallback(const unitree_legged_msgs::MotorState &msg);
  void LB_HFECallback(const unitree_legged_msgs::MotorState &msg);
  void LB_KFECallback(const unitree_legged_msgs::MotorState &msg);
};

#endif // IOROS_H
