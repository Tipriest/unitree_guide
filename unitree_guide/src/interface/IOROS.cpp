/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "interface/IOROS.h"
#include "interface/JoyStick.h"
#include "interface/KeyBoard.h"
#include <csignal>
#include <iostream>
#include <unistd.h>

/*
               head
  (leg1)LF ------------- RF(leg4)
           |           |
           |     x     |
  (leg2)LM |  y__|     | RM(leg5)
           |           |
           |           |
  (leg0)LB ------------- RB(leg3)
*/

void RosShutDown(int sig) {
  ROS_INFO("ROS interface shutting down!");
  ros::shutdown();
}

IOROS::IOROS() : IOInterface() {
  std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
  ros::param::get("/robot_name", _robot_name);
  std::cout << "robot_name: " << _robot_name << std::endl;

  // start subscriber
  initRecv();
  ros::AsyncSpinner subSpinner(1); // one threads
  subSpinner.start();
  usleep(300000); // wait for subscribers start
  // initialize publisher
  initSend();

  signal(SIGINT, RosShutDown);

  // cmdPanel = new KeyBoard();
  cmdPanel = new JoyStick(_nm);
}

IOROS::~IOROS() {
  delete cmdPanel;
  ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmdHexapod *cmd, LowlevelStateHexapod *state) {
  sendCmd(cmd);
  recvState(state);

  state->userCmd = cmdPanel->getUserCmd();
  state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmdHexapod *lowCmd) {
  for (int i = 0; i < 18; ++i) {
    _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
    _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
    _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
    _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
    _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
    _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
  }
  for (int m(0); m < 18; ++m) {
    _servo_pub[m].publish(_lowCmd.motorCmd[m]);
  }
  ros::spinOnce();
}

void IOROS::recvState(LowlevelStateHexapod *state) {
  for (int i = 0; i < 18; ++i) {
    state->motorState[i].q = _lowState.motorState[i].q;
    state->motorState[i].dq = _lowState.motorState[i].dq;
    state->motorState[i].ddq = _lowState.motorState[i].ddq;
    state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
  }
  for (int i = 0; i < 3; ++i) {
    state->imu.quaternion[i] = _lowState.imu.quaternion[i];
    state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
  }
  state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend() {
  _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LB_HAA_controller/command", 1);
  _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LB_HFE_controller/command", 1);
  _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LB_KFE_controller/command", 1);
  _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LF_HAA_controller/command", 1);
  _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LF_HFE_controller/command", 1);
  _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LF_KFE_controller/command", 1);
  _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LM_HAA_controller/command", 1);
  _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LM_HFE_controller/command", 1);
  _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/LM_KFE_controller/command", 1);
  _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RB_HAA_controller/command", 1);
  _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RB_HFE_controller/command", 1);
  _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RB_KFE_controller/command", 1);
  _servo_pub[12] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RF_HAA_controller/command", 1);
  _servo_pub[13] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RF_HFE_controller/command", 1);
  _servo_pub[14] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RF_KFE_controller/command", 1);
  _servo_pub[15] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RM_HAA_controller/command", 1);
  _servo_pub[16] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RM_HFE_controller/command", 1);
  _servo_pub[17] = _nm.advertise<unitree_legged_msgs::MotorCmd>(
      "/" + _robot_name + "_gazebo/RM_KFE_controller/command", 1);

}

void IOROS::initRecv() {
  _imu_sub = _nm.subscribe("/trunk_imu", 1, &IOROS::imuCallback, this);
  _servo_sub[0] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LB_HAA_controller/state", 1,
                    &IOROS::LB_HAACallback, this);
  _servo_sub[1] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LB_HFE_controller/state", 1,
                    &IOROS::LB_HFECallback, this);
  _servo_sub[2] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LB_KFE_controller/state", 1,
                    &IOROS::LB_KFECallback, this);
  _servo_sub[3] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LF_HAA_controller/state", 1,
                    &IOROS::LF_HAACallback, this);
  _servo_sub[4] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LF_HFE_controller/state", 1,
                    &IOROS::LF_HFECallback, this);
  _servo_sub[5] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LF_KFE_controller/state", 1,
                    &IOROS::LF_KFECallback, this);
  _servo_sub[6] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LM_HAA_controller/state", 1,
                    &IOROS::LM_HAACallback, this);
  _servo_sub[7] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LM_HFE_controller/state", 1,
                    &IOROS::LM_HFECallback, this);
  _servo_sub[8] =
      _nm.subscribe("/" + _robot_name + "_gazebo/LM_KFE_controller/state", 1,
                    &IOROS::LM_KFECallback, this);
  _servo_sub[9] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RB_HAA_controller/state", 1,
                    &IOROS::RB_HAACallback, this);
  _servo_sub[10] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RB_HFE_controller/state", 1,
                    &IOROS::RB_HFECallback, this);
  _servo_sub[11] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RB_KFE_controller/state", 1,
                    &IOROS::RB_KFECallback, this);
  _servo_sub[12] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RF_HAA_controller/state", 1,
                    &IOROS::RF_HAACallback, this);
  _servo_sub[13] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RF_HFE_controller/state", 1,
                    &IOROS::RF_HFECallback, this);
  _servo_sub[14] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RF_KFE_controller/state", 1,
                    &IOROS::RF_KFECallback, this);
  _servo_sub[15] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RM_HAA_controller/state", 1,
                    &IOROS::RM_HAACallback, this);
  _servo_sub[16] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RM_HFE_controller/state", 1,
                    &IOROS::RM_HFECallback, this);
  _servo_sub[17] =
      _nm.subscribe("/" + _robot_name + "_gazebo/RM_KFE_controller/state", 1,
                    &IOROS::RM_KFECallback, this);




}

void IOROS::imuCallback(const sensor_msgs::Imu &msg) {
  _lowState.imu.quaternion[0] = msg.orientation.w;
  _lowState.imu.quaternion[1] = msg.orientation.x;
  _lowState.imu.quaternion[2] = msg.orientation.y;
  _lowState.imu.quaternion[3] = msg.orientation.z;

  _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
  _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
  _lowState.imu.gyroscope[2] = msg.angular_velocity.z;

  _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
  _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
  _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}


void IOROS::LF_HAACallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[0].mode = msg.mode;
  _lowState.motorState[0].q = msg.q;
  _lowState.motorState[0].dq = msg.dq;
  _lowState.motorState[0].tauEst = msg.tauEst;
}

void IOROS::LF_HFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[1].mode = msg.mode;
  _lowState.motorState[1].q = msg.q;
  _lowState.motorState[1].dq = msg.dq;
  _lowState.motorState[1].tauEst = msg.tauEst;
}

void IOROS::LF_KFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[2].mode = msg.mode;
  _lowState.motorState[2].q = msg.q;
  _lowState.motorState[2].dq = msg.dq;
  _lowState.motorState[2].tauEst = msg.tauEst;
}

void IOROS::LM_HAACallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[3].mode = msg.mode;
  _lowState.motorState[3].q = msg.q;
  _lowState.motorState[3].dq = msg.dq;
  _lowState.motorState[3].tauEst = msg.tauEst;
}

void IOROS::LM_HFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[4].mode = msg.mode;
  _lowState.motorState[4].q = msg.q;
  _lowState.motorState[4].dq = msg.dq;
  _lowState.motorState[4].tauEst = msg.tauEst;
}

void IOROS::LM_KFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[5].mode = msg.mode;
  _lowState.motorState[5].q = msg.q;
  _lowState.motorState[5].dq = msg.dq;
  _lowState.motorState[5].tauEst = msg.tauEst;
}

void IOROS::LB_HAACallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[6].mode = msg.mode;
  _lowState.motorState[6].q = msg.q;
  _lowState.motorState[6].dq = msg.dq;
  _lowState.motorState[6].tauEst = msg.tauEst;
}

void IOROS::LB_HFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[7].mode = msg.mode;
  _lowState.motorState[7].q = msg.q;
  _lowState.motorState[7].dq = msg.dq;
  _lowState.motorState[7].tauEst = msg.tauEst;
}

void IOROS::LB_KFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[8].mode = msg.mode;
  _lowState.motorState[8].q = msg.q;
  _lowState.motorState[8].dq = msg.dq;
  _lowState.motorState[8].tauEst = msg.tauEst;
}

void IOROS::RB_HAACallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[9].mode = msg.mode;
  _lowState.motorState[9].q = msg.q;
  _lowState.motorState[9].dq = msg.dq;
  _lowState.motorState[9].tauEst = msg.tauEst;
}

void IOROS::RB_HFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[10].mode = msg.mode;
  _lowState.motorState[10].q = msg.q;
  _lowState.motorState[10].dq = msg.dq;
  _lowState.motorState[10].tauEst = msg.tauEst;
}

void IOROS::RB_KFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[11].mode = msg.mode;
  _lowState.motorState[11].q = msg.q;
  _lowState.motorState[11].dq = msg.dq;
  _lowState.motorState[11].tauEst = msg.tauEst;
}

void IOROS::RF_HAACallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[12].mode = msg.mode;
  _lowState.motorState[12].q = msg.q;
  _lowState.motorState[12].dq = msg.dq;
  _lowState.motorState[12].tauEst = msg.tauEst;
}

void IOROS::RF_HFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[13].mode = msg.mode;
  _lowState.motorState[13].q = msg.q;
  _lowState.motorState[13].dq = msg.dq;
  _lowState.motorState[13].tauEst = msg.tauEst;
}

void IOROS::RF_KFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[14].mode = msg.mode;
  _lowState.motorState[14].q = msg.q;
  _lowState.motorState[14].dq = msg.dq;
  _lowState.motorState[14].tauEst = msg.tauEst;
}

void IOROS::RM_HAACallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[15].mode = msg.mode;
  _lowState.motorState[15].q = msg.q;
  _lowState.motorState[15].dq = msg.dq;
  _lowState.motorState[15].tauEst = msg.tauEst;
}

void IOROS::RM_HFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[16].mode = msg.mode;
  _lowState.motorState[16].q = msg.q;
  _lowState.motorState[16].dq = msg.dq;
  _lowState.motorState[16].tauEst = msg.tauEst;
}

void IOROS::RM_KFECallback(const unitree_legged_msgs::MotorState &msg) {
  _lowState.motorState[17].mode = msg.mode;
  _lowState.motorState[17].q = msg.q;
  _lowState.motorState[17].dq = msg.dq;
  _lowState.motorState[17].tauEst = msg.tauEst;
}







