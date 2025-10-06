/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <csignal>
#include <iostream>
#include <sched.h>
#include <unistd.h>

#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#endif // COMPILE_WITH_ROS

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig) {
  std::cerr << "stop the controller" << std::endl;
  running = false;
}

void setProcessScheduler() {
  // 获取当前进程的 PID
  pid_t pid = getpid();
  // sched_param 是一个结构体，用于存储调度参数。在这里，param.sched_priority
  // 被设置为 SCHED_FIFO 调度策略的最高优先级。
  // sched_get_priority_max(SCHED_FIFO) 函数返回 SCHED_FIFO
  // 策略允许的最高优先级值。
  sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  // 用于设置指定进程的调度策略和优先级
  if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
    std::cerr << "[ERROR] Function setProcessScheduler failed." << std::endl;
  }
}

int main(int argc, char **argv) {
  /* set real-time process */
  setProcessScheduler();
  /* set the print format */
  std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS
  ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

  IOInterface *ioInter;
  CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_GAZEBO
  ioInter = new IOROS();
  ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_GAZEBO

#ifdef COMPILE_WITH_REAL_ROBOT
  ioInter = new IOSDK();
  ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

  CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
  ctrlComp->ctrlPlatform = ctrlPlat;
  ctrlComp->dt = 0.002; // run at 500hz
  ctrlComp->running = &running;

#ifdef ROBOT_TYPE_A1
  ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1
  ctrlComp->robotModel = new Go1Robot();
#endif

  ctrlComp->waveGen =
      new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
  // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));
  // //Crawl, only for sim ctrlComp->waveGen = new WaveGenerator(0.4, 0.6,
  // Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim ctrlComp->waveGen =
  // new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only
  // for sim ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));
  // //Pronk, only for sim

  ctrlComp->geneObj();

  ControlFrame ctrlFrame(ctrlComp);

  signal(SIGINT, ShutDown);

  while (running) {
    ctrlFrame.run();
  }

  delete ctrlComp;
  return 0;
}
