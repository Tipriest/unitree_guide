/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

class KeyBoard : public CmdPanel {
public:
  KeyBoard();
  ~KeyBoard();

private:
  static void *runKeyBoard(void *arg);
  void *run(void *arg);
  UserCommand checkCmd();
  void changeValue();

  pthread_t _tid;
  float sensitivityLeft = 0.05;
  float sensitivityRight = 0.05;
  struct termios _oldSettings, _newSettings;
  fd_set set;
  int res;
  int ret;
  char _c;
};

#endif // KEYBOARD_H