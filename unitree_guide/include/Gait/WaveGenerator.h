/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/enumClass.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif // COMPILE_DEBUG

/*generate linear wave, [0, 1]*/
class WaveGenerator {
public:
  WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);
  ~WaveGenerator();
  void calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult,
                        WaveStatus status);
  float getTstance();
  float getTswing();
  float getT();

private:
  void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

  double _period;  //步态周期P
  double _stRatio; //触地系数r
  Vec4 _bias;      //四条腿偏移时间b与步态周期P的比值

  Vec4 _normalT; // [0, 1)
  Vec4 _phase, _phasePast;
  VecInt4 _contact, _contactPast;
  VecInt4 _switchStatus; // 1: switching, 0: do not switch
  WaveStatus _statusPast;

  double _passT;     // unit: second
  long long _startT; // unit: us
#ifdef COMPILE_DEBUG
  PyPlot _testPlot;
#endif // COMPILE_DEBUG
};

#endif // WAVEGENERATOR_H