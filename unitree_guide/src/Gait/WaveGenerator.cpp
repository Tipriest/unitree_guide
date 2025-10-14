/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <math.h>
#include <sys/time.h>
/**
 * @brief Construct a new Wave Generator:: Wave Generator object
 * @param[in] period        步态周期P
 * @param[in] stancePhaseRatioMy 触地系数r
 * @param[in] bias          四条腿偏移时间b与步态周期P的比值
 */
WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias) {

  if ((_stRatio >= 1) || (_stRatio <= 0)) {
    std::cout
        << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)"
        << std::endl;
    exit(-1);
  }

  for (int i = 0; i < bias.rows(); ++i) {
    if ((bias(i) > 1) || (bias(i) < 0)) {
      std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]"
                << std::endl;
      exit(-1);
    }
  }

  _startT = getSystemTime();
  _contactPast.setZero();
  _phasePast << 0.5, 0.5, 0.5, 0.5;
  _statusPast = WaveStatus::SWING_ALL;
}

WaveGenerator::~WaveGenerator() {}

/**
 * @brief 计算当前时刻四个足端的相位phaseResult和接触状态contactResult
 * @param[out] phaseResult
 * @param[out] contactResult 1代表触地面，0代表腾空
 * @param[in] status 机器人的步态状态:
 *                      STANCE_ALL: 四条腿始终触地
 *                      SWING_ALL:  四条腿全部腾空
 *                      WAVE_ALL:   四条腿交替触地, 腾空
 *
 */
void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult,
                                     WaveStatus status) {

  calcWave(_phase, _contact, status);

  if (status != _statusPast) {
    if (_switchStatus.sum() == 0) {
      _switchStatus.setOnes();
    }
    calcWave(_phasePast, _contactPast, _statusPast);
    // two special case
    if ((status == WaveStatus::STANCE_ALL) &&
        (_statusPast == WaveStatus::SWING_ALL)) {
      _contactPast.setOnes();
    } else if ((status == WaveStatus::SWING_ALL) &&
               (_statusPast == WaveStatus::STANCE_ALL)) {
      _contactPast.setZero();
    }
  }

  if (_switchStatus.sum() != 0) {
    for (int i = 0; i < 4; ++i) {
      if (_contact(i) == _contactPast(i)) {
        _switchStatus(i) = 0;
      } else {
        _contact(i) = _contactPast(i);
        _phase(i) = _phasePast(i);
      }
    }
    if (_switchStatus.sum() == 0) {
      _statusPast = status;
    }
  }

  phaseResult = _phase;
  contactResult = _contact;
}

/**
 * @brief 返回足端的触地时长T_stance
 * @return float
 */
float WaveGenerator::getTstance() { return _period * _stRatio; }

/**
 * @brief 返回足端的腾空时长T_swing
 * @return float
 */
float WaveGenerator::getTswing() { return _period * (1 - _stRatio); }

/**
 * @brief 返回步态周期P
 * @return float
 */
float WaveGenerator::getT() { return _period; }

/**
 * @brief 计算按照从 WaveGenerator类
 * 初始化到现在的时间，按照一开始初始化好的步态系数，应该
 * 是接触态还是摆动态，是否理论上应该是触地的，理论相位是多少
 * @param[in] phase         理论当前相位
 * @param[in] contact       理论当前接触状态
 * @param[in] status        机器人处于完全腾空，完全支撑还是一半一半
 */
void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status) {
  if (status == WaveStatus::WAVE_ALL) {
    _passT = (double)(getSystemTime() - _startT) * 1e-6;
    for (int i = 0; i < 4; ++i) {
      _normalT(i) =
          fmod(_passT + _period - _period * _bias(i), _period) / _period;
      if (_normalT(i) < _stRatio) {
        contact(i) = 1;
        phase(i) = _normalT(i) / _stRatio;
      } else {
        contact(i) = 0;
        phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
      }
    }
  } else if (status == WaveStatus::SWING_ALL) {
    contact.setZero();
    phase << 0.5, 0.5, 0.5, 0.5;
  } else if (status == WaveStatus::STANCE_ALL) {
    contact.setOnes();
    phase << 0.5, 0.5, 0.5, 0.5;
  }
}