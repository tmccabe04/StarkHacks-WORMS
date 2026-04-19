#pragma once

#include <Arduino.h>
#include <L298N.h>

#include "robot_config.h"

class MotorDriverL298N {
public:
  MotorDriverL298N() = default;

  bool begin(const RobotConfig& config);
  void writeNormalized(float left_cmd, float right_cmd);
  void stop();

private:
  void applyChannel(L298N* motor, bool inverted, float cmd);
  int dutyFromCommand(float cmd_abs) const;

  L298N* left_motor_ {nullptr};
  L298N* right_motor_ {nullptr};
  bool left_inverted_ {false};
  bool right_inverted_ {false};

  float max_duty_ {0.85f};
  int duty_counts_max_ {255};
  bool initialized_ {false};
};
