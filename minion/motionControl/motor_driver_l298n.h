#pragma once

#include <Arduino.h>

#include "robot_config.h"

class MotorDriverL298N {
public:
  MotorDriverL298N() = default;

  bool begin(const RobotConfig& config);
  void writeNormalized(float left_cmd, float right_cmd);
  void stop();

private:
  struct ChannelConfig {
    uint8_t in1;
    uint8_t in2;
    uint8_t pwm_pin;
    uint8_t pwm_channel;
    bool inverted;
  };

  void applyChannel(const ChannelConfig& channel, float cmd);
  int dutyFromCommand(float cmd_abs) const;

  ChannelConfig left_ {};
  ChannelConfig right_ {};

  float max_duty_ {0.85f};
  int duty_counts_max_ {1023};
  bool initialized_ {false};
};
