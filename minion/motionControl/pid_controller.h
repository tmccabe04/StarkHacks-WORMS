#pragma once

#include "robot_config.h"

class PidController {
public:
  PidController() = default;

  void configure(const PidGains& gains) {
    gains_ = gains;
    reset();
  }

  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_ = false;
  }

  float update(float error, float dt_s) {
    if (dt_s <= 0.0f) {
      return 0.0f;
    }

    integral_ += error * dt_s;
    if (integral_ > gains_.integral_max) {
      integral_ = gains_.integral_max;
    } else if (integral_ < gains_.integral_min) {
      integral_ = gains_.integral_min;
    }

    float derivative = 0.0f;
    if (has_prev_) {
      derivative = (error - prev_error_) / dt_s;
    }

    float output = (gains_.kp * error) + (gains_.ki * integral_) + (gains_.kd * derivative);
    if (output > gains_.output_max) {
      output = gains_.output_max;
    } else if (output < gains_.output_min) {
      output = gains_.output_min;
    }

    prev_error_ = error;
    has_prev_ = true;
    return output;
  }

private:
  PidGains gains_ {};
  float integral_ {0.0f};
  float prev_error_ {0.0f};
  bool has_prev_ {false};
};
