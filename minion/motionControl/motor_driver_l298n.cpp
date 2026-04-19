#include "motor_driver_l298n.h"

#include <math.h>
#include <new>

namespace {

float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

}  // namespace

bool MotorDriverL298N::begin(const RobotConfig& config) {
  if (left_motor_ != nullptr) {
    delete left_motor_;
    left_motor_ = nullptr;
  }
  if (right_motor_ != nullptr) {
    delete right_motor_;
    right_motor_ = nullptr;
  }

  left_inverted_ = config.pins.left_inverted;
  right_inverted_ = config.pins.right_inverted;

  max_duty_ = clampf(config.motor_model.max_duty, 0.0f, 1.0f);
  duty_counts_max_ = (1 << config.motor_model.pwm_resolution_bits) - 1;
  if (duty_counts_max_ <= 0) {
    duty_counts_max_ = 255;
  }

  analogWriteResolution(config.pins.left_pwm, config.motor_model.pwm_resolution_bits);
  analogWriteResolution(config.pins.right_pwm, config.motor_model.pwm_resolution_bits);
  analogWriteFrequency(config.pins.left_pwm, config.motor_model.pwm_frequency_hz);
  analogWriteFrequency(config.pins.right_pwm, config.motor_model.pwm_frequency_hz);

  left_motor_ = new (std::nothrow) L298N(config.pins.left_pwm, config.pins.left_in1, config.pins.left_in2);
  right_motor_ = new (std::nothrow) L298N(config.pins.right_pwm, config.pins.right_in1, config.pins.right_in2);

  if (left_motor_ == nullptr || right_motor_ == nullptr) {
    if (left_motor_ != nullptr) {
      delete left_motor_;
      left_motor_ = nullptr;
    }
    if (right_motor_ != nullptr) {
      delete right_motor_;
      right_motor_ = nullptr;
    }
    return false;
  }

  initialized_ = true;
  stop();
  return true;
}

void MotorDriverL298N::stop() {
  if (!initialized_) {
    return;
  }
  left_motor_->stop();
  right_motor_->stop();
}

void MotorDriverL298N::writeNormalized(float left_cmd, float right_cmd) {
  if (!initialized_) {
    return;
  }
  applyChannel(left_motor_, left_inverted_, left_cmd);
  applyChannel(right_motor_, right_inverted_, right_cmd);
}

void MotorDriverL298N::applyChannel(L298N* motor, bool inverted, float cmd) {
  if (motor == nullptr) {
    return;
  }

  cmd = clampf(cmd, -1.0f, 1.0f);

  bool forward = (cmd >= 0.0f);
  if (inverted) {
    forward = !forward;
  }

  const int duty = dutyFromCommand(fabsf(cmd));
  motor->setSpeed(static_cast<unsigned short>(duty));

  if (duty == 0) {
    motor->stop();
    return;
  }

  if (forward) {
    motor->forward();
  } else {
    motor->backward();
  }
}

int MotorDriverL298N::dutyFromCommand(float cmd_abs) const {
  cmd_abs = clampf(cmd_abs, 0.0f, 1.0f);
  const float scaled = cmd_abs * max_duty_ * static_cast<float>(duty_counts_max_);
  int duty = static_cast<int>(lroundf(scaled));
  if (duty < 0) {
    duty = 0;
  } else if (duty > duty_counts_max_) {
    duty = duty_counts_max_;
  }
  return duty;
}
