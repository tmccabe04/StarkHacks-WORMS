#include "motor_driver_l298n.h"

#include <math.h>

namespace {

float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

}  // namespace

bool MotorDriverL298N::begin(const RobotConfig& config) {
  left_ = {
    config.pins.left_in1,
    config.pins.left_in2,
    config.pins.left_pwm,
    config.pins.left_pwm_channel,
    config.pins.left_inverted,
  };

  right_ = {
    config.pins.right_in1,
    config.pins.right_in2,
    config.pins.right_pwm,
    config.pins.right_pwm_channel,
    config.pins.right_inverted,
  };

  max_duty_ = clampf(config.motor_model.max_duty, 0.0f, 1.0f);
  duty_counts_max_ = (1 << config.motor_model.pwm_resolution_bits) - 1;

  pinMode(left_.in1, OUTPUT);
  pinMode(left_.in2, OUTPUT);
  pinMode(right_.in1, OUTPUT);
  pinMode(right_.in2, OUTPUT);

  if (!ledcAttachChannel(left_.pwm_pin, config.motor_model.pwm_frequency_hz, config.motor_model.pwm_resolution_bits, left_.pwm_channel)) {
    return false;
  }
  if (!ledcAttachChannel(right_.pwm_pin, config.motor_model.pwm_frequency_hz, config.motor_model.pwm_resolution_bits, right_.pwm_channel)) {
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
  digitalWrite(left_.in1, LOW);
  digitalWrite(left_.in2, LOW);
  digitalWrite(right_.in1, LOW);
  digitalWrite(right_.in2, LOW);
  ledcWriteChannel(left_.pwm_channel, 0);
  ledcWriteChannel(right_.pwm_channel, 0);
}

void MotorDriverL298N::writeNormalized(float left_cmd, float right_cmd) {
  if (!initialized_) {
    return;
  }
  applyChannel(left_, left_cmd);
  applyChannel(right_, right_cmd);
}

void MotorDriverL298N::applyChannel(const ChannelConfig& channel, float cmd) {
  cmd = clampf(cmd, -1.0f, 1.0f);

  bool forward = (cmd >= 0.0f);
  if (channel.inverted) {
    forward = !forward;
  }

  const int duty = dutyFromCommand(fabsf(cmd));

  if (duty == 0) {
    digitalWrite(channel.in1, LOW);
    digitalWrite(channel.in2, LOW);
    ledcWriteChannel(channel.pwm_channel, 0);
    return;
  }

  if (forward) {
    digitalWrite(channel.in1, HIGH);
    digitalWrite(channel.in2, LOW);
  } else {
    digitalWrite(channel.in1, LOW);
    digitalWrite(channel.in2, HIGH);
  }

  ledcWriteChannel(channel.pwm_channel, duty);
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
