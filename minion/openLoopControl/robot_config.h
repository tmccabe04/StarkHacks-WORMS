#pragma once

#include <stdint.h>

struct MotorPinMap {
  uint8_t left_in1 {10};
  uint8_t left_in2 {11};
  uint8_t left_pwm {9};

  uint8_t right_in1 {12};
  uint8_t right_in2 {13};
  uint8_t right_pwm {14};

  // OUT2 is left positive and OUT1 is left negative in this wiring.
  bool left_inverted {true};
  bool right_inverted {false};
};

struct MotionLimits {
  float move_max_speed_m_s {0.22f};
  float move_max_accel_m_s2 {0.40f};

  float turn_max_speed_rad_s {1.60f};
  float turn_max_accel_rad_s2 {3.00f};
};

struct SafetyConfig {
  uint32_t command_timeout_ms {15000};
  float max_control_dt_s {0.050f};
  float stop_deadband_cmd {0.02f};
};

struct MotorModelConfig {
  uint16_t pwm_frequency_hz {18000};
  uint8_t pwm_resolution_bits {10};

  float max_duty {0.70f};
  float min_start_duty {0.20f};
  float command_slew_per_s {2.4f};

  // Feedforward conversion: wheel speed ~= duty * duty_to_speed_m_s.
  float duty_to_speed_m_s {0.90f};
};

struct RobotConfig {
  float wheel_diameter_m {0.057f};
  float wheel_base_m {0.090f};

  uint32_t control_period_ms {10};

  MotorPinMap pins {};
  MotionLimits limits {};
  SafetyConfig safety {};
  MotorModelConfig motor_model {};
};

inline RobotConfig defaultRobotConfig() {
  RobotConfig cfg;
  return cfg;
}
