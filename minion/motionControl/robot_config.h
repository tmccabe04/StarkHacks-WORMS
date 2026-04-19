#pragma once

#include <stdint.h>

struct MotorPinMap {
  uint8_t left_in1 {10};
  uint8_t left_in2 {11};
  uint8_t left_pwm {9};

  uint8_t right_in1 {12};
  uint8_t right_in2 {13};
  uint8_t right_pwm {14};

  uint8_t left_pwm_channel {0};
  uint8_t right_pwm_channel {1};

  // OUT2 is left positive and OUT1 is left negative in your wiring,
  // so left command direction is inverted relative to IN1/IN2 "forward".
  bool left_inverted {true};
  bool right_inverted {false};
};

struct PidGains {
  float kp {0.0f};
  float ki {0.0f};
  float kd {0.0f};

  float output_min {-1.0f};
  float output_max {1.0f};

  float integral_min {-1.0f};
  float integral_max {1.0f};
};

struct MotionLimits {
  float move_max_speed_m_s {0.20f};
  float move_max_accel_m_s2 {0.35f};

  float turn_max_speed_rad_s {1.25f};
  float turn_max_accel_rad_s2 {2.00f};
};

struct SafetyConfig {
  uint32_t imu_stale_timeout_ms {120};
  uint32_t command_timeout_ms {15000};
  float max_control_dt_s {0.050f};
  float stop_deadband_cmd {0.04f};

  float turn_settle_error_rad {0.04f};
  float turn_settle_rate_rad_s {0.15f};
};

struct MotorModelConfig {
  uint16_t pwm_frequency_hz {18000};
  uint8_t pwm_resolution_bits {10};

  float max_duty {0.85f};
  float min_start_duty {0.18f};
  float command_slew_per_s {2.8f};

  float duty_to_speed_m_s {0.55f};
  float speed_time_constant_s {0.10f};
  float accel_blend {0.35f};

  float heading_correction_limit_rad_s {1.2f};
};

struct ImuConfig {
  uint8_t i2c_address {0x68};
  float magnetic_declination_deg {0.0f};

  bool run_accel_gyro_calibration {false};
  uint16_t bias_samples {250};
  uint16_t bias_sample_interval_ms {4};

  uint8_t filter_iterations {10};
};

struct RobotConfig {
  float wheel_diameter_m {0.057f};
  float wheel_base_m {0.090f};

  uint32_t control_period_ms {10};

  MotorPinMap pins {};
  MotionLimits limits {};
  SafetyConfig safety {};
  MotorModelConfig motor_model {};
  ImuConfig imu {};

  PidGains move_heading_pid {
    2.40f, 0.12f, 0.05f,
    -1.2f, 1.2f,
    -0.6f, 0.6f,
  };

  PidGains turn_heading_pid {
    3.20f, 0.28f, 0.07f,
    -2.5f, 2.5f,
    -1.2f, 1.2f,
  };

  PidGains left_speed_pi {
    1.15f, 0.85f, 0.0f,
    -0.60f, 0.60f,
    -0.35f, 0.35f,
  };

  PidGains right_speed_pi {
    1.15f, 0.85f, 0.0f,
    -0.60f, 0.60f,
    -0.35f, 0.35f,
  };
};

inline RobotConfig defaultRobotConfig() {
  RobotConfig cfg;
  return cfg;
}
