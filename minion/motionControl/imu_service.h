#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../src/MPU9250.h"
#include "robot_config.h"

struct ImuSample {
  bool valid {false};
  float yaw_deg {0.0f};
  float yaw_unwrapped_rad {0.0f};
  float yaw_rate_rad_s {0.0f};
  float forward_accel_m_s2 {0.0f};
  uint32_t updated_ms {0};
};

class ImuService {
public:
  ImuService() = default;

  bool begin(const RobotConfig& config);
  bool update(uint32_t now_ms);
  bool isHealthy(uint32_t now_ms, uint32_t stale_timeout_ms) const;

  ImuSample sample() const { return sample_; }

private:
  static float wrapDeltaDeg(float delta_deg);

  MPU9250 mpu_;
  ImuConfig cfg_ {};

  float gyro_z_bias_deg_s_ {0.0f};
  float linear_accel_x_bias_m_s2_ {0.0f};

  float yaw_unwrapped_deg_ {0.0f};
  float last_yaw_deg_ {0.0f};
  bool have_last_yaw_ {false};

  ImuSample sample_ {};
};
