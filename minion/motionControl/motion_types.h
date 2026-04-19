#pragma once

#include <stdint.h>

enum class MotionCommandType {
  MoveCm,
  TurnDeg,
  Stop,
};

enum class ControlState {
  Idle,
  Executing,
  Stopping,
  Fault,
};

enum class FaultCode {
  None,
  MotorInitFailed,
  ImuUnavailable,
  ImuStale,
  CommandTimeout,
  ControlLoopOverrun,
  QueueFull,
};

struct MotionCommand {
  MotionCommandType type {MotionCommandType::Stop};
  float value {0.0f};
  float max_speed {0.0f};
  float max_accel {0.0f};
  uint32_t created_ms {0};
};

struct Telemetry {
  ControlState state {ControlState::Idle};
  FaultCode fault {FaultCode::None};

  bool imu_ok {false};
  uint32_t last_imu_update_ms {0};

  float yaw_deg {0.0f};
  float yaw_unwrapped_rad {0.0f};
  float yaw_rate_rad_s {0.0f};
  float forward_accel_m_s2 {0.0f};

  float estimated_forward_speed_m_s {0.0f};
  float estimated_distance_m {0.0f};

  float target_left_speed_m_s {0.0f};
  float target_right_speed_m_s {0.0f};
  float estimated_left_speed_m_s {0.0f};
  float estimated_right_speed_m_s {0.0f};

  float left_motor_cmd {-0.0f};
  float right_motor_cmd {-0.0f};

  MotionCommand active_command {};
  float command_progress {0.0f};
};
