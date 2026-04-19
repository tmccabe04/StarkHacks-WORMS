#pragma once

#include <Arduino.h>

#include "imu_service.h"
#include "motion_planner.h"
#include "motion_types.h"
#include "motor_driver_l298n.h"
#include "pid_controller.h"
#include "robot_config.h"

class MotionController {
public:
  MotionController() = default;

  bool begin(const RobotConfig& config);
  bool submitCommand(const MotionCommand& command);

  void controlTick(uint32_t now_ms);

  Telemetry getTelemetry() const;
  ControlState state() const;
  FaultCode fault() const;

  void clearFault();

private:
  static constexpr size_t kQueueSize = 8;

  struct ActiveSetpoint {
    float target_linear_m_s {0.0f};
    float target_omega_rad_s {0.0f};
    bool command_done {false};
  };

  bool startNextCommand(uint32_t now_ms);
  void clearQueue();
  bool enqueue(const MotionCommand& cmd);
  bool dequeue(MotionCommand* cmd);

  void updateEstimator(float dt_s, const ImuSample& imu_sample);
  ActiveSetpoint computeActiveSetpoint(float dt_s, uint32_t now_ms, const ImuSample& imu_sample);
  void computeWheelCommandTargets(float dt_s, const ActiveSetpoint& setpoint, const ImuSample& imu_sample);
  void applyCommandSlew(float dt_s);

  void enterStopping();
  void enterFault(FaultCode fault);
  bool outputsNearZero() const;

  static float clampf(float value, float lo, float hi);
  static float wrapAngleRad(float angle);

  RobotConfig cfg_ {};

  MotorDriverL298N motor_;
  ImuService imu_;

  PidController move_heading_pid_;
  PidController turn_heading_pid_;
  PidController left_speed_pi_;
  PidController right_speed_pi_;

  MotionCommand queue_[kQueueSize] {};
  size_t queue_head_ {0};
  size_t queue_tail_ {0};
  size_t queue_count_ {0};

  bool has_active_command_ {false};
  MotionCommand active_command_ {};
  TrapezoidProfile active_profile_ {};
  uint32_t active_start_ms_ {0};
  uint8_t turn_settle_ticks_ {0};

  float command_start_yaw_rad_ {0.0f};
  float command_target_heading_rad_ {0.0f};
  float command_start_distance_m_ {0.0f};

  float model_left_speed_m_s_ {0.0f};
  float model_right_speed_m_s_ {0.0f};
  float estimated_forward_speed_m_s_ {0.0f};
  float estimated_distance_m_ {0.0f};

  float target_left_cmd_ {0.0f};
  float target_right_cmd_ {0.0f};
  float applied_left_cmd_ {0.0f};
  float applied_right_cmd_ {0.0f};

  uint32_t last_tick_ms_ {0};
  uint32_t last_command_rx_ms_ {0};

  ControlState state_ {ControlState::Idle};
  FaultCode fault_ {FaultCode::None};

  Telemetry telemetry_ {};
};
