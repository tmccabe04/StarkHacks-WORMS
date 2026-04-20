#pragma once

#include <Arduino.h>

#include "motion_planner.h"
#include "motion_types.h"
#include "motor_driver_l298n.h"
#include "robot_config.h"

struct RuntimeTuning {
  float duty_to_speed_m_s {0.90f};
  float min_start_duty {0.20f};
  float max_duty {0.70f};
  float command_slew_per_s {2.4f};
};

class OpenLoopController {
public:
  OpenLoopController() = default;

  bool begin(const RobotConfig& config);
  bool submitCommand(const MotionCommand& command);
  void controlTick(uint32_t now_ms);

  Telemetry getTelemetry() const;
  ControlState state() const;
  FaultCode fault() const;

  void clearFault();

  RuntimeTuning tuning() const;
  bool setDutyToSpeedMpsPerDuty(float value);
  bool setMinStartDuty(float value);
  bool setMaxDuty(float value);
  bool setCommandSlewPerS(float value);

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

  void updateEstimator(float dt_s);
  ActiveSetpoint computeActiveSetpoint(uint32_t now_ms);
  void computeWheelCommandTargets(const ActiveSetpoint& setpoint);
  void applyCommandSlew(float dt_s);

  void enterStopping();
  void enterFault(FaultCode fault);
  bool outputsNearZero() const;

  static float clampf(float value, float lo, float hi);
  static float signf(float value);
  static float wrapAngleRad(float angle);

  RobotConfig cfg_ {};
  RuntimeTuning tuning_ {};

  MotorDriverL298N motor_;

  MotionCommand queue_[kQueueSize] {};
  size_t queue_head_ {0};
  size_t queue_tail_ {0};
  size_t queue_count_ {0};

  bool has_active_command_ {false};
  MotionCommand active_command_ {};
  TrapezoidProfile active_profile_ {};
  uint32_t active_start_ms_ {0};

  float model_left_speed_m_s_ {0.0f};
  float model_right_speed_m_s_ {0.0f};
  float estimated_forward_speed_m_s_ {0.0f};
  float estimated_distance_m_ {0.0f};
  float estimated_heading_rad_ {0.0f};

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
