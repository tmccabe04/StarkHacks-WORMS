#include "open_loop_controller.h"

#include <math.h>

namespace {

constexpr float kMinDtS = 0.001f;
constexpr float kModelTimeConstantS = 0.09f;

}  // namespace

bool OpenLoopController::begin(const RobotConfig& config) {
  cfg_ = config;
  tuning_.duty_to_speed_m_s = cfg_.motor_model.duty_to_speed_m_s;
  tuning_.min_start_duty = cfg_.motor_model.min_start_duty;
  tuning_.max_duty = cfg_.motor_model.max_duty;
  tuning_.command_slew_per_s = cfg_.motor_model.command_slew_per_s;

  if (!motor_.begin(cfg_)) {
    enterFault(FaultCode::MotorInitFailed);
    return false;
  }
  motor_.setMaxDuty(tuning_.max_duty);

  state_ = ControlState::Idle;
  fault_ = FaultCode::None;
  last_tick_ms_ = millis();
  last_command_rx_ms_ = last_tick_ms_;

  telemetry_ = {};
  telemetry_.state = state_;
  telemetry_.fault = fault_;

  return true;
}

bool OpenLoopController::submitCommand(const MotionCommand& command) {
  const uint32_t now_ms = millis();
  last_command_rx_ms_ = now_ms;

  if (command.type == MotionCommandType::Stop) {
    clearQueue();
    has_active_command_ = false;
    active_command_ = command;
    enterStopping();
    return true;
  }

  MotionCommand normalized = command;
  normalized.created_ms = now_ms;

  if (!enqueue(normalized)) {
    enterFault(FaultCode::QueueFull);
    return false;
  }

  if (state_ == ControlState::Idle && !has_active_command_) {
    startNextCommand(now_ms);
  }

  return true;
}

void OpenLoopController::controlTick(uint32_t now_ms) {
  if (last_tick_ms_ == 0) {
    last_tick_ms_ = now_ms;
    return;
  }

  if (now_ms < last_tick_ms_) {
    return;
  }

  float dt_s = static_cast<float>(now_ms - last_tick_ms_) / 1000.0f;
  if (dt_s <= 0.0f) {
    return;
  }
  last_tick_ms_ = now_ms;

  if (dt_s > cfg_.safety.max_control_dt_s) {
    enterFault(FaultCode::ControlLoopOverrun);
  }
  if (dt_s < kMinDtS) {
    dt_s = kMinDtS;
  }

  if (state_ == ControlState::Executing && cfg_.safety.command_timeout_ms > 0) {
    if ((now_ms - last_command_rx_ms_) > cfg_.safety.command_timeout_ms) {
      enterFault(FaultCode::CommandTimeout);
    }
  }

  ActiveSetpoint setpoint {};

  switch (state_) {
    case ControlState::Fault:
      setpoint = {};
      break;

    case ControlState::Stopping:
      setpoint = {};
      if (outputsNearZero()) {
        if (!has_active_command_ && queue_count_ > 0) {
          startNextCommand(now_ms);
        } else if (!has_active_command_) {
          state_ = ControlState::Idle;
        }
      }
      break;

    case ControlState::Idle:
      if (!has_active_command_ && queue_count_ > 0) {
        startNextCommand(now_ms);
      }
      setpoint = {};
      break;

    case ControlState::Executing:
      setpoint = computeActiveSetpoint(now_ms);
      if (setpoint.command_done) {
        has_active_command_ = false;
        enterStopping();
      }
      break;
  }

  if (state_ == ControlState::Executing) {
    computeWheelCommandTargets(setpoint);
  } else {
    target_left_cmd_ = 0.0f;
    target_right_cmd_ = 0.0f;
  }

  applyCommandSlew(dt_s);
  motor_.writeNormalized(applied_left_cmd_, applied_right_cmd_);

  updateEstimator(dt_s);

  telemetry_.state = state_;
  telemetry_.fault = fault_;
  telemetry_.left_motor_cmd = applied_left_cmd_;
  telemetry_.right_motor_cmd = applied_right_cmd_;
  telemetry_.estimated_forward_speed_m_s = estimated_forward_speed_m_s_;
  telemetry_.estimated_distance_m = estimated_distance_m_;
  telemetry_.estimated_heading_deg = estimated_heading_rad_ * RAD_TO_DEG;
  telemetry_.active_command = active_command_;
}

Telemetry OpenLoopController::getTelemetry() const {
  return telemetry_;
}

ControlState OpenLoopController::state() const {
  return state_;
}

FaultCode OpenLoopController::fault() const {
  return fault_;
}

void OpenLoopController::clearFault() {
  if (state_ != ControlState::Fault) {
    return;
  }

  fault_ = FaultCode::None;
  state_ = ControlState::Idle;
  clearQueue();
  has_active_command_ = false;
  target_left_cmd_ = 0.0f;
  target_right_cmd_ = 0.0f;
  applied_left_cmd_ = 0.0f;
  applied_right_cmd_ = 0.0f;
  motor_.stop();
}

RuntimeTuning OpenLoopController::tuning() const {
  return tuning_;
}

bool OpenLoopController::setDutyToSpeedMpsPerDuty(float value) {
  if (!(value > 0.05f && value <= 5.0f)) {
    return false;
  }
  tuning_.duty_to_speed_m_s = value;
  return true;
}

bool OpenLoopController::setMinStartDuty(float value) {
  if (!(value >= 0.0f && value <= 1.0f)) {
    return false;
  }
  tuning_.min_start_duty = value;
  return true;
}

bool OpenLoopController::setMaxDuty(float value) {
  if (!(value > 0.0f && value <= 1.0f)) {
    return false;
  }
  tuning_.max_duty = value;
  motor_.setMaxDuty(tuning_.max_duty);
  return true;
}

bool OpenLoopController::setCommandSlewPerS(float value) {
  if (!(value > 0.05f && value <= 20.0f)) {
    return false;
  }
  tuning_.command_slew_per_s = value;
  return true;
}

bool OpenLoopController::startNextCommand(uint32_t now_ms) {
  MotionCommand next {};
  if (!dequeue(&next)) {
    return false;
  }

  float distance_or_angle = 0.0f;
  float max_speed = 0.0f;
  float max_accel = 0.0f;

  if (next.type == MotionCommandType::MoveCm) {
    distance_or_angle = next.value / 100.0f;
    max_speed = (next.max_speed > 0.0f) ? (next.max_speed / 100.0f) : cfg_.limits.move_max_speed_m_s;
    max_accel = (next.max_accel > 0.0f) ? (next.max_accel / 100.0f) : cfg_.limits.move_max_accel_m_s2;
  } else if (next.type == MotionCommandType::TurnDeg) {
    distance_or_angle = next.value * DEG_TO_RAD;
    max_speed = (next.max_speed > 0.0f) ? (next.max_speed * DEG_TO_RAD) : cfg_.limits.turn_max_speed_rad_s;
    max_accel = (next.max_accel > 0.0f) ? (next.max_accel * DEG_TO_RAD) : cfg_.limits.turn_max_accel_rad_s2;
  }

  active_profile_ = MotionPlanner::build(distance_or_angle, max_speed, max_accel);
  if (!active_profile_.valid) {
    has_active_command_ = false;
    return false;
  }

  active_command_ = next;
  has_active_command_ = true;
  active_start_ms_ = now_ms;

  state_ = ControlState::Executing;
  telemetry_.command_progress = 0.0f;
  return true;
}

void OpenLoopController::clearQueue() {
  queue_head_ = 0;
  queue_tail_ = 0;
  queue_count_ = 0;
}

bool OpenLoopController::enqueue(const MotionCommand& cmd) {
  if (queue_count_ >= kQueueSize) {
    return false;
  }

  queue_[queue_tail_] = cmd;
  queue_tail_ = (queue_tail_ + 1) % kQueueSize;
  ++queue_count_;
  return true;
}

bool OpenLoopController::dequeue(MotionCommand* cmd) {
  if (queue_count_ == 0 || cmd == nullptr) {
    return false;
  }

  *cmd = queue_[queue_head_];
  queue_head_ = (queue_head_ + 1) % kQueueSize;
  --queue_count_;
  return true;
}

void OpenLoopController::updateEstimator(float dt_s) {
  const float tau = kModelTimeConstantS;
  const float desired_left = applied_left_cmd_ * tuning_.duty_to_speed_m_s;
  const float desired_right = applied_right_cmd_ * tuning_.duty_to_speed_m_s;

  model_left_speed_m_s_ += (desired_left - model_left_speed_m_s_) * (dt_s / tau);
  model_right_speed_m_s_ += (desired_right - model_right_speed_m_s_) * (dt_s / tau);

  estimated_forward_speed_m_s_ = 0.5f * (model_left_speed_m_s_ + model_right_speed_m_s_);
  estimated_distance_m_ += estimated_forward_speed_m_s_ * dt_s;

  const float omega_rad_s = (model_right_speed_m_s_ - model_left_speed_m_s_) / cfg_.wheel_base_m;
  estimated_heading_rad_ = wrapAngleRad(estimated_heading_rad_ + (omega_rad_s * dt_s));
}

OpenLoopController::ActiveSetpoint OpenLoopController::computeActiveSetpoint(uint32_t now_ms) {
  ActiveSetpoint setpoint {};

  if (!has_active_command_) {
    return setpoint;
  }

  const float elapsed_s = static_cast<float>(now_ms - active_start_ms_) / 1000.0f;
  const TrapezoidSample profile_sample = MotionPlanner::sample(active_profile_, elapsed_s);

  const float abs_distance = fabsf(active_profile_.distance);
  if (abs_distance > 1e-6f) {
    telemetry_.command_progress = clampf(fabsf(profile_sample.position) / abs_distance, 0.0f, 1.0f);
  } else {
    telemetry_.command_progress = 1.0f;
  }

  if (active_command_.type == MotionCommandType::MoveCm) {
    setpoint.target_linear_m_s = profile_sample.velocity;
    setpoint.target_omega_rad_s = 0.0f;
    setpoint.command_done = profile_sample.done;
  } else if (active_command_.type == MotionCommandType::TurnDeg) {
    setpoint.target_linear_m_s = 0.0f;
    setpoint.target_omega_rad_s = profile_sample.velocity;
    setpoint.command_done = profile_sample.done;
  }

  return setpoint;
}

void OpenLoopController::computeWheelCommandTargets(const ActiveSetpoint& setpoint) {
  const float half_wheelbase = 0.5f * cfg_.wheel_base_m;

  const float target_left_speed = setpoint.target_linear_m_s - (setpoint.target_omega_rad_s * half_wheelbase);
  const float target_right_speed = setpoint.target_linear_m_s + (setpoint.target_omega_rad_s * half_wheelbase);

  telemetry_.target_left_speed_m_s = target_left_speed;
  telemetry_.target_right_speed_m_s = target_right_speed;

  float left_cmd = target_left_speed / tuning_.duty_to_speed_m_s;
  float right_cmd = target_right_speed / tuning_.duty_to_speed_m_s;

  left_cmd = clampf(left_cmd, -1.0f, 1.0f);
  right_cmd = clampf(right_cmd, -1.0f, 1.0f);

  if (fabsf(target_left_speed) > 0.005f && fabsf(left_cmd) < tuning_.min_start_duty) {
    left_cmd = signf(left_cmd == 0.0f ? target_left_speed : left_cmd) * tuning_.min_start_duty;
  }
  if (fabsf(target_right_speed) > 0.005f && fabsf(right_cmd) < tuning_.min_start_duty) {
    right_cmd = signf(right_cmd == 0.0f ? target_right_speed : right_cmd) * tuning_.min_start_duty;
  }

  if (fabsf(target_left_speed) < 0.005f && fabsf(target_right_speed) < 0.005f) {
    left_cmd = 0.0f;
    right_cmd = 0.0f;
  }

  target_left_cmd_ = left_cmd;
  target_right_cmd_ = right_cmd;
}

void OpenLoopController::applyCommandSlew(float dt_s) {
  const float max_delta = tuning_.command_slew_per_s * dt_s;

  const float left_delta = clampf(target_left_cmd_ - applied_left_cmd_, -max_delta, max_delta);
  const float right_delta = clampf(target_right_cmd_ - applied_right_cmd_, -max_delta, max_delta);

  applied_left_cmd_ += left_delta;
  applied_right_cmd_ += right_delta;

  if (fabsf(target_left_cmd_) < cfg_.safety.stop_deadband_cmd &&
      fabsf(applied_left_cmd_) < cfg_.safety.stop_deadband_cmd) {
    applied_left_cmd_ = 0.0f;
  }
  if (fabsf(target_right_cmd_) < cfg_.safety.stop_deadband_cmd &&
      fabsf(applied_right_cmd_) < cfg_.safety.stop_deadband_cmd) {
    applied_right_cmd_ = 0.0f;
  }
}

void OpenLoopController::enterStopping() {
  state_ = ControlState::Stopping;
}

void OpenLoopController::enterFault(FaultCode fault) {
  fault_ = fault;
  state_ = ControlState::Fault;
  clearQueue();
  has_active_command_ = false;
  telemetry_.command_progress = 0.0f;

  target_left_cmd_ = 0.0f;
  target_right_cmd_ = 0.0f;
  applied_left_cmd_ = 0.0f;
  applied_right_cmd_ = 0.0f;
  motor_.stop();
}

bool OpenLoopController::outputsNearZero() const {
  return fabsf(applied_left_cmd_) < cfg_.safety.stop_deadband_cmd &&
    fabsf(applied_right_cmd_) < cfg_.safety.stop_deadband_cmd &&
    fabsf(target_left_cmd_) < cfg_.safety.stop_deadband_cmd &&
    fabsf(target_right_cmd_) < cfg_.safety.stop_deadband_cmd;
}

float OpenLoopController::clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

float OpenLoopController::signf(float value) {
  if (value > 0.0f) return 1.0f;
  if (value < 0.0f) return -1.0f;
  return 0.0f;
}

float OpenLoopController::wrapAngleRad(float angle) {
  while (angle > PI) {
    angle -= TWO_PI;
  }
  while (angle < -PI) {
    angle += TWO_PI;
  }
  return angle;
}
