#include "motion_controller.h"

#include <math.h>

namespace {

constexpr float kMinDt = 0.001f;

float signf(float v) {
  if (v > 0.0f) return 1.0f;
  if (v < 0.0f) return -1.0f;
  return 0.0f;
}

}  // namespace

bool MotionController::begin(const RobotConfig& config) {
  cfg_ = config;

  move_heading_pid_.configure(cfg_.move_heading_pid);
  turn_heading_pid_.configure(cfg_.turn_heading_pid);
  left_speed_pi_.configure(cfg_.left_speed_pi);
  right_speed_pi_.configure(cfg_.right_speed_pi);

  if (!motor_.begin(cfg_)) {
    enterFault(FaultCode::MotorInitFailed);
    return false;
  }

  // IMU bypassed
  // if (!imu_.begin(cfg_)) {
  //   enterFault(FaultCode::ImuUnavailable);
  //   return false;
  // }

  state_ = ControlState::Idle;
  fault_ = FaultCode::None;
  last_tick_ms_ = millis();
  last_command_rx_ms_ = last_tick_ms_;
  telemetry_ = {};
  telemetry_.state = state_;
  telemetry_.fault = fault_;

  return true;
}

bool MotionController::submitCommand(const MotionCommand& command) {
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

void MotionController::controlTick(uint32_t now_ms) {
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
  if (dt_s < kMinDt) {
    dt_s = kMinDt;
  }

  ImuSample imu_sample = {};
  imu_sample.yaw_deg = 0.0f;
  imu_sample.yaw_unwrapped_rad = 0.0f;
  imu_sample.yaw_rate_rad_s = 0.0f;
  imu_sample.forward_accel_m_s2 = 0.0f;
  imu_sample.valid = true;
  const bool imu_ok = true;

  telemetry_.imu_ok = imu_ok;
  telemetry_.last_imu_update_ms = now_ms;
  telemetry_.yaw_deg = imu_sample.yaw_deg;
  telemetry_.yaw_unwrapped_rad = imu_sample.yaw_unwrapped_rad;
  telemetry_.yaw_rate_rad_s = imu_sample.yaw_rate_rad_s;
  telemetry_.forward_accel_m_s2 = imu_sample.forward_accel_m_s2;

  // ignore the IMU
  // if (!imu_ok) {
  //   enterFault(imu_sample.valid ? FaultCode::ImuStale : FaultCode::ImuUnavailable);
  // }

  if (state_ == ControlState::Executing && cfg_.safety.command_timeout_ms > 0) {
    if ((now_ms - last_command_rx_ms_) > cfg_.safety.command_timeout_ms) {
      enterFault(FaultCode::CommandTimeout);
    }
  }

  updateEstimator(dt_s, imu_sample);

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
      setpoint = computeActiveSetpoint(dt_s, now_ms, imu_sample);
      if (setpoint.command_done) {
        has_active_command_ = false;
        enterStopping();
      }
      break;
  }

  if (state_ == ControlState::Executing) {
    computeWheelCommandTargets(dt_s, setpoint, imu_sample);
  } else {
    target_left_cmd_ = 0.0f;
    target_right_cmd_ = 0.0f;
    left_speed_pi_.reset();
    right_speed_pi_.reset();
  }

  applyCommandSlew(dt_s);

  telemetry_.state = state_;
  telemetry_.fault = fault_;
  telemetry_.left_motor_cmd = applied_left_cmd_;
  telemetry_.right_motor_cmd = applied_right_cmd_;
  telemetry_.estimated_forward_speed_m_s = estimated_forward_speed_m_s_;
  telemetry_.estimated_distance_m = estimated_distance_m_;
  telemetry_.active_command = active_command_;
}

Telemetry MotionController::getTelemetry() const {
  return telemetry_;
}

ControlState MotionController::state() const {
  return state_;
}

FaultCode MotionController::fault() const {
  return fault_;
}

void MotionController::clearFault() {
  if (state_ != ControlState::Fault) {
    return;
  }

  fault_ = FaultCode::None;
  if (outputsNearZero()) {
    state_ = ControlState::Idle;
  } else {
    state_ = ControlState::Stopping;
  }
}

bool MotionController::startNextCommand(uint32_t now_ms) {
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
  turn_settle_ticks_ = 0;

  command_start_yaw_rad_ = telemetry_.yaw_unwrapped_rad;
  command_target_heading_rad_ = command_start_yaw_rad_;
  command_start_distance_m_ = estimated_distance_m_;

  move_heading_pid_.reset();
  turn_heading_pid_.reset();
  left_speed_pi_.reset();
  right_speed_pi_.reset();

  state_ = ControlState::Executing;
  return true;
}

void MotionController::clearQueue() {
  queue_head_ = 0;
  queue_tail_ = 0;
  queue_count_ = 0;
}

bool MotionController::enqueue(const MotionCommand& cmd) {
  if (queue_count_ >= kQueueSize) {
    return false;
  }

  queue_[queue_tail_] = cmd;
  queue_tail_ = (queue_tail_ + 1) % kQueueSize;
  ++queue_count_;
  return true;
}

bool MotionController::dequeue(MotionCommand* cmd) {
  if (queue_count_ == 0) {
    return false;
  }

  *cmd = queue_[queue_head_];
  queue_head_ = (queue_head_ + 1) % kQueueSize;
  --queue_count_;
  return true;
}

void MotionController::updateEstimator(float dt_s, const ImuSample& imu_sample) {
  const float tau = cfg_.motor_model.speed_time_constant_s > 0.01f ? cfg_.motor_model.speed_time_constant_s : 0.01f;
  const float duty_to_speed = cfg_.motor_model.duty_to_speed_m_s;

  const float desired_model_left = applied_left_cmd_ * duty_to_speed;
  const float desired_model_right = applied_right_cmd_ * duty_to_speed;

  model_left_speed_m_s_ += (desired_model_left - model_left_speed_m_s_) * (dt_s / tau);
  model_right_speed_m_s_ += (desired_model_right - model_right_speed_m_s_) * (dt_s / tau);

  const float model_forward = 0.5f * (model_left_speed_m_s_ + model_right_speed_m_s_);
  const float accel_integrated = estimated_forward_speed_m_s_ + (imu_sample.forward_accel_m_s2 * dt_s);

  const float blend = clampf(cfg_.motor_model.accel_blend, 0.0f, 1.0f);
  estimated_forward_speed_m_s_ = ((1.0f - blend) * accel_integrated) + (blend * model_forward);

  const float speed_limit = cfg_.motor_model.duty_to_speed_m_s * cfg_.motor_model.max_duty * 1.2f;
  estimated_forward_speed_m_s_ = clampf(estimated_forward_speed_m_s_, -speed_limit, speed_limit);

  estimated_distance_m_ += estimated_forward_speed_m_s_ * dt_s;

  telemetry_.estimated_left_speed_m_s = estimated_forward_speed_m_s_ - (0.5f * cfg_.wheel_base_m * imu_sample.yaw_rate_rad_s);
  telemetry_.estimated_right_speed_m_s = estimated_forward_speed_m_s_ + (0.5f * cfg_.wheel_base_m * imu_sample.yaw_rate_rad_s);
}

MotionController::ActiveSetpoint MotionController::computeActiveSetpoint(
  float dt_s,
  uint32_t now_ms,
  const ImuSample& imu_sample
) {
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
    const float heading_error = 0; // wrapAngleRad(command_target_heading_rad_ - imu_sample.yaw_unwrapped_rad);
    float omega_correction = move_heading_pid_.update(heading_error, dt_s);
    omega_correction = clampf(
      omega_correction,
      -cfg_.motor_model.heading_correction_limit_rad_s,
      cfg_.motor_model.heading_correction_limit_rad_s
    );

    setpoint.target_linear_m_s = profile_sample.velocity;
    setpoint.target_omega_rad_s = omega_correction;

    if (profile_sample.done) {
      setpoint.command_done = true;
    }
  } else if (active_command_.type == MotionCommandType::TurnDeg) {
    const float actual_relative_yaw = command_start_yaw_rad_;// imu_sample.yaw_unwrapped_rad - command_start_yaw_rad_;
    const float heading_error = profile_sample.position - actual_relative_yaw;

    float omega_cmd = profile_sample.velocity + turn_heading_pid_.update(heading_error, dt_s);
    omega_cmd = clampf(
      omega_cmd,
      -cfg_.limits.turn_max_speed_rad_s,
      cfg_.limits.turn_max_speed_rad_s
    );

    setpoint.target_linear_m_s = 0.0f;
    setpoint.target_omega_rad_s = omega_cmd;

    if (profile_sample.done &&
        fabsf(heading_error) < cfg_.safety.turn_settle_error_rad) {
      if (turn_settle_ticks_ < 255) {
        ++turn_settle_ticks_;
      }
    } else {
      turn_settle_ticks_ = 0;
    }

    if (profile_sample.done && turn_settle_ticks_ >= 6) {
      setpoint.command_done = true;
    }
  }

  return setpoint;
}

void MotionController::computeWheelCommandTargets(
  float dt_s,
  const ActiveSetpoint& setpoint,
  const ImuSample& imu_sample
) {
  const float half_wheelbase = 0.5f * cfg_.wheel_base_m;

  const float target_left_speed = setpoint.target_linear_m_s - (setpoint.target_omega_rad_s * half_wheelbase);
  const float target_right_speed = setpoint.target_linear_m_s + (setpoint.target_omega_rad_s * half_wheelbase);

  telemetry_.target_left_speed_m_s = target_left_speed;
  telemetry_.target_right_speed_m_s = target_right_speed;

  const float estimated_left_speed = telemetry_.estimated_left_speed_m_s;
  const float estimated_right_speed = telemetry_.estimated_right_speed_m_s;

  const float left_error = target_left_speed - estimated_left_speed;
  const float right_error = target_right_speed - estimated_right_speed;

  const float ff_left = target_left_speed / cfg_.motor_model.duty_to_speed_m_s;
  const float ff_right = target_right_speed / cfg_.motor_model.duty_to_speed_m_s;

  float left_cmd = ff_left + left_speed_pi_.update(left_error, dt_s);
  float right_cmd = ff_right + right_speed_pi_.update(right_error, dt_s);

  const float max_cmd = cfg_.motor_model.max_duty;
  left_cmd = clampf(left_cmd, -max_cmd, max_cmd);
  right_cmd = clampf(right_cmd, -max_cmd, max_cmd);

  if (fabsf(target_left_speed) > 0.01f && fabsf(left_cmd) < cfg_.motor_model.min_start_duty) {
    left_cmd = signf(left_cmd == 0.0f ? target_left_speed : left_cmd) * cfg_.motor_model.min_start_duty;
  }
  if (fabsf(target_right_speed) > 0.01f && fabsf(right_cmd) < cfg_.motor_model.min_start_duty) {
    right_cmd = signf(right_cmd == 0.0f ? target_right_speed : right_cmd) * cfg_.motor_model.min_start_duty;
  }

  if (fabsf(target_left_speed) < 0.005f && fabsf(target_right_speed) < 0.005f) {
    left_cmd = 0.0f;
    right_cmd = 0.0f;
  }

  target_left_cmd_ = left_cmd;
  target_right_cmd_ = right_cmd;
}

void MotionController::applyCommandSlew(float dt_s) {
  const float max_delta = cfg_.motor_model.command_slew_per_s * dt_s;

  const float left_delta = clampf(target_left_cmd_ - applied_left_cmd_, -max_delta, max_delta);
  const float right_delta = clampf(target_right_cmd_ - applied_right_cmd_, -max_delta, max_delta);

  applied_left_cmd_ += left_delta;
  applied_right_cmd_ += right_delta;

  if (fabsf(target_left_cmd_) < cfg_.safety.stop_deadband_cmd && fabsf(applied_left_cmd_) < cfg_.safety.stop_deadband_cmd) {
    applied_left_cmd_ = 0.0f;
  }
  if (fabsf(target_right_cmd_) < cfg_.safety.stop_deadband_cmd && fabsf(applied_right_cmd_) < cfg_.safety.stop_deadband_cmd) {
    applied_right_cmd_ = 0.0f;
  }

  motor_.writeNormalized(applied_left_cmd_, applied_right_cmd_);
}

void MotionController::enterStopping() {
  target_left_cmd_ = 0.0f;
  target_right_cmd_ = 0.0f;
  state_ = ControlState::Stopping;
}

void MotionController::enterFault(FaultCode fault) {
  if (state_ == ControlState::Fault && fault_ != FaultCode::None) {
    return;
  }

  fault_ = fault;
  state_ = ControlState::Fault;

  clearQueue();
  has_active_command_ = false;

  target_left_cmd_ = 0.0f;
  target_right_cmd_ = 0.0f;

  move_heading_pid_.reset();
  turn_heading_pid_.reset();
  left_speed_pi_.reset();
  right_speed_pi_.reset();
}

bool MotionController::outputsNearZero() const {
  return fabsf(applied_left_cmd_) < cfg_.safety.stop_deadband_cmd &&
    fabsf(applied_right_cmd_) < cfg_.safety.stop_deadband_cmd;
}

float MotionController::clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

float MotionController::wrapAngleRad(float angle) {
  while (angle > PI) {
    angle -= (2.0f * PI);
  }
  while (angle < -PI) {
    angle += (2.0f * PI);
  }
  return angle;
}
