#include "motion_planner.h"

#include <math.h>

namespace {

float signf(float value) {
  if (value > 0.0f) return 1.0f;
  if (value < 0.0f) return -1.0f;
  return 0.0f;
}

}  // namespace

TrapezoidProfile MotionPlanner::build(float distance, float max_speed, float max_accel) {
  TrapezoidProfile profile;

  const float abs_distance = fabsf(distance);
  if (abs_distance <= 0.0f || max_speed <= 0.0f || max_accel <= 0.0f) {
    return profile;
  }

  profile.distance = distance;
  profile.max_speed = max_speed;
  profile.max_accel = max_accel;

  const float accel_time_nominal = max_speed / max_accel;
  const float accel_distance_nominal = 0.5f * max_accel * accel_time_nominal * accel_time_nominal;

  if (2.0f * accel_distance_nominal >= abs_distance) {
    profile.accel_time_s = sqrtf(abs_distance / max_accel);
    profile.peak_speed = max_accel * profile.accel_time_s;
    profile.accel_distance = 0.5f * max_accel * profile.accel_time_s * profile.accel_time_s;
    profile.cruise_distance = 0.0f;
    profile.cruise_time_s = 0.0f;
  } else {
    profile.accel_time_s = accel_time_nominal;
    profile.peak_speed = max_speed;
    profile.accel_distance = accel_distance_nominal;
    profile.cruise_distance = abs_distance - (2.0f * accel_distance_nominal);
    profile.cruise_time_s = profile.cruise_distance / max_speed;
  }

  profile.total_time_s = (2.0f * profile.accel_time_s) + profile.cruise_time_s;
  profile.valid = true;
  return profile;
}

TrapezoidSample MotionPlanner::sample(const TrapezoidProfile& profile, float elapsed_s) {
  TrapezoidSample out;

  if (!profile.valid) {
    out.done = true;
    return out;
  }

  const float dir = signf(profile.distance);
  const float t = elapsed_s < 0.0f ? 0.0f : elapsed_s;

  if (t >= profile.total_time_s) {
    out.position = profile.distance;
    out.velocity = 0.0f;
    out.done = true;
    return out;
  }

  const float t_accel_end = profile.accel_time_s;
  const float t_cruise_end = profile.accel_time_s + profile.cruise_time_s;

  float pos_abs = 0.0f;
  float vel_abs = 0.0f;

  if (t <= t_accel_end) {
    pos_abs = 0.5f * profile.max_accel * t * t;
    vel_abs = profile.max_accel * t;
  } else if (t <= t_cruise_end) {
    const float t_cruise = t - t_accel_end;
    pos_abs = profile.accel_distance + (profile.peak_speed * t_cruise);
    vel_abs = profile.peak_speed;
  } else {
    const float t_decel = t - t_cruise_end;
    pos_abs = profile.accel_distance + profile.cruise_distance +
      (profile.peak_speed * t_decel) - (0.5f * profile.max_accel * t_decel * t_decel);
    vel_abs = profile.peak_speed - (profile.max_accel * t_decel);
    if (vel_abs < 0.0f) {
      vel_abs = 0.0f;
    }
  }

  out.position = dir * pos_abs;
  out.velocity = dir * vel_abs;
  out.done = false;
  return out;
}
