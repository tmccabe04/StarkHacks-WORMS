#pragma once

struct TrapezoidProfile {
  float distance {0.0f};
  float max_speed {0.0f};
  float max_accel {0.0f};

  float accel_time_s {0.0f};
  float cruise_time_s {0.0f};
  float total_time_s {0.0f};

  float accel_distance {0.0f};
  float cruise_distance {0.0f};
  float peak_speed {0.0f};

  bool valid {false};
};

struct TrapezoidSample {
  float position {0.0f};
  float velocity {0.0f};
  bool done {false};
};

class MotionPlanner {
public:
  static TrapezoidProfile build(float distance, float max_speed, float max_accel);
  static TrapezoidSample sample(const TrapezoidProfile& profile, float elapsed_s);
};
