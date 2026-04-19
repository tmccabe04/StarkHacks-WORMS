#include <Arduino.h>
#include <L298N.h>

// Pin map from current wiring.
constexpr uint8_t kLeftEn = 9;
constexpr uint8_t kLeftIn1 = 10;
constexpr uint8_t kLeftIn2 = 11;

constexpr uint8_t kRightEn = 14;
constexpr uint8_t kRightIn1 = 12;
constexpr uint8_t kRightIn2 = 13;

// Left polarity is inverted to match robot-forward convention used in motionControl.
constexpr bool kLeftInverted = true;
constexpr bool kRightInverted = false;

constexpr unsigned long kStageDurationMs = 2200;
constexpr unsigned long kStopDurationMs = 700;

constexpr uint8_t kPwmLow = 80;
constexpr uint8_t kPwmMedium = 140;
constexpr uint8_t kPwmHigh = 220;
constexpr unsigned long kLedBlinkMs = 180;

class MotorHarness {
public:
  MotorHarness()
    : left_(kLeftEn, kLeftIn1, kLeftIn2),
      right_(kRightEn, kRightIn1, kRightIn2) {}

  void begin() {
    left_.setSpeed(0);
    right_.setSpeed(0);
    left_.stop();
    right_.stop();
  }

  void setLeft(int16_t signed_pwm) {
    drive(&left_, signed_pwm, kLeftInverted);
  }

  void setRight(int16_t signed_pwm) {
    drive(&right_, signed_pwm, kRightInverted);
  }

  void stopAll() {
    left_.setSpeed(0);
    right_.setSpeed(0);
    left_.stop();
    right_.stop();
  }

private:
  static uint8_t absPwm(int16_t signed_pwm) {
    int16_t value = signed_pwm;
    if (value < 0) {
      value = -value;
    }
    if (value > 255) {
      value = 255;
    }
    return static_cast<uint8_t>(value);
  }

  static void drive(L298N* motor, int16_t signed_pwm, bool inverted) {
    if (motor == nullptr) {
      return;
    }

    if (signed_pwm == 0) {
      motor->setSpeed(0);
      motor->stop();
      return;
    }

    bool forward = signed_pwm > 0;
    if (inverted) {
      forward = !forward;
    }

    motor->setSpeed(absPwm(signed_pwm));
    if (forward) {
      motor->forward();
    } else {
      motor->backward();
    }
  }

  L298N left_;
  L298N right_;
};

struct Stage {
  const char* label;
  int16_t left_pwm;
  int16_t right_pwm;
  unsigned long run_ms;
  unsigned long stop_ms;
};

MotorHarness motors;

const Stage kStages[] = {
  {"L FWD LOW", +kPwmLow, 0, kStageDurationMs, kStopDurationMs},
  {"L FWD MED", +kPwmMedium, 0, kStageDurationMs, kStopDurationMs},
  {"L FWD HIGH", +kPwmHigh, 0, kStageDurationMs, kStopDurationMs},
  {"L REV LOW", -kPwmLow, 0, kStageDurationMs, kStopDurationMs},
  {"L REV MED", -kPwmMedium, 0, kStageDurationMs, kStopDurationMs},
  {"L REV HIGH", -kPwmHigh, 0, kStageDurationMs, kStopDurationMs},

  {"R FWD LOW", 0, +kPwmLow, kStageDurationMs, kStopDurationMs},
  {"R FWD MED", 0, +kPwmMedium, kStageDurationMs, kStopDurationMs},
  {"R FWD HIGH", 0, +kPwmHigh, kStageDurationMs, kStopDurationMs},
  {"R REV LOW", 0, -kPwmLow, kStageDurationMs, kStopDurationMs},
  {"R REV MED", 0, -kPwmMedium, kStageDurationMs, kStopDurationMs},
  {"R REV HIGH", 0, -kPwmHigh, kStageDurationMs, kStopDurationMs},

  {"BOTH FWD MIX", +kPwmMedium, +kPwmHigh, kStageDurationMs, kStopDurationMs},
  {"BOTH REV MIX", -kPwmHigh, -kPwmMedium, kStageDurationMs, kStopDurationMs},
  {"SPIN CW", +kPwmMedium, -kPwmMedium, kStageDurationMs, kStopDurationMs},
  {"SPIN CCW", -kPwmMedium, +kPwmMedium, kStageDurationMs, kStopDurationMs},
};

size_t stage_index = (sizeof(kStages) / sizeof(kStages[0])) - 1;
bool stage_running = false;
unsigned long stage_mark_ms = 0;

void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  // neopixelWrite expects GRB order.
  neopixelWrite(RGB_BUILTIN, g, r, b);
}

void updateStatusLed(bool running, unsigned long now_ms) {
  static unsigned long last_toggle_ms = 0;
  static bool blink_on = false;

  if (running) {
    if ((now_ms - last_toggle_ms) >= kLedBlinkMs) {
      last_toggle_ms = now_ms;
      blink_on = !blink_on;
    }
    if (blink_on) {
      setStatusLed(0, 80, 0);
    } else {
      setStatusLed(0, 0, 0);
    }
    return;
  }

  // Stopped phase between stages.
  setStatusLed(25, 0, 0);
}

void printStageHeader(const Stage& stage, size_t index) {
  Serial.printf(
    "[stage %u/%u] %s | L=%d R=%d\n",
    static_cast<unsigned>(index + 1),
    static_cast<unsigned>(sizeof(kStages) / sizeof(kStages[0])),
    stage.label,
    stage.left_pwm,
    stage.right_pwm
  );
}

void runStage(const Stage& stage) {
  motors.setLeft(stage.left_pwm);
  motors.setRight(stage.right_pwm);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  motors.begin();
  setStatusLed(0, 0, 0);

  Serial.println("motorTest start");
  Serial.println("ENA/ENB jumpers must be removed for PWM speed control.");
  Serial.println("This sketch loops forever through motor direction/speed stages.");
  Serial.println("Reset board to restart sequence.");

  // Start first stage immediately.
  stage_mark_ms = millis() - kStopDurationMs;
}

void loop() {
  const unsigned long now_ms = millis();
  const Stage& stage = kStages[stage_index];
  updateStatusLed(stage_running, now_ms);

  if (stage_running) {
    if ((now_ms - stage_mark_ms) >= stage.run_ms) {
      motors.stopAll();
      stage_running = false;
      stage_mark_ms = now_ms;
    }
    return;
  }

  if ((now_ms - stage_mark_ms) < stage.stop_ms) {
    return;
  }

  stage_index = (stage_index + 1) % (sizeof(kStages) / sizeof(kStages[0]));
  const Stage& next_stage = kStages[stage_index];
  printStageHeader(next_stage, stage_index);
  runStage(next_stage);
  stage_running = true;
  stage_mark_ms = now_ms;
}
