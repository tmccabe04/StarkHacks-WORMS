#include "src/minion.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t kMpuAddress = 0x68;  // AD0 grounded
constexpr uint8_t kI2cSdaPin = 5;
constexpr uint8_t kI2cSclPin = 4;
constexpr uint32_t kI2cClockHz = 400000;

constexpr float kMagDeclinationDeg = 1.63f;
constexpr uint16_t kDefaultOrientationHz = 10;
constexpr uint16_t kDefaultRawHz = 20;
constexpr uint16_t kMinStreamHz = 1;
constexpr uint16_t kMaxStreamHz = 100;
constexpr uint32_t kStaleWarnMs = 250;
constexpr float kLowRateWarnHz = 40.0f;

MPU9250 mpu;

enum class StreamMode {
  None,
  Orientation,
  Raw,
};

enum class LedMode {
  Error,
  Idle,
  Streaming,
  Calibrating,
};

bool mpu_ready = false;
StreamMode stream_mode = StreamMode::None;
LedMode led_mode = LedMode::Error;

float yaw_zero_deg = 0.0f;
uint16_t stream_hz = kDefaultOrientationHz;

uint32_t last_stream_print_ms = 0;
uint32_t last_health_print_ms = 0;
uint32_t last_mpu_update_ms = 0;

uint32_t total_update_ok = 0;
uint32_t total_update_fail = 0;
uint32_t health_window_updates = 0;
uint32_t health_window_failures = 0;
uint32_t health_window_start_ms = 0;

char line_buffer[96] = {0};
size_t line_length = 0;

void setLedMode(LedMode mode) {
  if (mode == led_mode) {
    return;
  }

  led_mode = mode;
  switch (mode) {
    case LedMode::Error:
      setRGB(255, 0, 0);
      break;
    case LedMode::Idle:
      setRGB(160, 120, 0);
      break;
    case LedMode::Streaming:
      setRGB(0, 180, 0);
      break;
    case LedMode::Calibrating:
      setRGB(0, 0, 220);
      break;
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h                : help");
  Serial.println("  s                : setup + health summary");
  Serial.println("  o [hz]           : orientation stream (yaw/pitch/roll)");
  Serial.println("  r [hz]           : raw stream (acc/gyro/mag)");
  Serial.println("  x                : stop stream");
  Serial.println("  z                : zero heading (relative yaw reference)");
  Serial.println("  c                : quick accel+gyro calibration (stationary)");
  Serial.println("  m                : magnetometer calibration (figure-eight)");
  Serial.println("  b                : retry MPU bring-up");
  Serial.println();
}

void printI2cScan() {
  Serial.println("I2C scan start...");
  uint8_t found_count = 0;
  bool found_mpu_address = false;

  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  found: 0x%02X\n", addr);
      ++found_count;
      if (addr == kMpuAddress) {
        found_mpu_address = true;
      }
    }
  }

  if (found_count == 0) {
    Serial.println("  no I2C devices found");
  }

  if (!found_mpu_address) {
    Serial.printf("WARNING: expected MPU9250 at 0x%02X not detected during scan\n", kMpuAddress);
  }

  Serial.println("I2C scan done.");
}

MPU9250Setting buildMpuSetting() {
  MPU9250Setting setting;

  // Sample rate should be >= 2x selected DLPF.
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  return setting;
}

bool beginMpu() {
  Serial.printf("Bringing up MPU9250 at 0x%02X (SDA=%u, SCL=%u, AD0=GND, FSYNC=GND) ...\n", kMpuAddress, kI2cSdaPin, kI2cSclPin);

  MPU9250Setting setting = buildMpuSetting();
  mpu.verbose(true);
  bool setup_ok = mpu.setup(kMpuAddress, setting, Wire, true);
  if (!setup_ok) {
    const bool imu_ok = mpu.isConnectedMPU9250();
    const bool mag_ok = mpu.isConnectedAK8963();
    Serial.printf("Primary setup failed. diag: imu_ok=%s mag_ok=%s\n", imu_ok ? "yes" : "no", mag_ok ? "yes" : "no");

    if (imu_ok && !mag_ok) {
      Serial.println("Retrying in IMU-only mode (AK8963 optional) ...");
      setup_ok = mpu.setup(kMpuAddress, setting, Wire, false);
    }
  }

  if (!setup_ok) {
    Serial.println("ERROR: mpu.setup() failed.");
    Serial.println("Check wiring/power and run command 'b' to retry.");
    mpu_ready = false;
    stream_mode = StreamMode::None;
    setLedMode(LedMode::Error);
    return false;
  }

  mpu.setMagneticDeclination(kMagDeclinationDeg);
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(15);

  mpu_ready = true;
  setLedMode(LedMode::Idle);

  // Reset statistics window.
  const uint32_t now = millis();
  total_update_ok = 0;
  total_update_fail = 0;
  health_window_updates = 0;
  health_window_failures = 0;
  health_window_start_ms = now;
  last_mpu_update_ms = now;
  last_health_print_ms = now;

  if (mpu.hasMagnetometer()) {
    Serial.println("MPU setup OK (AK8963 detected).");
  } else {
    Serial.println("MPU setup OK in IMU-only mode (no AK8963).");
  }
  return true;
}

void printSummary() {
  const uint32_t now = millis();
  const uint32_t stale_ms = now - last_mpu_update_ms;

  float window_rate_hz = 0.0f;
  const uint32_t window_elapsed_ms = now - health_window_start_ms;
  if (window_elapsed_ms > 0) {
    window_rate_hz = (1000.0f * static_cast<float>(health_window_updates)) / static_cast<float>(window_elapsed_ms);
  }

  Serial.println();
  Serial.println("---- MPU SUMMARY ----");
  Serial.printf("ready=%s stream=%s stream_hz=%u\n",
    mpu_ready ? "yes" : "no",
    (stream_mode == StreamMode::Orientation) ? "orientation" : (stream_mode == StreamMode::Raw) ? "raw" : "none",
    stream_hz);
  Serial.printf("mag_present=%s\n", mpu.hasMagnetometer() ? "yes" : "no");
  Serial.printf("last_update_age_ms=%lu window_rate_hz=%.1f\n", static_cast<unsigned long>(stale_ms), window_rate_hz);
  Serial.printf("updates_ok=%lu updates_fail=%lu\n", static_cast<unsigned long>(total_update_ok), static_cast<unsigned long>(total_update_fail));
  if (mpu_ready) {
    Serial.printf("yaw=%.2f yaw_rel=%.2f pitch=%.2f roll=%.2f temp=%.2fC\n",
      mpu.getYaw(),
      mpu.getYaw() - yaw_zero_deg,
      mpu.getPitch(),
      mpu.getRoll(),
      mpu.getTemperature());
  }
  Serial.println("---------------------");
  Serial.println();
}

uint16_t clampStreamHz(long hz_value) {
  if (hz_value < static_cast<long>(kMinStreamHz)) {
    return kMinStreamHz;
  }
  if (hz_value > static_cast<long>(kMaxStreamHz)) {
    return kMaxStreamHz;
  }
  return static_cast<uint16_t>(hz_value);
}

uint16_t parseOptionalHz(const char* args, uint16_t fallback_hz) {
  if (args == nullptr) {
    return fallback_hz;
  }

  while (*args == ' ' || *args == '\t') {
    ++args;
  }

  if (*args == '\0') {
    return fallback_hz;
  }

  char* end_ptr = nullptr;
  const long parsed = strtol(args, &end_ptr, 10);
  if (end_ptr == args) {
    Serial.println("Invalid hz argument, using previous value.");
    return fallback_hz;
  }

  return clampStreamHz(parsed);
}

void stopStreaming() {
  stream_mode = StreamMode::None;
  setLedMode(mpu_ready ? LedMode::Idle : LedMode::Error);
  Serial.println("Stream stopped.");
}

void startOrientationStream(uint16_t hz) {
  if (!mpu_ready) {
    Serial.println("MPU not ready. Use 'b' to retry setup.");
    return;
  }

  stream_mode = StreamMode::Orientation;
  stream_hz = hz;
  last_stream_print_ms = 0;
  setLedMode(LedMode::Streaming);
  Serial.printf("Orientation stream started at %u Hz.\n", stream_hz);
}

void startRawStream(uint16_t hz) {
  if (!mpu_ready) {
    Serial.println("MPU not ready. Use 'b' to retry setup.");
    return;
  }

  stream_mode = StreamMode::Raw;
  stream_hz = hz;
  last_stream_print_ms = 0;
  setLedMode(LedMode::Streaming);
  Serial.printf("Raw stream started at %u Hz.\n", stream_hz);
}

void runQuickCalibration() {
  if (!mpu_ready) {
    Serial.println("MPU not ready. Use 'b' to retry setup.");
    return;
  }

  setLedMode(LedMode::Calibrating);
  stream_mode = StreamMode::None;

  Serial.println("Quick accel+gyro calibration in 3 seconds.");
  Serial.println("Keep sensor still on a flat surface.");
  delay(3000);

  mpu.calibrateAccelGyro();
  Serial.println("Accel+gyro calibration done.");

  setLedMode(LedMode::Idle);
}

void runMagCalibration() {
  if (!mpu_ready) {
    Serial.println("MPU not ready. Use 'b' to retry setup.");
    return;
  }
  if (!mpu.hasMagnetometer()) {
    Serial.println("Mag calibration unavailable: AK8963 not detected.");
    return;
  }

  setLedMode(LedMode::Calibrating);
  stream_mode = StreamMode::None;

  Serial.println("Mag calibration in 3 seconds.");
  Serial.println("Move sensor in a figure-eight during calibration.");
  delay(3000);

  mpu.calibrateMag();
  Serial.println("Mag calibration done.");

  setLedMode(LedMode::Idle);
}

void zeroHeading() {
  if (!mpu_ready) {
    Serial.println("MPU not ready. Use 'b' to retry setup.");
    return;
  }

  yaw_zero_deg = mpu.getYaw();
  Serial.printf("Heading zero set to %.2f deg.\n", yaw_zero_deg);
}

void handleCommand(const char* line) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }

  const char cmd = static_cast<char>(tolower(line[0]));
  const char* args = line + 1;

  switch (cmd) {
    case 'h':
      printHelp();
      break;

    case 's':
      printSummary();
      break;

    case 'o':
      startOrientationStream(parseOptionalHz(args, kDefaultOrientationHz));
      break;

    case 'r':
      startRawStream(parseOptionalHz(args, kDefaultRawHz));
      break;

    case 'x':
      stopStreaming();
      break;

    case 'z':
      zeroHeading();
      break;

    case 'c':
      runQuickCalibration();
      break;

    case 'm':
      runMagCalibration();
      break;

    case 'b':
      beginMpu();
      break;

    default:
      Serial.printf("Unknown command: %c\n", cmd);
      printHelp();
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      line_buffer[line_length] = '\0';
      handleCommand(line_buffer);
      line_length = 0;
      continue;
    }

    if (line_length + 1 < sizeof(line_buffer)) {
      line_buffer[line_length++] = c;
    }
  }
}

void printHealthLine(uint32_t now_ms) {
  if (now_ms - last_health_print_ms < 1000) {
    return;
  }

  const uint32_t elapsed_ms = now_ms - health_window_start_ms;
  float rate_hz = 0.0f;
  if (elapsed_ms > 0) {
    rate_hz = (1000.0f * static_cast<float>(health_window_updates)) / static_cast<float>(elapsed_ms);
  }

  const uint32_t stale_ms = now_ms - last_mpu_update_ms;

  Serial.printf("[health] rate=%.1fHz stale=%lums win_ok=%lu win_fail=%lu\n",
    rate_hz,
    static_cast<unsigned long>(stale_ms),
    static_cast<unsigned long>(health_window_updates),
    static_cast<unsigned long>(health_window_failures));

  if (stale_ms > kStaleWarnMs) {
    Serial.printf("[warn] MPU updates stale (%lums). Check wiring/noise/power.\n", static_cast<unsigned long>(stale_ms));
  }

  if (rate_hz < kLowRateWarnHz) {
    Serial.printf("[warn] MPU update rate low (%.1fHz).\n", rate_hz);
  }

  health_window_start_ms = now_ms;
  health_window_updates = 0;
  health_window_failures = 0;
  last_health_print_ms = now_ms;
}

void printOrientationLine(uint32_t now_ms) {
  if (stream_mode != StreamMode::Orientation || stream_hz == 0) {
    return;
  }

  const uint32_t interval_ms = 1000UL / stream_hz;
  if (now_ms - last_stream_print_ms < interval_ms) {
    return;
  }

  last_stream_print_ms = now_ms;
  Serial.printf("t=%lums yaw=%.2f yaw_rel=%.2f pitch=%.2f roll=%.2f temp=%.2fC\n",
    static_cast<unsigned long>(now_ms),
    mpu.getYaw(),
    mpu.getYaw() - yaw_zero_deg,
    mpu.getPitch(),
    mpu.getRoll(),
    mpu.getTemperature());
}

void printRawLine(uint32_t now_ms) {
  if (stream_mode != StreamMode::Raw || stream_hz == 0) {
    return;
  }

  const uint32_t interval_ms = 1000UL / stream_hz;
  if (now_ms - last_stream_print_ms < interval_ms) {
    return;
  }

  last_stream_print_ms = now_ms;
  Serial.printf(
    "t=%lums acc[g]=%.3f,%.3f,%.3f gyro[dps]=%.3f,%.3f,%.3f mag[mG]=%.3f,%.3f,%.3f temp=%.2fC\n",
    static_cast<unsigned long>(now_ms),
    mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
    mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
    mpu.getMagX(), mpu.getMagY(), mpu.getMagZ(),
    mpu.getTemperature());
}

}  // namespace

void setup() {
  setLedMode(LedMode::Error);

  Serial.begin(115200);
  const uint32_t wait_start = millis();
  while (!Serial && (millis() - wait_start) < 2000) {
    delay(10);
  }

  Serial.println();
  Serial.println("mpuTest interactive harness");
  Serial.printf("I2C pins: SDA=%u SCL=%u, AD0=GND => addr 0x%02X, FSYNC=GND\n", kI2cSdaPin, kI2cSclPin, kMpuAddress);

  Wire.begin(kI2cSdaPin, kI2cSclPin);
  Wire.setClock(kI2cClockHz);

  printI2cScan();
  beginMpu();

  printHelp();
}

void loop() {
  pollSerial();

  const uint32_t now_ms = millis();

  if (mpu_ready) {
    if (mpu.update()) {
      ++total_update_ok;
      ++health_window_updates;
      last_mpu_update_ms = now_ms;
    } else {
      ++total_update_fail;
      ++health_window_failures;
    }

    printHealthLine(now_ms);
    printOrientationLine(now_ms);
    printRawLine(now_ms);
  }
}
