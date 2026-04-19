#include "imu_service.h"

namespace {

constexpr float kGravityMps2 = 9.80665f;

MPU9250Setting buildImuSetting() {
  MPU9250Setting setting;
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

}  // namespace

bool ImuService::begin(const RobotConfig& config) {
  cfg_ = config.imu;

  Wire.begin();

  MPU9250Setting setting = buildImuSetting();
  if (!mpu_.setup(cfg_.i2c_address, setting)) {
    return false;
  }

  mpu_.setMagneticDeclination(cfg_.magnetic_declination_deg);
  mpu_.selectFilter(QuatFilterSel::MADGWICK);
  mpu_.setFilterIterations(cfg_.filter_iterations);

  if (cfg_.run_accel_gyro_calibration) {
    mpu_.calibrateAccelGyro();
  }

  // Fast bias estimate: robot is expected to be stationary during boot.
  uint16_t samples = 0;
  float gyro_sum = 0.0f;
  float accel_x_sum = 0.0f;
  while (samples < cfg_.bias_samples) {
    if (mpu_.update()) {
      gyro_sum += mpu_.getGyroZ();
      accel_x_sum += mpu_.getLinearAccX() * kGravityMps2;
      ++samples;
    }
    delay(cfg_.bias_sample_interval_ms);
  }

  if (samples > 0) {
    gyro_z_bias_deg_s_ = gyro_sum / static_cast<float>(samples);
    linear_accel_x_bias_m_s2_ = accel_x_sum / static_cast<float>(samples);
  }

  have_last_yaw_ = false;
  sample_ = {};

  return true;
}

bool ImuService::update(uint32_t now_ms) {
  if (!mpu_.update()) {
    return false;
  }

  const float yaw_deg = mpu_.getYaw();
  if (!have_last_yaw_) {
    last_yaw_deg_ = yaw_deg;
    yaw_unwrapped_deg_ = yaw_deg;
    have_last_yaw_ = true;
  } else {
    const float delta = wrapDeltaDeg(yaw_deg - last_yaw_deg_);
    yaw_unwrapped_deg_ += delta;
    last_yaw_deg_ = yaw_deg;
  }

  const float gyro_z_dps = mpu_.getGyroZ() - gyro_z_bias_deg_s_;
  const float accel_forward_m_s2 = (mpu_.getLinearAccX() * kGravityMps2) - linear_accel_x_bias_m_s2_;

  sample_.valid = true;
  sample_.yaw_deg = yaw_deg;
  sample_.yaw_unwrapped_rad = yaw_unwrapped_deg_ * DEG_TO_RAD;
  sample_.yaw_rate_rad_s = gyro_z_dps * DEG_TO_RAD;
  sample_.forward_accel_m_s2 = accel_forward_m_s2;
  sample_.updated_ms = now_ms;
  return true;
}

bool ImuService::isHealthy(uint32_t now_ms, uint32_t stale_timeout_ms) const {
  if (!sample_.valid) {
    return false;
  }
  return (now_ms - sample_.updated_ms) <= stale_timeout_ms;
}

float ImuService::wrapDeltaDeg(float delta_deg) {
  while (delta_deg > 180.0f) {
    delta_deg -= 360.0f;
  }
  while (delta_deg < -180.0f) {
    delta_deg += 360.0f;
  }
  return delta_deg;
}
