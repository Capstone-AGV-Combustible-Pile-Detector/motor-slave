#include "robot_imu.h"
#include "robot_control_kinematics.h"

static uint8_t mpu_address = MPU6050_ADDR_PRIMARY;
bool imu_connected = false;
bool imu_is_stationary = true;

float imu_yaw_deg = 0.0f;
float imu_gyro_z_dps = 0.0f;
float imu_accel_x_g = 0.0f;
float imu_accel_y_g = 0.0f;

static float gyro_z_offset = 0.0f;
static unsigned long last_imu_micros = 0;

static bool writeMPURegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(mpu_address);
  Wire.write(reg);
  Wire.write(data);
  return (Wire.endTransmission() == 0);
}

static bool readMPUBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(mpu_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(mpu_address, length);
  if (received != length) return false;

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool robot_imu_init() {
  // Initialize I2C bus at 400kHz (Fast Mode)
  Wire.begin();
  Wire.setClock(400000);

  // Test primary address 0x68
  Wire.beginTransmission(MPU6050_ADDR_PRIMARY);
  if (Wire.endTransmission() == 0) {
    mpu_address = MPU6050_ADDR_PRIMARY;
  } else {
    // Test secondary address 0x69
    Wire.beginTransmission(MPU6050_ADDR_SECONDARY);
    if (Wire.endTransmission() == 0) {
      mpu_address = MPU6050_ADDR_SECONDARY;
    } else {
      Serial.println(F("[IMU] MPU6050 not found on I2C bus!"));
      imu_connected = false;
      return false;
    }
  }

  // 1. Wake up MPU6050 (clear sleep bit in PWR_MGMT_1)
  if (!writeMPURegister(MPU6050_PWR_MGMT_1, 0x00)) return false;
  delay(10);

  // 2. Set sample rate divider = 4 (sample rate = 1kHz / (1+4) = 200Hz)
  writeMPURegister(MPU6050_SMPLRT_DIV, 0x04);

  // 3. Set DLPF (Digital Low Pass Filter) to ~42Hz bandwidth for vibration rejection
  writeMPURegister(MPU6050_CONFIG, 0x03);

  // 4. Set Gyroscope Full Scale to ±500 deg/s (sensitivity: 65.5 LSB/(deg/s))
  writeMPURegister(MPU6050_GYRO_CONFIG, 0x08);

  // 5. Set Accelerometer Full Scale to ±4g (sensitivity: 8192 LSB/g)
  writeMPURegister(MPU6050_ACCEL_CONFIG, 0x08);

  delay(50);

  // 6. Calibrate Gyro Z (Keep robot stationary for ~1 second)
  Serial.println(F("[IMU] Calibrating MPU6050 gyro... Keep robot still!"));
  long sum_gz = 0;
  const int samples = 400;
  for (int i = 0; i < samples; i++) {
    uint8_t raw[2];
    if (readMPUBytes(MPU6050_GYRO_XOUT_H + 4, raw, 2)) {
      int16_t raw_gz = (int16_t)((raw[0] << 8) | raw[1]);
      sum_gz += raw_gz;
    }
    delay(2);
  }
  gyro_z_offset = (float)sum_gz / (float)samples;

  Serial.print(F("[IMU] MPU6050 initialized at 0x"));
  Serial.print(mpu_address, HEX);
  Serial.print(F(", GyroZ Offset: "));
  Serial.println(gyro_z_offset);

  imu_connected = true;
  last_imu_micros = micros();
  return true;
}

void robot_imu_update() {
  if (!imu_connected) return;

  unsigned long current_micros = micros();
  float dt = (current_micros - last_imu_micros) / 1000000.0f;
  last_imu_micros = current_micros;

  // Protect against micros overflow or unreasonable delta
  if (dt <= 0.0001f || dt > 0.2f) return;

  // Read Accel X, Y, Z (6 bytes) + Temp (2 bytes) + Gyro X, Y, Z (6 bytes) = 14 bytes
  uint8_t raw[14];
  if (!readMPUBytes(MPU6050_ACCEL_XOUT_H, raw, 14)) {
    return;
  }

  int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);

  // Accelerometer in g (scale: 8192 LSB/g for ±4g)
  imu_accel_x_g = (float)raw_ax / 8192.0f;
  imu_accel_y_g = (float)raw_ay / 8192.0f;

  if (imu_is_stationary) {
    // 1. Robot is stationary: continuously track and eliminate thermal zero-bias drift!
    // A slow IIR filter continually adapts gyro_z_offset to match resting sensor output
    gyro_z_offset = (0.995f * gyro_z_offset) + (0.005f * (float)raw_gz);
    imu_gyro_z_dps = 0.0f;
    // FREEZE YAW INTEGRATION: Prevents any yaw drift while robot is sitting still!
    RobotActualPositionTheta = imu_yaw_deg;
    return;
  }

  // 2. Robot is in motion: compute gyro rate with zero-calibrated offset
  float raw_gz_corrected = (float)raw_gz - gyro_z_offset;

  // Deadband filter for gyro noise (~0.18 deg/s)
  if (fabs(raw_gz_corrected) < 12.0f) { // ~0.18 dps * 65.5
    raw_gz_corrected = 0.0f;
  }

  imu_gyro_z_dps = raw_gz_corrected / 65.5f;

  // Integrate Gyro Z to track yaw (heading in degrees)
  imu_yaw_deg += imu_gyro_z_dps * dt;

  // Wrap yaw to [-180.0, 180.0] degrees
  if (imu_yaw_deg > 180.0f) {
    imu_yaw_deg -= 360.0f;
  } else if (imu_yaw_deg < -180.0f) {
    imu_yaw_deg += 360.0f;
  }

  // Update robot kinematics heading
  RobotActualPositionTheta = imu_yaw_deg;
}

void robot_imu_set_stationary(bool stationary) {
  imu_is_stationary = stationary;
}

void robot_imu_reset_yaw(float new_yaw_deg) {
  imu_yaw_deg = new_yaw_deg;
  RobotActualPositionTheta = new_yaw_deg;
}
