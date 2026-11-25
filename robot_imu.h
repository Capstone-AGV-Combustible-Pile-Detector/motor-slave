#ifndef __ROBOT_IMU_H__
#define __ROBOT_IMU_H__

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDR_PRIMARY   0x68
#define MPU6050_ADDR_SECONDARY 0x69

// MPU6050 Registers
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75

extern bool imu_connected;
extern bool imu_is_stationary;
extern float imu_yaw_deg;         // Accumulated continuous yaw (-180 to 180 or unrolled)
extern float imu_gyro_z_dps;      // Angular velocity around Z in deg/s
extern float imu_accel_x_g;       // Linear acceleration X in g
extern float imu_accel_y_g;       // Linear acceleration Y in g

bool robot_imu_init(void);
void robot_imu_update(void);
void robot_imu_reset_yaw(float new_yaw_deg = 0.0f);
void robot_imu_set_stationary(bool stationary);

#endif
