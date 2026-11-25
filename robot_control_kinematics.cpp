#include <Arduino.h>
#include "robot_control_kinematics.h"
#include "robot_motor_encoder.h"
#include "robot_imu.h"
#include "robot_serial.h"

// PID variables for odometry
float P_odom = 0.0;
float I_odom = 0.0;
float D_odom = 0.0;
float KP_odom = 0.0;
float KI_odom = 0.0;
float KD_odom = 0.0;
float PID_odom = 0.0;
float errorDistance = 0.0;
float lastErrorDistance = 0.0;

// PID variables for heading lock
float P_yaw = 0.0;
float I_yaw = 0.0;
float D_yaw = 0.0;
float KP_yaw = 0.0;
float KI_yaw = 0.0;
float KD_yaw = 0.0;
float PID_yaw = 0.0;
float errorYaw = 0.0;
float setPointYaw = 0.0;
float lastErrorYaw = 0.0;
float yaw_pid_min = -255.0;
float yaw_pid_max = 255.0;

// PID variables for x-axis movement
float P_xAxis = 0.0;
float I_xAxis = 0.0;
float D_xAxis = 0.0;
float KP_xAxis = 0.0;
float KI_xAxis = 0.0;
float KD_xAxis = 0.0;
float PID_xAxis = 0.0;
float errorPosXAxis = 0.0;
float lastErrorPosXAxis = 0.0;

// PID variables for y-axis movement
float P_yAxis = 0.0;
float I_yAxis = 0.0;
float D_yAxis = 0.0;
float KP_yAxis = 0.0;
float KI_yAxis = 0.0;
float KD_yAxis = 0.0;
float PID_yAxis = 0.0;
float errorPosYAxis = 0.0;
float lastErrorPosYAxis = 0.0;

// PID variables for z-axis movement
float P_theta = 0.0;
float I_theta = 0.0;
float D_theta = 0.0;
float KP_theta = 0.0;
float KI_theta = 0.0;
float KD_theta = 0.0;
float PID_theta = 0.0;
float errorPosTheta = 0.0;
float lastErrorPosTheta = 0.0;

// Odometry variables
bool statusWaypointReached = false;

float RobotSetPointX = 0.0;
float RobotSetPointY = 0.0;
float RobotSetPointTheta = 0.0;

float RobotActualPositionX = 0.0;
float RobotActualPositionY = 0.0;
float RobotActualPositionTheta = 0.0;

float maxSpeedRobotLin = 0.0;
float maxSpeedRobotAng = 0.0;

float DistanceRobotToTarget = 0.0;
float TotalDistanceTravelledByRobot = 0.0;

float VelocityRobotX = 0.0;
float VelocityRobotY = 0.0;
float VelocityRobotZ = 0.0;

float PID_velocity = 0.0;

float ErrorPositionX = 0.0;
float ErrorPositionY = 0.0;
float ErrorPositionTheta = 0.0;

float v_robot_l_x = 0.0;
float v_robot_l_y = 0.0;
float v_robot_w = 0.0;
bool robot_is_stationary = true;

unsigned long currentTime_odom = 0;
unsigned long previousTime_odom = 0;

float velocity_wheel[4];
float velocity_robot[2];
float theta_dot[4];

float yaw_robot = 0.0;
float yaw_polar = 0.0;

int deltaTime_odom = 0;
int delta_encoder[4];

bool enable_position_control = false;

// Matrix Odometry D_WHEEL 0.06 OFFSET HEADING -45
float m_odometry[2][4] = {
    {0.010607f, -0.010607f, -0.010607f, 0.010607f},
    {0.010607f, 0.010607f, -0.010607f, -0.010607f}
};

void config_PID_gain_yaw(float kp, float ki, float kd){
  KP_yaw = kp;
  KI_yaw = ki;
  KD_yaw = kd;
}

void config_PID_gain_odom_x(float kp, float ki, float kd){
  KP_xAxis = kp;
  KI_xAxis = ki;
  KD_xAxis = kd;
}

void config_PID_gain_odom_y(float kp, float ki, float kd){
  KP_yAxis = kp;
  KI_yAxis = ki;
  KD_yAxis = kd;
}

void config_PID_gain_odom_z(float kp, float ki, float kd){
  KP_theta = kp;
  KI_theta = ki;
  KD_theta = kd;
}

void config_PID_gain_odom_robot(float kp, float ki, float kd){
  KP_odom = kp;
  KI_odom = ki;
  KD_odom = kd;
}

void drive_robot(float speed_global_x,
                float speed_global_y,
                float speed_angular_w,
                float offset_heading,
                int number_of_wheels,
                float wheelRadius,
                float robotRadius,
                float *motorOut){

  float del_angle = 360.0f / number_of_wheels;

  for(int i = 0; i < number_of_wheels; i++){
    float angleDeg = del_angle * i + offset_heading;
    float angleRad = angleDeg * PI / 180.0f;

    motorOut[i] = (-sinf(angleRad) * speed_global_x) / wheelRadius;
    motorOut[i] += (cosf(angleRad) * speed_global_y) / wheelRadius;
    motorOut[i] += (robotRadius * speed_angular_w) / wheelRadius;
  }
}

void drive_robot_controller(float speed_global_x,
                            float speed_global_y,
                            float speed_angular_w,
                            float offset_heading,
                            int number_of_wheels,
                            float wheelRadius,
                            float robotRadius){

  float del_angle = 360.0f / number_of_wheels;
  float motorOut[number_of_wheels];

  for(int i = 0; i < number_of_wheels; i++){
    float angleDeg = del_angle * i + offset_heading;
    float angleRad = angleDeg * PI / 180.0f;

    motorOut[i] = (-sinf(angleRad) * speed_global_x) / wheelRadius;
    motorOut[i] += (cosf(angleRad) * speed_global_y) / wheelRadius;
    motorOut[i] += (robotRadius * speed_angular_w) / wheelRadius;

    if (motorOut[i] > 255) motorOut[i] = 255;
    if (motorOut[i] < -255) motorOut[i] = -255;
  }
  drive_motor(motorOut[0], motorOut[1], motorOut[2], motorOut[3]);
}

void set_robot_position(float x, float y, float w, float maxSpeedLin, float maxSpeedAng) {
  RobotSetPointX = x;
  RobotSetPointY = y;
  RobotSetPointTheta = w;
  maxSpeedRobotLin = maxSpeedLin;
  maxSpeedRobotAng = maxSpeedAng;
}

void set_robot_speed(float x, float y, float w){
  drive_robot(x, y, w, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, setPoint_RPM);
}

void robot_odometry(float MI[2][4], float T[4], float angle, float output[2]) {
    float rad = angle * PI / 180.0f;

    float R[2][2] = {
        {cosf(rad), -sinf(rad)},
        {sinf(rad),  cosf(rad)}
    };

    float M[2][1] = {0};

    for(int i = 0; i < 2; i++) {
        M[i][0] = 0;
        for(int j = 0; j < 4; j++) {
            M[i][0] += MI[i][j] * T[j];
        }
    }

    for(int i = 0; i < 2; i++) {
        output[i] = 0;
        for(int j = 0; j < 2; j++) {
            output[i] += R[i][j] * M[j][0];
        }
    }
}

void robot_odometry_timer(){
  currentTime_odom = millis();
  deltaTime_odom = currentTime_odom - previousTime_odom;
  float dt = deltaTime_odom / 1000.0f;

  if (dt <= 0.0001f || dt > 0.5f) {
    previousTime_odom = currentTime_odom;
    return;
  }
  previousTime_odom = currentTime_odom;

  // 1. Get wheel linear displacements and velocities
  // Each wheel delta ticks was computed and filtered in encoder_calculation()
  float delta_dist_m[NUM_OF_MOTORS];
  bool all_wheels_stopped = true;

  for (int i = 0; i < NUM_OF_MOTORS; i++){
    int32_t ticks = wheel_delta_ticks[i];
    if (abs(ticks) > 0) {
      all_wheels_stopped = false;
    }
    // Linear displacement per wheel in meters
    delta_dist_m[i] = ((float)ticks / WHEEL_PPR) * (PI * D_WHEEL);
    // Linear velocity per wheel in m/s (from low-pass filtered speed)
    velocity_wheel[i] = wheel_speed_filtered_ms[i];
  }

  // 2. Stationary / Zero-Velocity Update (ZUPT) Detection
  // Check if robot velocity setpoint is zero AND wheels are stopped
  bool cmd_zero = (abs(setPoint_RPM[0]) == 0 && abs(setPoint_RPM[1]) == 0 &&
                   abs(setPoint_RPM[2]) == 0 && abs(setPoint_RPM[3]) == 0);

  static int stationary_confirm_count = 0;
  if (cmd_zero && all_wheels_stopped) {
    if (stationary_confirm_count < 10) {
      stationary_confirm_count++;
    }
  } else {
    stationary_confirm_count = 0;
  }

  bool is_stationary = (stationary_confirm_count >= 3);
  robot_is_stationary = is_stationary;
  robot_imu_set_stationary(is_stationary);

  if (is_stationary) {
    // Robot is at rest: STRICTLY clamp velocities and freeze odometry accumulation to eliminate drift!
    v_robot_l_x = 0.0f;
    v_robot_l_y = 0.0f;
    v_robot_w = 0.0f;
    return;
  }

  // 3. Forward Kinematics for 4-wheel omnidirectional robot (offset -45 deg)
  // Wheel 0 = Motor 4 (-45 deg)
  // Wheel 1 = Motor 1 (+45 deg)
  // Wheel 2 = Motor 2 (+135 deg)
  // Wheel 3 = Motor 3 (+225 deg)
  //
  // delta_x_local = (sqrt(2)/4) * (d0 - d1 - d2 + d3)
  // delta_y_local = (sqrt(2)/4) * (d0 + d1 - d2 - d3)
  float delta_x_local = INV_SQRT2_OVER_2 * (delta_dist_m[0] - delta_dist_m[1] - delta_dist_m[2] + delta_dist_m[3]);
  float delta_y_local = INV_SQRT2_OVER_2 * (delta_dist_m[0] + delta_dist_m[1] - delta_dist_m[2] - delta_dist_m[3]);

  // Compute raw local velocities in m/s
  float v_lx_raw = INV_SQRT2_OVER_2 * (velocity_wheel[0] - velocity_wheel[1] - velocity_wheel[2] + velocity_wheel[3]);
  float v_ly_raw = INV_SQRT2_OVER_2 * (velocity_wheel[0] + velocity_wheel[1] - velocity_wheel[2] - velocity_wheel[3]);

  // Smooth local velocities with low-pass filter (alpha = 0.40)
  v_robot_l_x = (0.40f * v_lx_raw) + (0.60f * v_robot_l_x);
  v_robot_l_y = (0.40f * v_ly_raw) + (0.60f * v_robot_l_y);

  // Angular velocity from IMU (or wheels if IMU is unavailable)
  float delta_theta_rad = 0.0f;
  if (imu_connected) {
    v_robot_w = imu_gyro_z_dps * (PI / 180.0f);
    delta_theta_rad = v_robot_w * dt;
  } else {
    // Fallback: wheel kinematics angular velocity
    // w = (1 / (4 * R_robot)) * (d0 + d1 + d2 + d3)
    float delta_theta_wheel = (delta_dist_m[0] + delta_dist_m[1] + delta_dist_m[2] + delta_dist_m[3]) / (4.0f * R_ROBOT);
    delta_theta_rad = delta_theta_wheel;
    v_robot_w = delta_theta_wheel / dt;
    RobotActualPositionTheta += delta_theta_wheel * (180.0f / PI);
    if (RobotActualPositionTheta > 180.0f) RobotActualPositionTheta -= 360.0f;
    else if (RobotActualPositionTheta < -180.0f) RobotActualPositionTheta += 360.0f;
  }

  // 4. Runge-Kutta 2nd-order (Midpoint) Integration for Global Pose
  // Use average heading during this time step to eliminate curvature drift
  float current_heading_rad = RobotActualPositionTheta * (PI / 180.0f);
  float mid_heading_rad = current_heading_rad - (delta_theta_rad / 2.0f);

  float cos_th = cosf(mid_heading_rad);
  float sin_th = sinf(mid_heading_rad);

  float delta_X = (delta_x_local * cos_th) - (delta_y_local * sin_th);
  float delta_Y = (delta_x_local * sin_th) + (delta_y_local * cos_th);

  RobotActualPositionX += delta_X;
  RobotActualPositionY += delta_Y;
}

void reset_robot_odometry(float x, float y, float theta_deg){
  RobotActualPositionX = x;
  RobotActualPositionY = y;
  RobotActualPositionTheta = theta_deg;
  v_robot_l_x = 0.0f;
  v_robot_l_y = 0.0f;
  v_robot_w = 0.0f;
  for(int i = 0; i < NUM_OF_MOTORS; i++){
    wheel_speed_filtered_ms[i] = 0.0f;
    wheel_delta_ticks[i] = 0;
  }
  robot_imu_reset_yaw(theta_deg);
}
