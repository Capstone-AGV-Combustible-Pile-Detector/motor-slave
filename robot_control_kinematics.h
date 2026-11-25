#ifndef __ROBOT_CONTROL_KINEMATICS_H__
#define __ROBOT_CONTROL_KINEMATICS_H__

#include <Arduino.h>

#define D_WHEEL         0.06f
#define R_WHEEL         0.03f
#define R_ROBOT         0.10f
#define OFFSET_HEADING  -45.0f
#define PI              3.14159265359f
#define ODOM_SCALE      1.0f
#define INV_SQRT2_OVER_2 0.35355339f // sqrt(2)/4 forward kinematics coefficient

// PID variables for odometry
extern float P_odom;
extern float I_odom;
extern float D_odom;

extern float KP_odom;
extern float KI_odom;
extern float KD_odom;

extern float PID_odom;

extern float errorDistance;
extern float lastErrorDistance;

// PID variables for heading lock
extern float P_yaw;
extern float I_yaw;
extern float D_yaw;

extern float KP_yaw;
extern float KI_yaw;
extern float KD_yaw;

extern float PID_yaw;

extern float errorYaw;
extern float setPointYaw;
extern float lastErrorYaw;

extern float yaw_pid_min;
extern float yaw_pid_max;

// PID variables for x-axis movement
extern float P_xAxis;
extern float I_xAxis;
extern float D_xAxis;

extern float KP_xAxis;
extern float KI_xAxis;
extern float KD_xAxis;

extern float PID_xAxis;

extern float errorPosXAxis;
extern float lastErrorPosXAxis;

// PID variables for y-axis movement
extern float P_yAxis;
extern float I_yAxis;
extern float D_yAxis;

extern float KP_yAxis;
extern float KI_yAxis;
extern float KD_yAxis;

extern float PID_yAxis;

extern float errorPosYAxis;
extern float lastErrorPosYAxis;

// PID variables for z-axis movement
extern float P_theta;
extern float I_theta;
extern float D_theta;

extern float KP_theta;
extern float KI_theta;
extern float KD_theta;

extern float PID_theta;

extern float errorPosTheta;
extern float lastErrorPosTheta;

// Odometry variables
extern bool statusWaypointReached;

extern float RobotSetPointX;
extern float RobotSetPointY;
extern float RobotSetPointTheta;

extern float RobotActualPositionX;
extern float RobotActualPositionY;
extern float RobotActualPositionTheta;

extern float maxSpeedRobotLin;
extern float maxSpeedRobotAng;

extern float DistanceRobotToTarget;
extern float TotalDistanceTravelledByRobot;

extern float VelocityRobotX;
extern float VelocityRobotY;
extern float VelocityRobotZ;

extern float PID_velocity;

extern float ErrorPositionX;
extern float ErrorPositionY;
extern float ErrorPositionTheta;

extern float v_robot_l_x;
extern float v_robot_l_y;
extern float v_robot_w;
extern bool robot_is_stationary;

extern unsigned long currentTime_odom;
extern unsigned long previousTime_odom;

extern float velocity_wheel[4];
extern float velocity_robot[2];
extern float theta_dot[4];

extern float yaw_robot;
extern float yaw_polar;

extern int deltaTime_odom;
extern int delta_encoder[4];

extern bool enable_position_control;

extern float m_odometry[2][4];

// Function prototypes
void config_PID_gain_yaw(float kp, float ki, float kd);
void config_PID_gain_odom_x(float kp, float ki, float kd);
void config_PID_gain_odom_y(float kp, float ki, float kd);
void config_PID_gain_odom_z(float kp, float ki, float kd);
void config_PID_gain_odom_robot(float kp, float ki, float kd);

void drive_robot(float speed_x, float speed_y, float speed_w, float offset_heading, int number_of_wheels, float wheel_radius, float robot_radius, float *motor_out);
void drive_robot_controller(float speed_global_x, float speed_global_y, float speed_angular_w, float offset_heading, int number_of_wheels, float wheelRadius, float robotRadius);
void robot_odometry(float MI[2][4], float T[4], float angle, float output[2]);
void set_robot_position(float x, float y, float z, float max_speed_linear, float max_speed_angular);
void set_robot_speed(float speed_x, float speed_y, float speed_w);
void robot_odometry_timer(void);
void reset_robot_odometry(float x = 0.0f, float y = 0.0f, float theta_deg = 0.0f);

#endif
