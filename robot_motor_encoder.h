#ifndef __ROBOT_MOTOR_ENCODER_H__
#define __ROBOT_MOTOR_ENCODER_H__

#include <Arduino.h>
#include <stdint.h>

#define NUM_OF_MOTORS         4
#define NUM_OF_ENCODER        4
#define ENCODER_PPR           11
#define ENCODER_QUADRATURE    2
#define GEARBOX_RATIO         46.0f
#define WHEEL_PPR             (ENCODER_PPR * ENCODER_QUADRATURE * GEARBOX_RATIO)  // 1012.0 ticks/rev
#define WHEEL_DIAMETER        0.06f  // 60mm wheel diameter
#define WHEEL_RADIUS          (WHEEL_DIAMETER / 2.0f)
#define WHEEL_CIRCUMFERENCE   (PI * WHEEL_DIAMETER)

// Odometry & encoder filter thresholds
#define ENCODER_DEADBAND_TICKS   1    // Filter out 1-tick jitter when stationary
#define ENCODER_MAX_DELTA_TICKS  120  // Reject impossible EMI noise spikes (>700 RPM in 10ms)

#define CW    0
#define CCW   1

extern volatile int32_t encoder_cnt[NUM_OF_MOTORS];
extern volatile int32_t encoder_prev_cnt[NUM_OF_MOTORS];
extern volatile int32_t encoder_last_cnt[NUM_OF_MOTORS];
extern volatile int32_t encoder_RPM[NUM_OF_MOTORS];
extern volatile int32_t gearbox_RPM[NUM_OF_MOTORS];

extern int32_t wheel_delta_ticks[NUM_OF_MOTORS];
extern float wheel_speed_filtered_ms[NUM_OF_MOTORS];
extern bool is_motors_commanded_stop;

extern float setPoint_velocity[NUM_OF_MOTORS];
extern float setPoint_RPM[NUM_OF_MOTORS];
extern float error_RPM[NUM_OF_MOTORS];
extern float lastError[NUM_OF_MOTORS];
extern float wheel_speed_ms[NUM_OF_MOTORS];
extern float wheel_position_rad[NUM_OF_MOTORS];
extern float wheel_velocity_rad[NUM_OF_MOTORS];

extern float P_RPM[NUM_OF_MOTORS];
extern float I_RPM[NUM_OF_MOTORS];
extern float D_RPM[NUM_OF_MOTORS];
extern float PID_RPM[NUM_OF_MOTORS];

extern float motor_pid_min;
extern float motor_pid_max;

extern int PIN_MOT_1A;
extern int PIN_MOT_1B;
extern int PIN_ENC_1A;
extern int PIN_ENC_1B;
extern volatile int VAL_ENC_1A;
extern volatile int VAL_ENC_1B;

extern int PIN_MOT_2A;
extern int PIN_MOT_2B;
extern int PIN_ENC_2A;
extern int PIN_ENC_2B;
extern volatile int VAL_ENC_2A;
extern volatile int VAL_ENC_2B;

extern int PIN_MOT_3A;
extern int PIN_MOT_3B;
extern int PIN_ENC_3A;
extern int PIN_ENC_3B;
extern volatile int VAL_ENC_3A;
extern volatile int VAL_ENC_3B;

extern int PIN_MOT_4A;
extern int PIN_MOT_4B;
extern int PIN_ENC_4A;
extern int PIN_ENC_4B;
extern volatile int VAL_ENC_4A;
extern volatile int VAL_ENC_4B;

extern int frequency;
extern int resolution;
extern int pwmChannel1;
extern int pwmChannel2;
extern int pwmChannel3;
extern int pwmChannel4;

extern bool enable_motor_control;

void robot_motor_encoder_init(void);

void encoder1_ISR(void);
void encoder2_ISR(void);
void encoder3_ISR(void);
void encoder4_ISR(void);

void encoder1_RPM(void);
void encoder2_RPM(void);
void encoder3_RPM(void);
void encoder4_RPM(void);
void encoder_calculation(void);

void motor1(int speed);
void motor2(int speed);
void motor3(int speed);
void motor4(int speed);

void drive_motor(int PWM_Motor1, int PWM_Motor2, int PWM_Motor3, int PWM_Motor4);
void config_PID_min_max(float minimumOut, float maximumOut);
void config_PID_motor(float KP, float KI, float KD);
void config_RPM_min_max(float minimumRPM, float maximumRPM);
void setMotorRPM(int RPM_Motor1, int RPM_Motor2, int RPM_Motor3, int RPM_Motor4);
void setMotorPWM(int PWM_Motor1, int PWM_Motor2, int PWM_Motor3, int PWM_Motor4);
void global_motor_control(void);

#endif
