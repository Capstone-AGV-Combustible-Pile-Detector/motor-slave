#include <Arduino.h>
#include "robot_motor_encoder.h"

volatile int32_t encoder_cnt[NUM_OF_MOTORS];
volatile int32_t encoder_prev_cnt[NUM_OF_MOTORS];
volatile int32_t encoder_last_cnt[NUM_OF_MOTORS];
volatile int32_t encoder_RPM[NUM_OF_MOTORS];
volatile int32_t gearbox_RPM[NUM_OF_MOTORS];

float setPoint_velocity[NUM_OF_MOTORS];
float setPoint_RPM[NUM_OF_MOTORS];
float error_RPM[NUM_OF_MOTORS];
float lastError[NUM_OF_MOTORS];
float wheel_speed_ms[NUM_OF_MOTORS];
float wheel_position_rad[NUM_OF_MOTORS];
float wheel_velocity_rad[NUM_OF_MOTORS];

int32_t wheel_delta_ticks[NUM_OF_MOTORS] = {0, 0, 0, 0};
float wheel_speed_filtered_ms[NUM_OF_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f};
bool is_motors_commanded_stop = true;

float P_RPM[NUM_OF_MOTORS];
float I_RPM[NUM_OF_MOTORS];
float D_RPM[NUM_OF_MOTORS];
float PID_RPM[NUM_OF_MOTORS];

float min_rpm = 0;
float max_rpm = 0;

float motor_p = 0;
float motor_i = 0;
float motor_d = 0;

float motor_pid_min;
float motor_pid_max;

unsigned long currentTime = 0;
unsigned long previousTime = 0;
int interval = 10;
long delta_counter;

int PIN_MOT_1A = 23;
int PIN_MOT_1B = 19;
int PIN_ENC_1A = 36;
int PIN_ENC_1B = 39;
volatile int VAL_ENC_1A = 0;
volatile int VAL_ENC_1B = 0;

int PIN_MOT_2A = 18;
int PIN_MOT_2B = 4;
int PIN_ENC_2A = 34;
int PIN_ENC_2B = 35;
volatile int VAL_ENC_2A = 0;
volatile int VAL_ENC_2B = 0;

int PIN_MOT_3A = 2;
int PIN_MOT_3B = 13;
int PIN_ENC_3A = 32;
int PIN_ENC_3B = 33;
volatile int VAL_ENC_3A = 0;
volatile int VAL_ENC_3B = 0;

int PIN_MOT_4A = 14;
int PIN_MOT_4B = 27;
int PIN_ENC_4A = 25;
int PIN_ENC_4B = 26;
volatile int VAL_ENC_4A = 0;
volatile int VAL_ENC_4B = 0;

int frequency   = 5000;
int resolution  = 8;
int pwmChannel1 = 0;
int pwmChannel2 = 1;
int pwmChannel3 = 2;
int pwmChannel4 = 3;

bool enable_motor_control = true;

void robot_motor_encoder_init(){
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_MOT_1A, frequency, resolution); pinMode(PIN_MOT_1B, OUTPUT);
  pinMode(PIN_ENC_1A, INPUT_PULLUP);      pinMode(PIN_ENC_1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1A), encoder1_ISR, CHANGE);

  ledcAttach(PIN_MOT_2A, frequency, resolution); pinMode(PIN_MOT_2B, OUTPUT);
  pinMode(PIN_ENC_2A, INPUT_PULLUP);      pinMode(PIN_ENC_2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2A), encoder2_ISR, CHANGE);

  ledcAttach(PIN_MOT_3A, frequency, resolution); pinMode(PIN_MOT_3B, OUTPUT);
  pinMode(PIN_ENC_3A, INPUT_PULLUP);      pinMode(PIN_ENC_3B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_3A), encoder3_ISR, CHANGE);

  ledcAttach(PIN_MOT_4A, frequency, resolution); pinMode(PIN_MOT_4B, OUTPUT);
  pinMode(PIN_ENC_4A, INPUT_PULLUP);      pinMode(PIN_ENC_4B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_4A), encoder4_ISR, CHANGE);
#else
  ledcSetup(pwmChannel1, frequency, resolution);
  ledcAttachPin(PIN_MOT_1A, pwmChannel1); pinMode(PIN_MOT_1B, OUTPUT);
  pinMode(PIN_ENC_1A, INPUT_PULLUP);      pinMode(PIN_ENC_1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1A), encoder1_ISR, CHANGE);

  ledcSetup(pwmChannel2, frequency, resolution);
  ledcAttachPin(PIN_MOT_2A, pwmChannel2); pinMode(PIN_MOT_2B, OUTPUT);
  pinMode(PIN_ENC_2A, INPUT_PULLUP);      pinMode(PIN_ENC_2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2A), encoder2_ISR, CHANGE);

  ledcSetup(pwmChannel3, frequency, resolution);
  ledcAttachPin(PIN_MOT_3A, pwmChannel3); pinMode(PIN_MOT_3B, OUTPUT);
  pinMode(PIN_ENC_3A, INPUT_PULLUP);      pinMode(PIN_ENC_3B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_3A), encoder3_ISR, CHANGE);

  ledcSetup(pwmChannel4, frequency, resolution);
  ledcAttachPin(PIN_MOT_4A, pwmChannel4); pinMode(PIN_MOT_4B, OUTPUT);
  pinMode(PIN_ENC_4A, INPUT_PULLUP);      pinMode(PIN_ENC_4B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_4A), encoder4_ISR, CHANGE);
#endif
}

void IRAM_ATTR encoder1_ISR(){
  VAL_ENC_1A = digitalRead(PIN_ENC_1A); VAL_ENC_1B = digitalRead(PIN_ENC_1B);
  if ((VAL_ENC_1A == HIGH) != (VAL_ENC_1B == LOW)){encoder_cnt[1]++;}
  else{encoder_cnt[1]--;}
}

void IRAM_ATTR encoder2_ISR(){
  VAL_ENC_2A = digitalRead(PIN_ENC_2A); VAL_ENC_2B = digitalRead(PIN_ENC_2B);
  if ((VAL_ENC_2A == HIGH) != (VAL_ENC_2B == LOW)){encoder_cnt[2]++;}
  else{encoder_cnt[2]--;}
}

void IRAM_ATTR encoder3_ISR(){
  VAL_ENC_3A = digitalRead(PIN_ENC_3A); VAL_ENC_3B = digitalRead(PIN_ENC_3B);
  if ((VAL_ENC_3A == HIGH) != (VAL_ENC_3B == LOW)){encoder_cnt[3]++;}
  else{encoder_cnt[3]--;}
}

void IRAM_ATTR encoder4_ISR(){
  VAL_ENC_4A = digitalRead(PIN_ENC_4A); VAL_ENC_4B = digitalRead(PIN_ENC_4B);
  if ((VAL_ENC_4A == HIGH) != (VAL_ENC_4B == LOW)){encoder_cnt[0]++;}
  else{encoder_cnt[0]--;}
}

void encoder1_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_RPM[0] = ((encoder_cnt[0] - encoder_prev_cnt[0]) * 60) / ENCODER_PPR * ENCODER_QUADRATURE; // RPM on motor shaft
    encoder_prev_cnt[0] = encoder_cnt[0];
  }
}

void encoder2_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_RPM[1] = ((encoder_cnt[1] - encoder_prev_cnt[1]) * 60) / ENCODER_PPR * ENCODER_QUADRATURE; // RPM on motor shaft
    encoder_prev_cnt[1] = encoder_cnt[1];
  }
}

void encoder3_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_RPM[2] = ((encoder_cnt[2] - encoder_prev_cnt[2]) * 60) / ENCODER_PPR * ENCODER_QUADRATURE; // RPM on motor shaft
    encoder_prev_cnt[2] = encoder_cnt[2];
  }
}

void encoder4_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_RPM[3] = ((encoder_cnt[3] - encoder_prev_cnt[3]) * 60) / ENCODER_PPR * ENCODER_QUADRATURE; // RPM on motor shaft
    encoder_prev_cnt[3] = encoder_cnt[3];
  }
}

void encoder_calculation(){
  currentTime = millis();

  if (currentTime - previousTime >= interval){
    float dt = (currentTime - previousTime) / 1000.0f;
    previousTime = currentTime;
    if (dt <= 0.0001f) dt = 0.010f;

    // Check if robot is commanded to be at rest
    is_motors_commanded_stop = (setPoint_RPM[0] == 0 && setPoint_RPM[1] == 0 &&
                                setPoint_RPM[2] == 0 && setPoint_RPM[3] == 0);

    for(int i = 0; i < NUM_OF_MOTORS; i++){
      // Atomically sample encoder count to avoid race conditions or dropped ticks
      int32_t current_cnt = encoder_cnt[i];
      int32_t delta = current_cnt - encoder_prev_cnt[i];
      encoder_prev_cnt[i] = current_cnt;

      // 1. Noise glitch rejection: reject impossible sudden tick jumps (EMI spikes from motor brushes)
      if (abs(delta) > ENCODER_MAX_DELTA_TICKS) {
        delta = 0;
      }

      // 2. Deadband filter: Suppress 1-tick jitter / vibration when commanded to stop
      if (is_motors_commanded_stop && abs(delta) <= ENCODER_DEADBAND_TICKS) {
        delta = 0;
      }

      wheel_delta_ticks[i] = delta;

      encoder_RPM[i] = (delta * 60) / ((ENCODER_PPR * ENCODER_QUADRATURE));
      gearbox_RPM[i] = (delta * 60) / ((ENCODER_PPR * ENCODER_QUADRATURE * GEARBOX_RATIO));

      float raw_wheel_speed = (PI * WHEEL_DIAMETER * gearbox_RPM[i]) / 60.0f;
      wheel_speed_ms[i] = raw_wheel_speed;

      // 3. Low-Pass Exponential Moving Average (EMA) filter on wheel linear speed
      if (is_motors_commanded_stop && delta == 0) {
        // Drop immediately to zero without dragging long filter decay tails when stopped
        wheel_speed_filtered_ms[i] = 0.0f;
      } else {
        wheel_speed_filtered_ms[i] = (0.35f * raw_wheel_speed) + (0.65f * wheel_speed_filtered_ms[i]);
      }

      float delta_theta = delta * (2.0f * PI) / (ENCODER_PPR * ENCODER_QUADRATURE * GEARBOX_RATIO);

      wheel_position_rad[i] += delta_theta;
      wheel_velocity_rad[i] = delta_theta / dt;
    }
  }
}

void motor1(int PWM){
  int dir;  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_MOT_1A, PWM);
#else
  ledcWrite(pwmChannel1, PWM);
#endif
  digitalWrite(PIN_MOT_1B, dir);
}

void motor2(int PWM){
  int dir;  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_MOT_2A, PWM);
#else
  ledcWrite(pwmChannel2, PWM);
#endif
  digitalWrite(PIN_MOT_2B, dir);
}

void motor3(int PWM){
  int dir;  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_MOT_3A, PWM);
#else
  ledcWrite(pwmChannel3, PWM);
#endif
  digitalWrite(PIN_MOT_3B, dir);
}

void motor4(int PWM){
  int dir;  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_MOT_4A, PWM);
#else
  ledcWrite(pwmChannel4, PWM);
#endif
  digitalWrite(PIN_MOT_4B, dir);
}

void drive_motor(int speedVal1, int speedVal2, int speedVal3, int speedVal4){
  motor1(speedVal1);  motor2(speedVal2);  motor3(speedVal3);  motor4(speedVal4);
}

void config_PID_min_max(float min, float max){motor_pid_min = min; motor_pid_max = max;}

void config_PID_motor(float kp, float ki, float kd){motor_p = kp; motor_i = ki; motor_d = kd;}

void config_RPM_min_max(float min, float max){min_rpm = min; max_rpm = max;}

void setMotorRPM(int speed1, int speed2, int speed3, int speed4){
  setPoint_RPM[0] = speed1; setPoint_RPM[1] = speed2; setPoint_RPM[2] = speed3; setPoint_RPM[3] = speed4;
}

void setMotorPWM(int speed1, int speed2, int speed3, int speed4){
  motor1(speed1); motor2(speed2); motor3(speed3); motor4(speed4);
}

void global_motor_control(){
  if (!enable_motor_control) return;

  // When robot is commanded to stop and wheels are essentially at rest,
  // cut power to motors to prevent coil whine, micro-jitter, and encoder creep
  if (is_motors_commanded_stop &&
      abs(encoder_RPM[0]) < 2 && abs(encoder_RPM[1]) < 2 &&
      abs(encoder_RPM[2]) < 2 && abs(encoder_RPM[3]) < 2) {
    for (int i = 0; i < NUM_OF_MOTORS; i++) {
      I_RPM[i] = 0.0f;
      PID_RPM[i] = 0.0f;
      lastError[i] = 0.0f;
    }
    drive_motor(0, 0, 0, 0);
    return;
  }

  for (int i = 0; i < NUM_OF_MOTORS; i++){
    error_RPM[i] = (setPoint_RPM[i] - encoder_RPM[i]);
    P_RPM[i] = (float)(error_RPM[i] * motor_p);
    I_RPM[i] += error_RPM[i] * motor_i;
    D_RPM[i] = (error_RPM[i] - lastError[i]) * motor_d;
    PID_RPM[i] = P_RPM[i] + I_RPM[i] + D_RPM[i];

    if (I_RPM[i] > motor_pid_max) I_RPM[i] = motor_pid_max;
    if (I_RPM[i] < motor_pid_min) I_RPM[i] = motor_pid_min;

    if (PID_RPM[i] > motor_pid_max) PID_RPM[i] = motor_pid_max;
    if (PID_RPM[i] < motor_pid_min) PID_RPM[i] = motor_pid_min;

    if (error_RPM[i] == 0) I_RPM[i] = 0;

    lastError[i] = error_RPM[i];
  }

  drive_motor(PID_RPM[1], PID_RPM[2], PID_RPM[3], PID_RPM[0]);
}
