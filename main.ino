#include "robot_serial.h"
#include "robot_gpio.h"
#include "robot_motor_encoder.h"
#include "robot_imu.h"
#include "robot_control_kinematics.h"
#include "robot_rtos_handler.h"

// Hardware emergency pause toggle
bool isEmergencyHalted = false;

void setup() {
  // 1. Initialize Serial communication (both USB Serial and UART2)
  robot_serial_init();

  // 2. Initialize IMU (MPU6050 on I2C bus)
  robot_imu_init();

  // 3. Initialize GPIO (Buttons)
  robot_gpio_init();

  // 4. Initialize Motor PWM and Encoders
  robot_motor_encoder_init();

  // 5. Motor and Kinematics Configuration
  config_PID_min_max(-255, 255);
  config_PID_gain_yaw(10, 0, 5);

  // Set motor PID gains
  config_PID_motor(0.1, 0.25, 0);
  config_RPM_min_max(-200, 200);

  // CRITICAL: Ensure initial setpoints are strictly 0 to avoid motor runaway on power-on
  setMotorRPM(0, 0, 0, 0);
  drive_motor(0, 0, 0, 0);

  // Configure Odometry PID gains
  enable_position_control = false;
  config_PID_gain_odom_robot(10, 0, 1.5);

  // Reset all encoder counts and odometry state
  for (int i = 0; i < NUM_OF_MOTORS; i++) {
    encoder_cnt[i] = 0;
    encoder_last_cnt[i] = 0;
    encoder_prev_cnt[i] = 0;
    wheel_delta_ticks[i] = 0;
    wheel_speed_filtered_ms[i] = 0.0f;
  }
  reset_robot_odometry(0.0f, 0.0f, 0.0f);

  // 6. Direct ROS2 Mode Configuration
  // Enable motor PID closed-loop control for ROS2 navigation commands
  enable_motor_control = true;

  // 7. Create RTOS tasks (pinned to Core 0 for Serial and Core 1 for Odometry & PID)
  rtosCreateTaskSerial();
  rtosCreateTaskOdometry();

  // 8. Safely resume RTOS tasks now that all hardware and variables are initialized
  rtosResumeAllTasks();
}

void loop() {
  // Check physical buttons for Emergency Stop / Resume
  if (PUSH_BUTTON_RUN_IS_PRESSED || PUSH_BUTTON_OK_IS_PRESSED) {
    delay(50); // Debounce
    if (PUSH_BUTTON_RUN_IS_PRESSED || PUSH_BUTTON_OK_IS_PRESSED) {
      isEmergencyHalted = !isEmergencyHalted;
      if (isEmergencyHalted) {
        enable_motor_control = false;
        drive_motor(0, 0, 0, 0);
        setMotorRPM(0, 0, 0, 0);
      } else {
        enable_motor_control = true;
      }
      // Wait for release
      while (PUSH_BUTTON_RUN_IS_PRESSED || PUSH_BUTTON_OK_IS_PRESSED) {
        delay(10);
      }
    }
  }

  // Small delay to yield to lower priority tasks
  delay(20);
}
