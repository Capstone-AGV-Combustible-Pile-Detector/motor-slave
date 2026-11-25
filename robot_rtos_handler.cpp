#include <Arduino.h>
#include "robot_rtos_handler.h"
#include "freertos/portmacro.h"

#include "robot_serial.h"
#include "robot_control_kinematics.h"
#include "robot_motor_encoder.h"
#include "robot_imu.h"

TaskHandle_t Task_Serial = NULL;
TaskHandle_t Task_Odometry = NULL;

void rtosTaskSerial(void *parameter){
  for(;;){
    // 1. Read serial data from USB Serial and UART2
    SerialDataReceived();

    // 2. Parse serial data if complete
    SerialDataParsing();

    // 3. Convert and execute velocity commands
    SerialDataConversion();

    // 4. Reset serial input buffers
    SerialDataReset();

    // 5. Watchdog: Stop motors if ROS2 communication drops
    check_ros2_watchdog();

    // 6. Send odometry telemetry back to ROS2
    send_ros2_odometry();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void rtosCreateTaskSerial(){
  xTaskCreatePinnedToCore(
    rtosTaskSerial,
    "rtosTaskSerial",
    4096,
    NULL,
    1,
    &Task_Serial,
    0);
  // Suspend immediately; setup() calls rtosResumeAllTasks() when fully ready.
  if (Task_Serial != NULL) vTaskSuspend(Task_Serial);
}

void rtosTaskOdometry(void *parameter){
  for(;;){
    // Read all encoder RPM & calculate wheel velocities
    encoder_calculation();

    // Read MPU6050 and update robot heading
    robot_imu_update();

    // Closed-loop PID control of all 4 motor RPMs
    global_motor_control();

    // Update forward kinematics and robot odometry pose
    robot_odometry_timer();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void rtosCreateTaskOdometry(){
  xTaskCreatePinnedToCore(
    rtosTaskOdometry,
    "rtosTaskOdometry",
    4096,
    NULL,
    1,
    &Task_Odometry,
    0);
  // Suspend immediately; setup() calls rtosResumeAllTasks() when fully ready.
  if (Task_Odometry != NULL) vTaskSuspend(Task_Odometry);
}

void rtosResumeAllTasks(){
  if (Task_Serial   != NULL) vTaskResume(Task_Serial);
  if (Task_Odometry != NULL) vTaskResume(Task_Odometry);
}
