#include <Arduino.h>
#include "robot_serial.h"
#include "robot_control_kinematics.h"
#include "robot_motor_encoder.h"
#include "robot_imu.h"

HardwareSerial SlaveSerial2(2);

const char startDataIdentifier = '*';
const char dataSeparator = ',';
const char stopDataIdentifier = '#';

char dataReceived;

String dataPackage;
String bufferDataIn;
String bufferDataParsing[ARRAY_SIZE];

bool dataIsComplete = false;
byte indexOfData = 0;

float SerialData[NUM_OF_DATA];

unsigned long lastCommandTime = 0;
unsigned long lastOdomSendTime = 0;
unsigned long rxPacketCount = 0;
bool ros2Connected = false;

// Track active port (USB Serial or UART2)
int activeSerialPort = 0; // 0 = USB Serial, 1 = SlaveSerial2

void robot_serial_init(){
  Serial.begin(SLAVE_SERIAL_BAUDRATE);
  SlaveSerial2.begin(SLAVE_SERIAL_BAUDRATE, SERIAL_8N1, RX2, TX2);
  lastCommandTime = millis();
}

void SerialDataReceived(){
  // Check USB Serial first
  while (Serial.available() > 0){
    dataReceived = Serial.read();
    if (dataReceived == startDataIdentifier){
      bufferDataIn = startDataIdentifier;
      bufferDataIn += Serial.readStringUntil(stopDataIdentifier);
      dataIsComplete = true;
      bufferDataIn += stopDataIdentifier;
      activeSerialPort = 0;
      return;
    }
  }

  // Also check Hardware UART2 (pins 16/17)
  while (SlaveSerial2.available() > 0){
    dataReceived = SlaveSerial2.read();
    if (dataReceived == startDataIdentifier){
      bufferDataIn = startDataIdentifier;
      bufferDataIn += SlaveSerial2.readStringUntil(stopDataIdentifier);
      dataIsComplete = true;
      bufferDataIn += stopDataIdentifier;
      activeSerialPort = 1;
      return;
    }
  }
}

void SerialDataParsing(){
  if (!dataIsComplete) return;

  indexOfData = 0;
  for (int i = 0; i < ARRAY_SIZE; i++) {
    bufferDataParsing[i] = "";
  }

  for (int i = 0; i < bufferDataIn.length(); i++){
    if (bufferDataIn[i] == startDataIdentifier){
      bufferDataParsing[indexOfData] = "";
    } else if (bufferDataIn[i] != dataSeparator && bufferDataIn[i] != stopDataIdentifier){
      bufferDataParsing[indexOfData] += bufferDataIn[i];
    } else if (bufferDataIn[i] == dataSeparator){
      if (indexOfData < ARRAY_SIZE - 1) {
        indexOfData++;
        bufferDataParsing[indexOfData] = "";
      }
    }
  }
}

void SerialDataConversion(){
  if (!dataIsComplete) return;

  // Packet format: *mode,yaw,vx,vy,w#
  // Mode
  SerialData[0] = bufferDataParsing[0].toInt();

  // Floating point values: yaw, twist_linear_x, twist_linear_y, twist_angular_z
  for (int i = 1; i < NUM_OF_DATA; i++){
    if (i <= indexOfData) {
      SerialData[i] = bufferDataParsing[i].toFloat();
    } else {
      SerialData[i] = 0.0f;
    }
  }

  // Update timestamps and packet count
  lastCommandTime = millis();
  rxPacketCount++;
  ros2Connected = true;

  // Mode 11: Reset Odometry command (*11,x,y,yaw#)
  if (SerialData[0] == 11) {
    float reset_x = SerialData[1];
    float reset_y = SerialData[2];
    float reset_yaw = SerialData[3];
    reset_robot_odometry(reset_x, reset_y, reset_yaw);
    return;
  }

  // Execute motion directly if mode is 10 (ROS2 navigation) or general control
  float twist_x = SerialData[2];
  float twist_y = SerialData[3];
  float twist_w = SerialData[4];

  set_robot_speed(twist_x, twist_y, twist_w);
}

void SerialDataReset(){
  dataReceived = 0;
  bufferDataIn = "";
  dataIsComplete = false;
  indexOfData = 0;
}

void send_ros2_odometry(){
  unsigned long now = millis();
  // Transmit telemetry at ~25 Hz (every 40ms)
  if (now - lastOdomSendTime >= 40){
    lastOdomSendTime = now;

    float omega_z_rad = v_robot_w;

    // Filter tiny residual float noise when robot is stopped
    float out_vel_x = (fabs(v_robot_l_x) < 0.001f) ? 0.0f : v_robot_l_x;
    float out_vel_y = (fabs(v_robot_l_y) < 0.001f) ? 0.0f : v_robot_l_y;
    float out_vel_w = (fabs(omega_z_rad) < 0.002f) ? 0.0f : omega_z_rad;

    // Packet format: *10,posX,posY,posZ,yaw,velX,velY,velW#
    dataPackage = (String)startDataIdentifier + "10" + dataSeparator
                  + String(RobotActualPositionX, 3) + dataSeparator
                  + String(RobotActualPositionY, 3) + dataSeparator
                  + "0.000" + dataSeparator
                  + String(RobotActualPositionTheta, 2) + dataSeparator
                  + String(out_vel_x, 3) + dataSeparator
                  + String(out_vel_y, 3) + dataSeparator
                  + String(out_vel_w, 3) + stopDataIdentifier;

    // Send on both ports so receiver gets it regardless of physical connection
    Serial.println(dataPackage);
    SlaveSerial2.println(dataPackage);
  }
}

void check_ros2_watchdog(){
  // Safety timeout: If no packet received in the last 500ms, stop robot
  if (millis() - lastCommandTime > 500){
    ros2Connected = false;
    set_robot_speed(0.0f, 0.0f, 0.0f);
    setMotorRPM(0, 0, 0, 0);
  }
}
