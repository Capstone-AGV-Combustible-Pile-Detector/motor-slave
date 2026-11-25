#ifndef __ROBOT_SERIAL_H__
#define __ROBOT_SERIAL_H__

#include <stdint.h>
#include <Arduino.h>

#define RX2 16
#define TX2 17
#define SLAVE_SERIAL_BAUDRATE 115200

#define ARRAY_SIZE 21
#define NUM_OF_DATA 10

extern HardwareSerial SlaveSerial2;

extern const char startDataIdentifier;
extern const char dataSeparator;
extern const char stopDataIdentifier;

extern char dataReceived;

extern String dataPackage;
extern String bufferDataIn;
extern String bufferDataParsing[ARRAY_SIZE];

extern bool dataIsComplete;
extern byte indexOfData;
extern float SerialData[NUM_OF_DATA];

extern unsigned long lastCommandTime;
extern unsigned long lastOdomSendTime;
extern unsigned long rxPacketCount;
extern bool ros2Connected;

void robot_serial_init(void);
void SerialDataReceived(void);
void SerialDataParsing(void);
void SerialDataConversion(void);
void SerialDataReset(void);
void send_ros2_odometry(void);
void check_ros2_watchdog(void);

#endif
