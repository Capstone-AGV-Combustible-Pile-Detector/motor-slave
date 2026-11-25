#ifndef __ROBOT_RTOS_HANDLER_H__
#define __ROBOT_RTOS_HANDLER_H__

#include <Arduino.h>

extern TaskHandle_t Task_Serial;
extern TaskHandle_t Task_Odometry;

void rtosTaskSerial(void *parameter);
void rtosCreateTaskSerial(void);

void rtosTaskOdometry(void *parameter);
void rtosCreateTaskOdometry(void);

void rtosResumeAllTasks(void);

#endif
