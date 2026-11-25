#ifndef __ROBOT_GPIO_H__
#define __ROBOT_GPIO_H__

#include <Arduino.h>

#define PB_RUN 12
#define PB_UP 5
#define PB_DOWN 3
#define PB_OK 15

#define PUSH_BUTTON_OK_IS_PRESSED   (digitalRead(PB_OK) == LOW)
#define PUSH_BUTTON_DOWN_IS_PRESSED (digitalRead(PB_DOWN) == LOW)
#define PUSH_BUTTON_UP_IS_PRESSED   (digitalRead(PB_UP) == LOW)
#define PUSH_BUTTON_RUN_IS_PRESSED  (digitalRead(PB_RUN) == LOW)

void robot_gpio_init(void);

#endif
