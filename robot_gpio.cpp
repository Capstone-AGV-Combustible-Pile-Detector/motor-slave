#include <Arduino.h>
#include "robot_gpio.h"

void robot_gpio_init(){
  pinMode(PB_UP, INPUT_PULLUP);
  pinMode(PB_DOWN, INPUT_PULLUP);
  pinMode(PB_OK, INPUT_PULLUP);
  pinMode(PB_RUN, INPUT_PULLUP);
}
