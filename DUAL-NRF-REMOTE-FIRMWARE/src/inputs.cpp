#include "inputs.h"

#include "input.h"

void inputsInit() {
  inputInit();
}

uint16_t readBatteryMv() {
  return inputReadBatteryMv();
}

uint16_t readLeftThrottleRaw() {
  InputSnapshot s = {};
  inputReadSnapshot(&s);
  return s.left_x_raw;
}

bool readLeftButtonPressed() {
  InputSnapshot s = {};
  inputReadSnapshot(&s);
  return s.left_button_pressed;
}

uint16_t readRightXRaw() {
  InputSnapshot s = {};
  inputReadSnapshot(&s);
  return s.right_x_raw;
}

uint16_t readRightYRaw() {
  InputSnapshot s = {};
  inputReadSnapshot(&s);
  return s.right_y_raw;
}
