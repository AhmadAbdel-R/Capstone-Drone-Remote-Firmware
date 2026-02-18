#pragma once

#include <stdint.h>

struct InputSnapshot {
  uint16_t battery_mv = 0;
  uint8_t battery_pct = 0;

  uint16_t left_x_raw = 0;
  uint16_t right_x_raw = 0;
  uint16_t right_y_raw = 0;

  int16_t left_x_mapped = 0;   // -512..+512
  int16_t right_x_mapped = 0;  // -512..+512
  int16_t right_y_mapped = 0;  // -512..+512

  bool left_button_pressed = false;
};

void inputInit();
void inputReadSnapshot(InputSnapshot* out);

uint16_t inputReadBatteryMv();
uint8_t inputBatteryPercent(uint16_t battery_mv);
