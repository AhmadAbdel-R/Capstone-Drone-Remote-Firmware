#pragma once

#include <stdint.h>

void inputsInit();

uint16_t readBatteryMv();
uint16_t readLeftThrottleRaw();
bool readLeftButtonPressed();
uint16_t readRightXRaw();
uint16_t readRightYRaw();
