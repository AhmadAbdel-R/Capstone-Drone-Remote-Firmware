#pragma once

#include <stdint.h>

// Legacy TFT text UI API.
// The current project uses src/ui.cpp + include/ui.h for the main LVGL UI.
// This header is provided so src/display_ui.cpp can compile if kept in /src.
void displayUiInit();
void displayUiTick(uint16_t local_battery_mv);

