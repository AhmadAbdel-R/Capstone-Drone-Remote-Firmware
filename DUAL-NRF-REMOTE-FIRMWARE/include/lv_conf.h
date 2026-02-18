#pragma once

// Lightweight LVGL configuration for ESP32 + ST7789 (320x240 landscape).
// Display is 262K (18-bit capable) ST7789VW, but LVGL is driving it as RGB565 (16-bit) over SPI.
// Hardware: 262K (RGB666 capable)
// LVGL pipeline: RGB565 (16-bit)
// Result: still looks great; panel capability is higher than what we transmit.
// Do not use 18/24-bit LVGL color depth for this SPI path.
// Unspecified options use LVGL defaults from lv_conf_internal.h.
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

#define LV_HOR_RES_MAX 320
#define LV_VER_RES_MAX 240

#define LV_MEM_SIZE (64U * 1024U)
#define LV_MEM_CUSTOM 0

#define LV_USE_LOG 0

#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1

#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1

#define LV_USE_CHART 1

#endif  // LV_CONF_H
