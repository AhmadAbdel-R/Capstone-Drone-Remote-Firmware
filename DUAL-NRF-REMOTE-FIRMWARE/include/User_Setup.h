#pragma once

// Project-local TFT_eSPI setup for Waveshare 2" ST7789 on ESP32-WROOM-32D (VSPI).
#define USER_SETUP_INFO "ESP32-WROOM-32D ST7789 VSPI"

#define ST7789_DRIVER
#define TFT_RGB_ORDER TFT_RGB
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
// #define CGRAM_OFFSET  // Disabled for 240x320 ST7789 to avoid left-gap offset

#define TFT_MISO -1
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS 21
#define TFT_DC 22
#define TFT_RST -1

#define TFT_BL 25
#define TFT_BACKLIGHT_ON HIGH

#define SPI_FREQUENCY 40000000
#define SPI_READ_FREQUENCY 20000000

// Touch is not used on this build.
#define TOUCH_CS -1

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT
