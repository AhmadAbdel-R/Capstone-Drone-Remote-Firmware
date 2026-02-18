#pragma once

#include <Arduino.h>

// ---------------- display (Waveshare 2" ST7789, 240x320) ----------------
inline constexpr int8_t PIN_TFT_MISO = -1;
inline constexpr uint8_t PIN_TFT_SCK = 18;
inline constexpr uint8_t PIN_TFT_MOSI = 23;
inline constexpr uint8_t PIN_TFT_CS = 21;
inline constexpr uint8_t PIN_TFT_DC = 22;
inline constexpr int8_t PIN_TFT_RST = -1;  // Tied to EN
inline constexpr uint8_t PIN_TFT_BL = 25;

inline constexpr uint32_t TFT_SPI_HZ = 40000000UL;

// ---------------- nRF24 (2x modules on shared HSPI) ----------------
inline constexpr uint8_t PIN_NRF_SCK = 14;
inline constexpr uint8_t PIN_NRF_MOSI = 13;
inline constexpr uint8_t PIN_NRF_MISO = 27;

inline constexpr uint8_t PIN_CTRL_CSN = 17;
inline constexpr uint8_t PIN_CTRL_CE = 16;
inline constexpr uint8_t PIN_CTRL_IRQ = 19;

inline constexpr uint8_t PIN_TELM_CSN = 5;   // strap pin
inline constexpr uint8_t PIN_TELM_CE = 4;    // strap pin
inline constexpr uint8_t PIN_TELM_IRQ = 26;

inline constexpr uint32_t NRF_SPI_HZ = 10000000UL;

// Allow strap pins to settle before driving CE/CSN.
inline constexpr uint32_t RADIO_BOOT_DELAY_MS = 120UL;

// ---------------- analog / inputs ----------------
inline constexpr uint8_t PIN_BATTERY_ADC = 33;
inline constexpr uint8_t PIN_LEFT_JOY_X = 34;
inline constexpr uint8_t PIN_LEFT_JOY_BTN = 15;  // strap pin
inline constexpr uint8_t PIN_RIGHT_JOY_X = 35;
inline constexpr uint8_t PIN_RIGHT_JOY_Y = 32;

inline constexpr uint32_t BATTERY_DIVIDER_R1_OHM = 10000UL;
inline constexpr uint32_t BATTERY_DIVIDER_R2_OHM = 10000UL;

inline constexpr uint16_t ADC_MAX_COUNT = 4095U;
