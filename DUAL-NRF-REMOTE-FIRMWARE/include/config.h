#pragma once

#include <RF24.h>

// ---------------- role / test toggles ----------------
#ifndef DEVICE_ROLE_REMOTE
#define DEVICE_ROLE_REMOTE 1
#endif

#ifndef TEST_MODE_BIG_PACKET
#define TEST_MODE_BIG_PACKET 0
#endif

#ifndef STRESS_TEST_ENABLE
#define STRESS_TEST_ENABLE 0
#endif

#ifndef STRESS_TEST_TARGET_COUNT
#define STRESS_TEST_TARGET_COUNT 10000UL
#endif

#ifndef STRESS_CTRL_SEND_INTERVAL_MS
#define STRESS_CTRL_SEND_INTERVAL_MS 0UL
#endif

#ifndef STRESS_TELM_SEND_INTERVAL_MS
#define STRESS_TELM_SEND_INTERVAL_MS 0UL
#endif

#ifndef DEBUG_LOG_ENABLE
#define DEBUG_LOG_ENABLE 0
#endif

#ifndef DEBUG_RX_PRINT_CTRL
#define DEBUG_RX_PRINT_CTRL 1
#endif

#ifndef DEBUG_RX_PRINT_TELM
#define DEBUG_RX_PRINT_TELM 1
#endif

// ---------------- serial / ui ----------------
inline constexpr uint32_t SERIAL_BAUD_RATE = 115200UL;
inline constexpr uint32_t SERIAL_BOOT_DELAY_MS = 250UL;
inline constexpr uint32_t UI_REFRESH_PERIOD_MS = 100UL;  // 10Hz max
inline constexpr uint8_t TFT_ROTATION = 1;               // stable landscape for this ST7789 panel
// Some Waveshare ST7789 modules show a left-gap artifact on rotation=3.
// Keep rotation=1 and optionally force the upside-down landscape MADCTL mode.
inline constexpr bool TFT_FORCE_LANDSCAPE_FLIP_WITH_ROT1 = true;
// Color order is controlled from TFT_eSPI setup via TFT_RGB_ORDER.
// Waveshare 2" ST7789 glass needs display inversion enabled for correct contrast/sharpness.
inline constexpr bool UI_TFT_INVERT_COLORS = true;
inline constexpr uint8_t UI_DEFAULT_BRIGHTNESS_PCT = 85;
inline constexpr uint8_t UI_IDLE_FPS_DEFAULT = 10;
inline constexpr uint8_t UI_ACTIVE_FPS_DEFAULT = 30;
inline constexpr uint8_t UI_ACTIVE_FPS_ECO = 20;
inline constexpr uint32_t UI_BUTTON_LONG_PRESS_MS = 600UL;
inline constexpr uint32_t UI_BUTTON_DEBOUNCE_MS = 35UL;
inline constexpr bool UI_STARTUP_COLOR_TEST_ENABLE = true;
inline constexpr uint32_t UI_STARTUP_COLOR_TEST_STEP_MS = 2600UL;
inline constexpr uint16_t UI_BACKLIGHT_PWM_FREQ_HZ = 5000U;
inline constexpr uint8_t UI_BACKLIGHT_PWM_BITS = 8;
inline constexpr uint8_t UI_BACKLIGHT_PWM_CHANNEL = 0;
inline constexpr uint8_t JOYSTICK_DEADZONE_COUNTS = 24;
inline constexpr uint8_t INPUT_ADC_MEDIAN_SAMPLES = 15;
inline constexpr uint8_t INPUT_AXIS_EMA_ALPHA_NUM = 1;   // alpha = 1/4
inline constexpr uint8_t INPUT_AXIS_EMA_ALPHA_DEN = 4;
inline constexpr int16_t INPUT_AXIS_SLEW_STEP = 26;      // max mapped counts per update

// ---------------- RF config ----------------
#ifndef RF_CHANNEL_CTRL
#define RF_CHANNEL_CTRL 108
#endif

#ifndef RF_CHANNEL_TELM
#define RF_CHANNEL_TELM 108
#endif

#ifndef RF_DATA_RATE_CTRL
#define RF_DATA_RATE_CTRL RF24_250KBPS
#endif

#ifndef RF_DATA_RATE_TELM
#define RF_DATA_RATE_TELM RF24_250KBPS
#endif

#ifndef RF_CRC_LEN_CTRL
#define RF_CRC_LEN_CTRL RF24_CRC_16
#endif

#ifndef RF_CRC_LEN_TELM
#define RF_CRC_LEN_TELM RF24_CRC_16
#endif

#ifndef RF_PA_LEVEL_CTRL
#define RF_PA_LEVEL_CTRL RF24_PA_MAX
#endif

#ifndef RF_PA_LEVEL_TELM
#define RF_PA_LEVEL_TELM RF24_PA_MAX
#endif

#ifndef RF_ADDR_WIDTH_CTRL
#define RF_ADDR_WIDTH_CTRL 5
#endif

#ifndef RF_ADDR_WIDTH_TELM
#define RF_ADDR_WIDTH_TELM 5
#endif

#ifndef CTRL_DYNAMIC_PAYLOADS
#define CTRL_DYNAMIC_PAYLOADS 0
#endif

#ifndef TELM_DYNAMIC_PAYLOADS
#define TELM_DYNAMIC_PAYLOADS 0
#endif

#ifndef CTRL_AUTO_ACK
#define CTRL_AUTO_ACK 0
#endif

#ifndef CTRL_RETRY_DELAY
#define CTRL_RETRY_DELAY 0
#endif

#ifndef CTRL_RETRY_COUNT
#define CTRL_RETRY_COUNT 0
#endif

#ifndef TELM_AUTO_ACK
#define TELM_AUTO_ACK 1
#endif

#ifndef TELM_RETRY_DELAY
#define TELM_RETRY_DELAY 5
#endif

#ifndef TELM_RETRY_COUNT
#define TELM_RETRY_COUNT 15
#endif

// ---------------- periods / limits ----------------
inline constexpr uint32_t SUMMARY_PERIOD_MS = 1000UL;
inline constexpr uint32_t CTRL_LINK_TIMEOUT_MS = 200UL;
inline constexpr uint32_t TELM_LINK_TIMEOUT_MS = 500UL;
inline constexpr uint32_t CTRL_TX_PERIOD_MS = 20UL;
inline constexpr uint32_t TELM_TX_PERIOD_MS = 100UL;
inline constexpr uint32_t CTRL_RX_PRINT_MIN_INTERVAL_MS = 100UL;
inline constexpr uint32_t TELM_RX_PRINT_MIN_INTERVAL_MS = 250UL;
inline constexpr uint32_t BAD_RX_PRINT_MIN_INTERVAL_MS = 500UL;
inline constexpr uint32_t LATENCY_SANITY_MAX_MS = 60000UL;
inline constexpr uint32_t SEQ_RESET_THRESHOLD = 3000UL;
inline constexpr uint32_t STRESS_PROGRESS_PERIOD_MS = 1000UL;
inline constexpr uint32_t STRESS_FINAL_REPORT_PERIOD_MS = 2000UL;
inline constexpr uint32_t STRESS_RX_IDLE_DONE_MS = 3000UL;
inline constexpr uint32_t STRESS_NEAR_END_MARGIN = 2000UL;

// ---------------- ADC conversion ----------------
// Approximate full-scale ADC input voltage at selected attenuation.
#ifndef ADC_VREF_MV
#define ADC_VREF_MV 3300UL
#endif

// Optional calibration scaling for battery conversion (1.000 = 1000/1000).
#ifndef ADC_BATTERY_CAL_SCALE_NUM
#define ADC_BATTERY_CAL_SCALE_NUM 1000UL
#endif

#ifndef ADC_BATTERY_CAL_SCALE_DEN
#define ADC_BATTERY_CAL_SCALE_DEN 1000UL
#endif

inline constexpr uint16_t LIPO_EMPTY_MV = 3300U;
inline constexpr uint16_t LIPO_FULL_MV = 4200U;

static_assert(DEVICE_ROLE_REMOTE == 0 || DEVICE_ROLE_REMOTE == 1,
              "DEVICE_ROLE_REMOTE must be 0 (FCU) or 1 (REMOTE).");
static_assert(STRESS_TEST_TARGET_COUNT > 0, "STRESS_TEST_TARGET_COUNT must be > 0.");
static_assert(ADC_BATTERY_CAL_SCALE_DEN != 0, "ADC_BATTERY_CAL_SCALE_DEN must be > 0.");
