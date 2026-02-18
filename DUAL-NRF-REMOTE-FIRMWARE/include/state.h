#pragma once

#include <stddef.h>
#include <stdint.h>

#include "radio_link.h"

enum ArmedState : uint8_t {
  ARMED_STATE_DISARMED = 0,
  ARMED_STATE_ARMING = 1,
  ARMED_STATE_ARMED = 2
};

enum UiCommandType : uint8_t {
  UI_CMD_NONE = 0,
  UI_CMD_APPLY_RADIO_SETTINGS,
  UI_CMD_SET_BRIGHTNESS,
  UI_CMD_SET_REFRESH_MODE,
  UI_CMD_SET_ARMED_STATE,
  UI_CMD_SET_MODE_INDEX
};

struct JoystickState {
  uint16_t left_x_raw = 0;
  uint16_t right_x_raw = 0;
  uint16_t right_y_raw = 0;

  int16_t left_x_mapped = 0;   // -512..+512
  int16_t right_x_mapped = 0;  // -512..+512
  int16_t right_y_mapped = 0;  // -512..+512

  bool left_button_pressed = false;
};

struct LinkRuntimeState {
  bool chip_connected = false;
  bool link_ok = false;
  bool valid = false;
  uint32_t last_seq = 0;
  uint32_t age_ms = 0;
};

struct StreamCounters {
  uint32_t tx_count = 0;
  uint32_t tx_fail = 0;
  uint32_t tx_bytes = 0;

  uint32_t rx_ok = 0;
  uint32_t rx_bad_crc = 0;
  uint32_t rx_bad_magic = 0;
  uint32_t payload_corrupt = 0;
  uint32_t rx_bytes = 0;

  uint32_t last_seq = 0;
  uint32_t highest_seq = 0;
  uint32_t gaps = 0;
  uint32_t duplicates = 0;
  uint32_t seq_resets = 0;
};

struct UiRuntimeSettings {
  RadioRuntimeSettings radio = {};
  uint8_t backlight_duty = 216;  // 85% of 255
  uint8_t brightness_pct = 85;
  uint8_t idle_fps = 10;
  uint8_t active_fps = 30;
  bool requires_restart = false;
};

struct SharedState {
  uint32_t uptime_ms = 0;

  // Requested shared model fields.
  float batteryVoltage = 0.0f;
  uint8_t batteryPercent = 0;
  bool linkOk = false;
  uint32_t pktCount = 0;
  char lastTelemLine[96] = "Telemetry idle";
  bool gpsFix = false;
  float lat = 0.0f;
  float lon = 0.0f;
  float altitudeM = 0.0f;
  uint32_t distanceMm = 0;
  char mode[16] = "MANUAL";
  uint8_t modeIndex = 0;
  ArmedState armedState = ARMED_STATE_DISARMED;

  // Existing fields kept for compatibility with radio/input plumbing.
  bool is_remote_role = false;
  bool stress_mode_enabled = false;
  bool big_packet_mode_enabled = false;

  uint16_t battery_mv = 0;
  uint8_t battery_pct = 0;

  JoystickState joystick = {};

  LinkRuntimeState ctrl = {};
  LinkRuntimeState telm = {};

  StreamCounters ctrl_stats = {};
  StreamCounters telm_stats = {};

  uint32_t ctrl_tx_pps = 0;
  uint32_t telm_tx_pps = 0;

  uint32_t free_heap = 0;
  uint8_t ui_core = 0;
  uint8_t control_core = 1;

  UiRuntimeSettings settings = {};
};

using SystemState = SharedState;

struct UiCommand {
  UiCommandType type = UI_CMD_NONE;
  RadioRuntimeSettings radio = {};
  uint8_t backlight_duty = 216;
  uint8_t brightness_pct = 85;
  uint8_t idle_fps = 10;
  uint8_t active_fps = 30;
  ArmedState armed_state = ARMED_STATE_DISARMED;
  uint8_t mode_index = 0;
};
