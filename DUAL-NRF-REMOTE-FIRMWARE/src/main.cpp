#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "input.h"
#include "log.h"
#include "radio_link.h"
#include "state.h"
#include "ui.h"

namespace {

SystemState gState = {};
SemaphoreHandle_t gStateMutex = nullptr;
QueueHandle_t gUiCmdQueue = nullptr;

constexpr uint32_t STATE_PUBLISH_PERIOD_MS = 20UL;
constexpr uint8_t kBacklightDutyMax = 255U;

const char* kFlightModes[] = {"MANUAL", "ALT HOLD", "RTL"};
constexpr uint8_t kFlightModeCount = static_cast<uint8_t>(sizeof(kFlightModes) / sizeof(kFlightModes[0]));

uint8_t clampU8(int value, uint8_t lo, uint8_t hi) {
  if (value < static_cast<int>(lo)) {
    return lo;
  }
  if (value > static_cast<int>(hi)) {
    return hi;
  }
  return static_cast<uint8_t>(value);
}

uint8_t dutyFromPercent(uint8_t pct) {
  return static_cast<uint8_t>((static_cast<uint32_t>(pct) * kBacklightDutyMax) / 100U);
}

uint8_t percentFromDuty(uint8_t duty) {
  return static_cast<uint8_t>((static_cast<uint32_t>(duty) * 100U) / kBacklightDutyMax);
}

uint8_t normalizeModeIndex(uint8_t mode_index) {
  if (kFlightModeCount == 0U) {
    return 0U;
  }
  return static_cast<uint8_t>(mode_index % kFlightModeCount);
}

const char* modeLabel(uint8_t mode_index) {
  return kFlightModes[normalizeModeIndex(mode_index)];
}

void fillStreamCounters(const RadioLinkStatsSnapshot& in, StreamCounters* out) {
  if (out == nullptr) {
    return;
  }
  out->tx_count = in.tx_count;
  out->tx_fail = in.tx_fail;
  out->tx_bytes = in.tx_bytes;
  out->rx_ok = in.rx_ok;
  out->rx_bad_crc = in.rx_bad_crc;
  out->rx_bad_magic = in.rx_bad_magic;
  out->payload_corrupt = in.payload_corrupt;
  out->rx_bytes = in.rx_bytes;
  out->last_seq = in.last_seq;
  out->highest_seq = in.highest_seq;
  out->gaps = in.gaps;
  out->duplicates = in.duplicates;
  out->seq_resets = in.seq_resets;
}

uint32_t calcRate(uint32_t count_now, uint32_t count_prev, uint32_t dt_ms) {
  if (dt_ms == 0U) {
    return 0U;
  }
  const uint32_t delta = static_cast<uint32_t>(count_now - count_prev);
  return static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000ULL) / dt_ms);
}

void publishSystemState() {
  static uint32_t last_publish_ms = 0;
  static uint32_t last_ctrl_tx = 0;
  static uint32_t last_telm_tx = 0;
  static uint32_t last_pps_ms = 0;
  static bool telem_line_init = false;
  static bool last_role_link_ok = false;
  static bool last_role_is_remote = false;
  static uint32_t last_role_seq = 0;

  const uint32_t now = millis();
  if ((now - last_publish_ms) < STATE_PUBLISH_PERIOD_MS) {
    return;
  }
  last_publish_ms = now;

  InputSnapshot input = {};
  inputReadSnapshot(&input);

  RadioLinkUiSnapshot link = {};
  RadioLinkStatsSnapshot ctrl_stats = {};
  RadioLinkStatsSnapshot telm_stats = {};
  bool ctrl_chip = false;
  bool telm_chip = false;
  RadioRuntimeSettings runtime = {};

  radioLinkGetUiSnapshot(&link);
  radioLinkGetStatsSnapshot(&ctrl_stats, &telm_stats);
  radioLinkGetChipStatus(&ctrl_chip, &telm_chip);
  radioLinkGetRuntimeSettings(&runtime);

  const uint32_t dt_ms = (last_pps_ms == 0U) ? 1000UL : static_cast<uint32_t>(now - last_pps_ms);
  const uint32_t ctrl_pps = calcRate(ctrl_stats.tx_count, last_ctrl_tx, dt_ms);
  const uint32_t telm_pps = calcRate(telm_stats.tx_count, last_telm_tx, dt_ms);
  last_ctrl_tx = ctrl_stats.tx_count;
  last_telm_tx = telm_stats.tx_count;
  last_pps_ms = now;

  UiRuntimeSettings settings = {};
  settings.radio = runtime;
  settings.backlight_duty = dutyFromPercent(UI_DEFAULT_BRIGHTNESS_PCT);
  settings.brightness_pct = UI_DEFAULT_BRIGHTNESS_PCT;
  settings.idle_fps = UI_IDLE_FPS_DEFAULT;
  settings.active_fps = UI_ACTIVE_FPS_DEFAULT;
  settings.requires_restart = false;

  uint8_t mode_index = 0U;
  ArmedState armed_state = ARMED_STATE_DISARMED;
  float lat = 0.0f;
  float lon = 0.0f;
  float altitude_m = 0.0f;
  uint32_t distance_mm = 0U;
  char prev_telem_line[sizeof(gState.lastTelemLine)] = {};

  if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
    settings.backlight_duty = gState.settings.backlight_duty;
    settings.brightness_pct = gState.settings.brightness_pct;
    settings.idle_fps = gState.settings.idle_fps;
    settings.active_fps = gState.settings.active_fps;
    settings.requires_restart = gState.settings.requires_restart;

    mode_index = gState.modeIndex;
    armed_state = gState.armedState;
    lat = gState.lat;
    lon = gState.lon;
    altitude_m = gState.altitudeM;
    distance_mm = gState.distanceMm;
    snprintf(prev_telem_line, sizeof(prev_telem_line), "%s", gState.lastTelemLine);

    xSemaphoreGive(gStateMutex);
  }

  settings.backlight_duty = clampU8(settings.backlight_duty, 0, kBacklightDutyMax);
  settings.brightness_pct = percentFromDuty(settings.backlight_duty);

  SystemState next = {};
  next.uptime_ms = now;
  next.is_remote_role = link.is_remote_role;
  next.stress_mode_enabled = link.stress_mode_enabled;
  next.big_packet_mode_enabled = link.big_packet_mode_enabled;

  next.battery_mv = input.battery_mv;
  next.battery_pct = input.battery_pct;
  next.batteryVoltage = static_cast<float>(input.battery_mv) / 1000.0f;
  next.batteryPercent = input.battery_pct;

  next.joystick.left_x_raw = input.left_x_raw;
  next.joystick.right_x_raw = input.right_x_raw;
  next.joystick.right_y_raw = input.right_y_raw;
  next.joystick.left_x_mapped = input.left_x_mapped;
  next.joystick.right_x_mapped = input.right_x_mapped;
  next.joystick.right_y_mapped = input.right_y_mapped;
  next.joystick.left_button_pressed = input.left_button_pressed;

  next.ctrl.chip_connected = ctrl_chip;
  next.ctrl.link_ok = link.ctrl_link_ok;
  next.ctrl.valid = link.ctrl_valid;
  next.ctrl.last_seq = link.ctrl_seq;
  next.ctrl.age_ms = link.ctrl_age_ms;

  next.telm.chip_connected = telm_chip;
  next.telm.link_ok = link.telm_link_ok;
  next.telm.valid = link.telm_valid;
  next.telm.last_seq = link.telm_seq;
  next.telm.age_ms = link.telm_age_ms;

  fillStreamCounters(ctrl_stats, &next.ctrl_stats);
  fillStreamCounters(telm_stats, &next.telm_stats);

  const bool role_link_ok = next.is_remote_role ? link.telm_link_ok : link.ctrl_link_ok;
  const uint32_t role_pkt_count = next.is_remote_role ? telm_stats.rx_ok : ctrl_stats.rx_ok;
  const uint32_t role_seq = next.is_remote_role ? link.telm_seq : link.ctrl_seq;
  const uint32_t role_age_ms = next.is_remote_role ? link.telm_age_ms : link.ctrl_age_ms;

  next.linkOk = role_link_ok;
  next.pktCount = role_pkt_count;
  next.gpsFix = link.telm_valid;
  next.lat = lat;
  next.lon = lon;
  next.altitudeM = altitude_m;
  next.distanceMm = distance_mm;
  next.modeIndex = normalizeModeIndex(mode_index);
  next.armedState = armed_state;
  snprintf(next.mode, sizeof(next.mode), "%s", modeLabel(next.modeIndex));

  const bool role_switched = (!telem_line_init) || (last_role_is_remote != next.is_remote_role);
  const bool link_changed = role_switched || (role_link_ok != last_role_link_ok);
  const bool seq_changed = role_switched || (role_seq != last_role_seq);

  if (link_changed || seq_changed) {
    snprintf(next.lastTelemLine,
             sizeof(next.lastTelemLine),
             "%s seq=%lu age=%lums rx=%lu",
             role_link_ok ? "LINK OK" : "LINK LOST",
             static_cast<unsigned long>(role_seq),
             static_cast<unsigned long>(role_age_ms),
             static_cast<unsigned long>(role_pkt_count));
    telem_line_init = true;
    last_role_link_ok = role_link_ok;
    last_role_is_remote = next.is_remote_role;
    last_role_seq = role_seq;
  } else {
    snprintf(next.lastTelemLine, sizeof(next.lastTelemLine), "%s", prev_telem_line);
  }

  next.ctrl_tx_pps = ctrl_pps;
  next.telm_tx_pps = telm_pps;
  next.free_heap = ESP.getFreeHeap();
  next.ui_core = 0;
  next.control_core = static_cast<uint8_t>(xPortGetCoreID());
  next.settings = settings;

  if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
    gState = next;
    xSemaphoreGive(gStateMutex);
  }
}

void processUiCommands() {
  UiCommand cmd = {};
  while (xQueueReceive(gUiCmdQueue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case UI_CMD_APPLY_RADIO_SETTINGS: {
        bool requires_restart = false;
        const bool ok = radioLinkApplyRuntimeSettings(&cmd.radio, &requires_restart);
        if (!ok) {
          LOG_WARN("UI->Radio settings apply failed");
        }
        if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
          if (ok) {
            gState.settings.radio = cmd.radio;
          }
          gState.settings.requires_restart = requires_restart;
          xSemaphoreGive(gStateMutex);
        }
        break;
      }

      case UI_CMD_SET_BRIGHTNESS:
        if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
          const uint8_t duty = clampU8(cmd.backlight_duty, 0, kBacklightDutyMax);
          gState.settings.backlight_duty = duty;
          gState.settings.brightness_pct = percentFromDuty(duty);
          xSemaphoreGive(gStateMutex);
        }
        break;

      case UI_CMD_SET_REFRESH_MODE:
        if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
          gState.settings.idle_fps = clampU8(cmd.idle_fps, 1, 60);
          gState.settings.active_fps = clampU8(cmd.active_fps, 1, 60);
          xSemaphoreGive(gStateMutex);
        }
        break;

      case UI_CMD_SET_ARMED_STATE:
        if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
          gState.armedState = cmd.armed_state;
          xSemaphoreGive(gStateMutex);
        }
        break;

      case UI_CMD_SET_MODE_INDEX:
        if (xSemaphoreTake(gStateMutex, 0) == pdTRUE) {
          gState.modeIndex = normalizeModeIndex(cmd.mode_index);
          snprintf(gState.mode, sizeof(gState.mode), "%s", modeLabel(gState.modeIndex));
          xSemaphoreGive(gStateMutex);
        }
        break;

      default:
        break;
    }
  }
}

void initSharedStateDefaults() {
  RadioRuntimeSettings runtime = {};
  radioLinkGetRuntimeSettings(&runtime);

  gState = SharedState{};
  gState.settings.radio = runtime;
  gState.settings.backlight_duty = dutyFromPercent(UI_DEFAULT_BRIGHTNESS_PCT);
  gState.settings.brightness_pct = UI_DEFAULT_BRIGHTNESS_PCT;
  gState.settings.idle_fps = UI_IDLE_FPS_DEFAULT;
  gState.settings.active_fps = UI_ACTIVE_FPS_DEFAULT;
  gState.batteryVoltage = 0.0f;
  gState.batteryPercent = 0;
  gState.linkOk = false;
  gState.pktCount = 0;
  snprintf(gState.lastTelemLine, sizeof(gState.lastTelemLine), "%s", "LINK LOST seq=0 age=0ms rx=0");
  snprintf(gState.mode, sizeof(gState.mode), "%s", modeLabel(0));
  gState.modeIndex = 0;
  gState.armedState = ARMED_STATE_DISARMED;
  gState.ui_core = 0;
  gState.control_core = static_cast<uint8_t>(xPortGetCoreID());
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  uiPrepareDisplayBoot();
  delay(SERIAL_BOOT_DELAY_MS);

  Serial.println();
  Serial.println("Serial OK");
  Serial.println("[i] Serial LOG OK");

  inputInit();
  radioLinkInit();

  gStateMutex = xSemaphoreCreateMutex();
  gUiCmdQueue = xQueueCreate(8, sizeof(UiCommand));
  if ((gStateMutex == nullptr) || (gUiCmdQueue == nullptr)) {
    LOG_ERR("Failed to create UI state synchronization primitives");
    while (true) {
      delay(1000);
    }
  }

  initSharedStateDefaults();
  publishSystemState();

  if (!uiStart(&gState, gStateMutex, gUiCmdQueue)) {
    LOG_ERR("Failed to start UI task");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  radioLinkTick();
  processUiCommands();
  publishSystemState();
}
