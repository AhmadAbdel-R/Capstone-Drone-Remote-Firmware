#include "ui.h"

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_heap_caps.h>
#include <lvgl.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "log.h"
#include "pins.h"

namespace {

constexpr int16_t kScreenW = 320;
constexpr int16_t kScreenH = 240;
constexpr int16_t kTopBarH = 32;
constexpr int16_t kQuickNavH = 26;
constexpr int16_t kContentY = kTopBarH + kQuickNavH + 2;
constexpr int16_t kContentH = kScreenH - kContentY;

constexpr uint32_t kTftPowerSettleDelayMs = 180;
constexpr uint32_t kTftRetryGapDelayMs = 20;
constexpr uint8_t kTftInitPasses = 2;

constexpr uint16_t kLvBufferLines = 24;
constexpr uint32_t kStatePollPeriodMs = 60;
constexpr uint32_t kArmHoldMs = 1000;
constexpr uint32_t kLowBatteryWarnPeriodMs = 2500;
constexpr uint32_t kDiagSamplePeriodMs = 180;
constexpr uint32_t kNavStepRepeatMs = 260;
constexpr uint32_t kEditStepRepeatMs = 200;
constexpr int16_t kNavAxisStepThreshold = 180;
constexpr int16_t kNavAxisReleaseThreshold = 110;
constexpr int16_t kWakeAxisThreshold = 120;
constexpr int16_t kEditAxisStepThreshold = 220;
constexpr int16_t kEditAxisReleaseThreshold = 140;

constexpr uint8_t kBacklightDutyMax = 255;

constexpr uint8_t kTelemetryCapacity = 40;
constexpr uint8_t kTelemetryVisibleMax = 38;
constexpr size_t kTelemetryLineChars = 80;
constexpr size_t kTelemetryTextChars = 4096;

constexpr uint32_t C_BG = 0x0D0F12;
constexpr uint32_t C_TOP = 0x161A1F;
constexpr uint32_t C_CARD = 0x1D2228;
constexpr uint32_t C_CARD_ACTIVE = 0x2A323B;
constexpr uint32_t C_BORDER = 0x3E4853;
constexpr uint32_t C_TEXT = 0xE7ECEF;
constexpr uint32_t C_DIM = 0xA9B1B8;
constexpr uint32_t C_GREEN = 0x55C271;
constexpr uint32_t C_RED = 0xE35D5D;

const char* kFlightModes[] = {"MANUAL", "ALT HOLD", "RTL"};
constexpr uint8_t kFlightModeCount = static_cast<uint8_t>(sizeof(kFlightModes) / sizeof(kFlightModes[0]));

constexpr bool kBacklightActiveHigh =
#ifdef TFT_BACKLIGHT_ON
    (TFT_BACKLIGHT_ON == HIGH);
#else
    true;
#endif

static_assert(sizeof(lv_color_t) == 2, "LVGL must run in RGB565 (LV_COLOR_DEPTH=16).");
constexpr bool kLvglFlushSwapBytes =
#if LV_COLOR_16_SWAP
    false;
#else
    true;
#endif

enum class UiPageId : uint8_t {
  DASHBOARD = 0,
  TELEMETRY = 1,
  FLIGHT = 2,
  DIAGNOSTICS = 3,
  CALIBRATION = 4,
  SETTINGS = 5,
};

enum class UiFocusId : uint8_t {
  TAB_DASH = 0,
  TAB_TELEM,
  TAB_FLIGHT,
  TAB_DIAG,
  TAB_CAL,
  TAB_SETTINGS,
  TELEM_FILTER_ALL,
  TELEM_FILTER_LINK,
  TELEM_FILTER_GPS,
  TELEM_FILTER_WARN,
  FLIGHT_MODE_NEXT,
  FLIGHT_TAKEOFF,
  DIAG_CLEAR_CHART,
  CAL_SET_CENTER,
  CAL_SAVE_TEMPLATE,
  SETTINGS_PAGE_BACK,
  SETTINGS_HOME_BRIGHTNESS,
  SETTINGS_HOME_CTRL,
  SETTINGS_HOME_TELM,
  SETTINGS_BRIGHT_VALUE,
  SETTINGS_BRIGHT_SLEEP_ENABLE,
  SETTINGS_BRIGHT_SLEEP_TIMEOUT,
  SETTINGS_BRIGHT_APPLY,
  SETTINGS_BRIGHT_BACK,
  SETTINGS_CTRL_CHANNEL,
  SETTINGS_CTRL_DR,
  SETTINGS_CTRL_PA,
  SETTINGS_CTRL_ACK,
  SETTINGS_CTRL_RETRY_DELAY,
  SETTINGS_CTRL_RETRY_COUNT,
  SETTINGS_CTRL_APPLY,
  SETTINGS_CTRL_BACK,
  SETTINGS_TELM_CHANNEL,
  SETTINGS_TELM_DR,
  SETTINGS_TELM_PA,
  SETTINGS_TELM_ACK,
  SETTINGS_TELM_RETRY_DELAY,
  SETTINGS_TELM_RETRY_COUNT,
  SETTINGS_TELM_APPLY,
  SETTINGS_TELM_BACK,
};

enum class SettingsSubpage : uint8_t {
  HOME = 0,
  BRIGHTNESS = 1,
  NRF_CTRL = 2,
  NRF_TELM = 3,
};

enum class TelemKind : uint8_t {
  INFO = 0,
  LINK,
  GPS,
  WARN,
};

enum class TelemFilter : uint8_t {
  ALL = 0,
  LINK,
  GPS,
  WARN,
};

struct TelemetryEntry {
  TelemKind kind = TelemKind::INFO;
  char text[kTelemetryLineChars] = {};
};

struct TelemetryRing {
  TelemetryEntry entries[kTelemetryCapacity] = {};
  uint8_t head = 0;
  uint8_t count = 0;
};

struct UiContext {
  SharedState* state = nullptr;
  SemaphoreHandle_t mutex = nullptr;
  QueueHandle_t cmdq = nullptr;
  TaskHandle_t task = nullptr;
  bool running = false;
};

struct UiModel {
  UiPageId page = UiPageId::DASHBOARD;
  UiFocusId focus = UiFocusId::TAB_DASH;
  TelemFilter filter = TelemFilter::ALL;
  SettingsSubpage settings_subpage = SettingsSubpage::HOME;
  RadioRuntimeSettings radio_edit = {};
  bool radio_edit_loaded = false;
  bool radio_dirty = false;
  bool brightness_dirty = false;
  bool brightness_apply_ok = false;
  bool radio_apply_ok = false;
  bool sleep_enabled = true;
  uint32_t sleep_timeout_ms = 60000;
  bool sleeping = false;
  uint32_t last_user_activity_ms = 0;
  uint8_t awake_backlight_duty = 216;

  uint32_t lastStatePollMs = 0;
  uint32_t lastLowBattWarnMs = 0;
  uint32_t lastDiagSampleMs = 0;

  bool armHoldActive = false;
  bool armCommitSent = false;
  uint32_t armHoldStartMs = 0;

  char lastTelemLineSeen[96] = {};

  int8_t nav_axis_dir = 0;
  uint32_t nav_last_step_ms = 0;
  int8_t edit_axis_dir = 0;
  uint32_t edit_last_step_ms = 0;
  bool nav_btn_latched = false;
  int32_t diag_ctrl_ema = 0;
  int32_t diag_telm_ema = 0;
};

struct UiWidgets {
  lv_obj_t* topbar = nullptr;
  lv_obj_t* link_dot = nullptr;
  lv_obj_t* link_label = nullptr;
  lv_obj_t* title = nullptr;
  lv_obj_t* batt_icon = nullptr;
  lv_obj_t* batt_label = nullptr;

  lv_obj_t* nav_dash = nullptr;
  lv_obj_t* nav_telem = nullptr;
  lv_obj_t* nav_flight = nullptr;
  lv_obj_t* nav_diag = nullptr;
  lv_obj_t* nav_cal = nullptr;
  lv_obj_t* nav_settings = nullptr;

  lv_obj_t* content = nullptr;
  lv_obj_t* page_dashboard = nullptr;
  lv_obj_t* page_telemetry = nullptr;
  lv_obj_t* page_flight = nullptr;
  lv_obj_t* page_diagnostics = nullptr;
  lv_obj_t* page_calibration = nullptr;
  lv_obj_t* page_settings = nullptr;

  lv_obj_t* dash_battery = nullptr;
  lv_obj_t* dash_gps = nullptr;
  lv_obj_t* dash_altitude = nullptr;
  lv_obj_t* dash_distance = nullptr;
  lv_obj_t* dash_mode = nullptr;
  lv_obj_t* dash_link = nullptr;
  lv_obj_t* dash_fcu_card = nullptr;
  lv_obj_t* dash_fcu_status = nullptr;
  lv_obj_t* dash_prop_card = nullptr;
  lv_obj_t* dash_prop_spinner = nullptr;

  lv_obj_t* telem_filter_all = nullptr;
  lv_obj_t* telem_filter_link = nullptr;
  lv_obj_t* telem_filter_gps = nullptr;
  lv_obj_t* telem_filter_warn = nullptr;
  lv_obj_t* telem_log_container = nullptr;
  lv_obj_t* telem_log_label = nullptr;

  lv_obj_t* flight_status = nullptr;
  lv_obj_t* flight_combo = nullptr;
  lv_obj_t* flight_bar = nullptr;
  lv_obj_t* flight_progress = nullptr;
  lv_obj_t* flight_mode_next = nullptr;
  lv_obj_t* flight_takeoff = nullptr;

  lv_obj_t* diag_ctrl_state = nullptr;
  lv_obj_t* diag_telm_state = nullptr;
  lv_obj_t* diag_pkt_line = nullptr;
  lv_obj_t* diag_loss_line = nullptr;
  lv_obj_t* diag_heap_line = nullptr;
  lv_obj_t* diag_mode_line = nullptr;
  lv_obj_t* diag_axis_y = nullptr;
  lv_obj_t* diag_axis_x = nullptr;
  lv_obj_t* diag_chart = nullptr;
  lv_chart_series_t* diag_series_ctrl = nullptr;
  lv_chart_series_t* diag_series_telm = nullptr;
  lv_obj_t* diag_clear = nullptr;

  lv_obj_t* cal_lx_label = nullptr;
  lv_obj_t* cal_rx_label = nullptr;
  lv_obj_t* cal_ry_label = nullptr;
  lv_obj_t* cal_lx_bar = nullptr;
  lv_obj_t* cal_rx_bar = nullptr;
  lv_obj_t* cal_ry_bar = nullptr;
  lv_obj_t* cal_pid_label = nullptr;
  lv_obj_t* cal_set_center = nullptr;
  lv_obj_t* cal_save_template = nullptr;

  lv_obj_t* settings_page_back = nullptr;
  lv_obj_t* settings_list = nullptr;
  lv_obj_t* settings_home_brightness = nullptr;
  lv_obj_t* settings_home_ctrl = nullptr;
  lv_obj_t* settings_home_telm = nullptr;
  lv_obj_t* settings_sub_brightness = nullptr;
  lv_obj_t* settings_sub_ctrl = nullptr;
  lv_obj_t* settings_sub_telm = nullptr;
  lv_obj_t* settings_bright_value = nullptr;
  lv_obj_t* settings_bright_sleep_enable = nullptr;
  lv_obj_t* settings_bright_sleep_timeout = nullptr;
  lv_obj_t* settings_bright_apply = nullptr;
  lv_obj_t* settings_bright_back = nullptr;
  lv_obj_t* settings_ctrl_channel = nullptr;
  lv_obj_t* settings_ctrl_dr = nullptr;
  lv_obj_t* settings_ctrl_pa = nullptr;
  lv_obj_t* settings_ctrl_ack = nullptr;
  lv_obj_t* settings_ctrl_retry_delay = nullptr;
  lv_obj_t* settings_ctrl_retry_count = nullptr;
  lv_obj_t* settings_ctrl_apply = nullptr;
  lv_obj_t* settings_ctrl_back = nullptr;
  lv_obj_t* settings_telm_channel = nullptr;
  lv_obj_t* settings_telm_dr = nullptr;
  lv_obj_t* settings_telm_pa = nullptr;
  lv_obj_t* settings_telm_ack = nullptr;
  lv_obj_t* settings_telm_retry_delay = nullptr;
  lv_obj_t* settings_telm_retry_count = nullptr;
  lv_obj_t* settings_telm_apply = nullptr;
  lv_obj_t* settings_telm_back = nullptr;
  lv_obj_t* sleep_overlay = nullptr;
  lv_obj_t* sleep_label = nullptr;
};

UiContext gCtx = {};
UiModel gUi = {};
UiWidgets gW = {};
SharedState gLastState = {};
bool gLastStateValid = false;

TFT_eSPI gTft;
lv_disp_draw_buf_t gDrawBuf;
lv_disp_drv_t gDispDrv;
lv_color_t* gBuf1 = nullptr;
lv_color_t* gBuf2 = nullptr;

TelemetryRing gTelem = {};
char gTelemText[kTelemetryTextChars] = {};

lv_style_t gStyleCard;
lv_style_t gStyleBtn;
bool gStylesInit = false;
bool gBacklightPwmReady = false;

void refreshFocusVisuals();

lv_color_t col(uint32_t rgb) {
  return lv_color_hex(rgb);
}

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

const char* dataRateLabel(rf24_datarate_e rate) {
  switch (rate) {
    case RF24_250KBPS: return "250K";
    case RF24_2MBPS: return "2M";
    default: return "1M";
  }
}

const char* paLabel(uint8_t pa) {
  switch (pa) {
    case RF24_PA_MIN: return "MIN";
    case RF24_PA_LOW: return "LOW";
    case RF24_PA_HIGH: return "HIGH";
    default: return "MAX";
  }
}

rf24_datarate_e cycleDataRate(rf24_datarate_e current, int8_t dir) {
  const rf24_datarate_e order[3] = {RF24_250KBPS, RF24_1MBPS, RF24_2MBPS};
  int idx = 0;
  for (int i = 0; i < 3; ++i) {
    if (order[i] == current) {
      idx = i;
      break;
    }
  }
  idx += (dir >= 0) ? 1 : -1;
  if (idx < 0) {
    idx = 2;
  } else if (idx > 2) {
    idx = 0;
  }
  return order[idx];
}

uint8_t cyclePaLevel(uint8_t current, int8_t dir) {
  const uint8_t order[4] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
  int idx = 0;
  for (int i = 0; i < 4; ++i) {
    if (order[i] == current) {
      idx = i;
      break;
    }
  }
  idx += (dir >= 0) ? 1 : -1;
  if (idx < 0) {
    idx = 3;
  } else if (idx > 3) {
    idx = 0;
  }
  return order[idx];
}

const char* pageTitle(UiPageId page) {
  switch (page) {
    case UiPageId::DASHBOARD: return "Dashboard";
    case UiPageId::TELEMETRY: return "Telemetry";
    case UiPageId::FLIGHT: return "Flight";
    case UiPageId::DIAGNOSTICS: return "Diagnostics";
    case UiPageId::CALIBRATION: return "Calibration";
    case UiPageId::SETTINGS: return "Settings";
    default: return "UI";
  }
}

struct FocusItem {
  UiFocusId id;
  lv_obj_t* obj;
};

constexpr uint8_t kFocusListCapacity = 16;

UiFocusId tabFocusForPage(UiPageId page) {
  switch (page) {
    case UiPageId::DASHBOARD: return UiFocusId::TAB_DASH;
    case UiPageId::TELEMETRY: return UiFocusId::TAB_TELEM;
    case UiPageId::FLIGHT: return UiFocusId::TAB_FLIGHT;
    case UiPageId::DIAGNOSTICS: return UiFocusId::TAB_DIAG;
    case UiPageId::CALIBRATION: return UiFocusId::TAB_CAL;
    default: return UiFocusId::TAB_SETTINGS;
  }
}

UiPageId pageForTabFocus(UiFocusId focus) {
  switch (focus) {
    case UiFocusId::TAB_DASH: return UiPageId::DASHBOARD;
    case UiFocusId::TAB_TELEM: return UiPageId::TELEMETRY;
    case UiFocusId::TAB_FLIGHT: return UiPageId::FLIGHT;
    case UiFocusId::TAB_DIAG: return UiPageId::DIAGNOSTICS;
    case UiFocusId::TAB_CAL: return UiPageId::CALIBRATION;
    default: return UiPageId::SETTINGS;
  }
}

UiFocusId defaultFocusForPage(UiPageId page) {
  if (page == UiPageId::SETTINGS) {
    return UiFocusId::SETTINGS_PAGE_BACK;
  }
  return tabFocusForPage(page);
}

lv_obj_t* objectForFocus(UiFocusId focus) {
  switch (focus) {
    case UiFocusId::TAB_DASH: return gW.nav_dash;
    case UiFocusId::TAB_TELEM: return gW.nav_telem;
    case UiFocusId::TAB_FLIGHT: return gW.nav_flight;
    case UiFocusId::TAB_DIAG: return gW.nav_diag;
    case UiFocusId::TAB_CAL: return gW.nav_cal;
    case UiFocusId::TAB_SETTINGS: return gW.nav_settings;
    case UiFocusId::TELEM_FILTER_ALL: return gW.telem_filter_all;
    case UiFocusId::TELEM_FILTER_LINK: return gW.telem_filter_link;
    case UiFocusId::TELEM_FILTER_GPS: return gW.telem_filter_gps;
    case UiFocusId::TELEM_FILTER_WARN: return gW.telem_filter_warn;
    case UiFocusId::FLIGHT_MODE_NEXT: return gW.flight_mode_next;
    case UiFocusId::FLIGHT_TAKEOFF: return gW.flight_takeoff;
    case UiFocusId::DIAG_CLEAR_CHART: return gW.diag_clear;
    case UiFocusId::CAL_SET_CENTER: return gW.cal_set_center;
    case UiFocusId::CAL_SAVE_TEMPLATE: return gW.cal_save_template;
    case UiFocusId::SETTINGS_PAGE_BACK: return gW.settings_page_back;
    case UiFocusId::SETTINGS_HOME_BRIGHTNESS: return gW.settings_home_brightness;
    case UiFocusId::SETTINGS_HOME_CTRL: return gW.settings_home_ctrl;
    case UiFocusId::SETTINGS_HOME_TELM: return gW.settings_home_telm;
    case UiFocusId::SETTINGS_BRIGHT_VALUE: return gW.settings_bright_value;
    case UiFocusId::SETTINGS_BRIGHT_SLEEP_ENABLE: return gW.settings_bright_sleep_enable;
    case UiFocusId::SETTINGS_BRIGHT_SLEEP_TIMEOUT: return gW.settings_bright_sleep_timeout;
    case UiFocusId::SETTINGS_BRIGHT_APPLY: return gW.settings_bright_apply;
    case UiFocusId::SETTINGS_BRIGHT_BACK: return gW.settings_bright_back;
    case UiFocusId::SETTINGS_CTRL_CHANNEL: return gW.settings_ctrl_channel;
    case UiFocusId::SETTINGS_CTRL_DR: return gW.settings_ctrl_dr;
    case UiFocusId::SETTINGS_CTRL_PA: return gW.settings_ctrl_pa;
    case UiFocusId::SETTINGS_CTRL_ACK: return gW.settings_ctrl_ack;
    case UiFocusId::SETTINGS_CTRL_RETRY_DELAY: return gW.settings_ctrl_retry_delay;
    case UiFocusId::SETTINGS_CTRL_RETRY_COUNT: return gW.settings_ctrl_retry_count;
    case UiFocusId::SETTINGS_CTRL_APPLY: return gW.settings_ctrl_apply;
    case UiFocusId::SETTINGS_CTRL_BACK: return gW.settings_ctrl_back;
    case UiFocusId::SETTINGS_TELM_CHANNEL: return gW.settings_telm_channel;
    case UiFocusId::SETTINGS_TELM_DR: return gW.settings_telm_dr;
    case UiFocusId::SETTINGS_TELM_PA: return gW.settings_telm_pa;
    case UiFocusId::SETTINGS_TELM_ACK: return gW.settings_telm_ack;
    case UiFocusId::SETTINGS_TELM_RETRY_DELAY: return gW.settings_telm_retry_delay;
    case UiFocusId::SETTINGS_TELM_RETRY_COUNT: return gW.settings_telm_retry_count;
    case UiFocusId::SETTINGS_TELM_APPLY: return gW.settings_telm_apply;
    default: return gW.settings_telm_back;
  }
}

uint8_t buildFocusList(FocusItem* out, uint8_t cap) {
  if ((out == nullptr) || (cap == 0U)) {
    return 0U;
  }

  uint8_t n = 0U;
  auto push = [&](UiFocusId id) {
    if (n >= cap) {
      return;
    }
    out[n].id = id;
    out[n].obj = objectForFocus(id);
    ++n;
  };

  if (gUi.page != UiPageId::SETTINGS) {
    push(UiFocusId::TAB_DASH);
    push(UiFocusId::TAB_TELEM);
    push(UiFocusId::TAB_FLIGHT);
    push(UiFocusId::TAB_DIAG);
    push(UiFocusId::TAB_CAL);
    push(UiFocusId::TAB_SETTINGS);
  }

  switch (gUi.page) {
    case UiPageId::DASHBOARD:
      break;
    case UiPageId::TELEMETRY:
      push(UiFocusId::TELEM_FILTER_ALL);
      push(UiFocusId::TELEM_FILTER_LINK);
      push(UiFocusId::TELEM_FILTER_GPS);
      push(UiFocusId::TELEM_FILTER_WARN);
      break;
    case UiPageId::FLIGHT:
      push(UiFocusId::FLIGHT_MODE_NEXT);
      push(UiFocusId::FLIGHT_TAKEOFF);
      break;
    case UiPageId::DIAGNOSTICS:
      push(UiFocusId::DIAG_CLEAR_CHART);
      break;
    case UiPageId::CALIBRATION:
      push(UiFocusId::CAL_SET_CENTER);
      push(UiFocusId::CAL_SAVE_TEMPLATE);
      break;
    case UiPageId::SETTINGS:
      push(UiFocusId::SETTINGS_PAGE_BACK);
      if (gUi.settings_subpage == SettingsSubpage::HOME) {
        push(UiFocusId::SETTINGS_HOME_BRIGHTNESS);
        push(UiFocusId::SETTINGS_HOME_CTRL);
        push(UiFocusId::SETTINGS_HOME_TELM);
      } else if (gUi.settings_subpage == SettingsSubpage::BRIGHTNESS) {
        push(UiFocusId::SETTINGS_BRIGHT_VALUE);
        push(UiFocusId::SETTINGS_BRIGHT_SLEEP_ENABLE);
        push(UiFocusId::SETTINGS_BRIGHT_SLEEP_TIMEOUT);
        push(UiFocusId::SETTINGS_BRIGHT_APPLY);
        push(UiFocusId::SETTINGS_BRIGHT_BACK);
      } else if (gUi.settings_subpage == SettingsSubpage::NRF_CTRL) {
        push(UiFocusId::SETTINGS_CTRL_CHANNEL);
        push(UiFocusId::SETTINGS_CTRL_DR);
        push(UiFocusId::SETTINGS_CTRL_PA);
        push(UiFocusId::SETTINGS_CTRL_ACK);
        push(UiFocusId::SETTINGS_CTRL_RETRY_DELAY);
        push(UiFocusId::SETTINGS_CTRL_RETRY_COUNT);
        push(UiFocusId::SETTINGS_CTRL_APPLY);
        push(UiFocusId::SETTINGS_CTRL_BACK);
      } else {
        push(UiFocusId::SETTINGS_TELM_CHANNEL);
        push(UiFocusId::SETTINGS_TELM_DR);
        push(UiFocusId::SETTINGS_TELM_PA);
        push(UiFocusId::SETTINGS_TELM_ACK);
        push(UiFocusId::SETTINGS_TELM_RETRY_DELAY);
        push(UiFocusId::SETTINGS_TELM_RETRY_COUNT);
        push(UiFocusId::SETTINGS_TELM_APPLY);
        push(UiFocusId::SETTINGS_TELM_BACK);
      }
      break;
  }

  return n;
}

bool focusIsInCurrentList(UiFocusId focus) {
  FocusItem list[kFocusListCapacity] = {};
  const uint8_t count = buildFocusList(list, kFocusListCapacity);
  for (uint8_t i = 0; i < count; ++i) {
    if (list[i].id == focus) {
      return true;
    }
  }
  return false;
}

const char* kindTag(TelemKind kind) {
  switch (kind) {
    case TelemKind::LINK: return "LINK";
    case TelemKind::GPS: return "GPS";
    case TelemKind::WARN: return "WARN";
    default: return "INFO";
  }
}

bool filterMatch(TelemFilter filter, TelemKind kind) {
  if (filter == TelemFilter::ALL) {
    return true;
  }
  if (filter == TelemFilter::LINK) {
    return kind == TelemKind::LINK;
  }
  if (filter == TelemFilter::GPS) {
    return kind == TelemKind::GPS;
  }
  return kind == TelemKind::WARN;
}

TelemKind detectKind(const char* line) {
  if (line == nullptr) {
    return TelemKind::INFO;
  }
  if (strstr(line, "WARN") != nullptr) {
    return TelemKind::WARN;
  }
  if (strstr(line, "GPS") != nullptr) {
    return TelemKind::GPS;
  }
  if (strstr(line, "LINK") != nullptr) {
    return TelemKind::LINK;
  }
  return TelemKind::INFO;
}

bool isArmingCombo(const SharedState& s) {
  // "Both sticks down": left vertical (mapped as left_x here) + dominant right vertical axis.
  const int16_t right_vertical =
      (abs(static_cast<int>(s.joystick.right_y_mapped)) >= abs(static_cast<int>(s.joystick.right_x_mapped)))
          ? s.joystick.right_y_mapped
          : s.joystick.right_x_mapped;

  const bool both_negative = (s.joystick.left_x_mapped <= -260) && (right_vertical <= -260);
  const bool both_positive = (s.joystick.left_x_mapped >= 260) && (right_vertical >= 260);
  return both_negative || both_positive;
}

bool fetchState(SharedState* out) {
  if ((out == nullptr) || (gCtx.state == nullptr) || (gCtx.mutex == nullptr)) {
    return false;
  }
  if (xSemaphoreTake(gCtx.mutex, pdMS_TO_TICKS(3)) != pdTRUE) {
    return false;
  }
  *out = *gCtx.state;
  xSemaphoreGive(gCtx.mutex);
  return true;
}

void sendUiCommand(const UiCommand& cmd) {
  if (gCtx.cmdq == nullptr) {
    return;
  }
  (void)xQueueSend(gCtx.cmdq, &cmd, 0);
}

void queueBrightnessDuty(uint8_t duty) {
  UiCommand cmd = {};
  cmd.type = UI_CMD_SET_BRIGHTNESS;
  cmd.backlight_duty = duty;
  cmd.brightness_pct = percentFromDuty(duty);
  sendUiCommand(cmd);
}

void queueArmedState(ArmedState state) {
  UiCommand cmd = {};
  cmd.type = UI_CMD_SET_ARMED_STATE;
  cmd.armed_state = state;
  sendUiCommand(cmd);
}

void queueModeIndex(uint8_t mode_index) {
  UiCommand cmd = {};
  cmd.type = UI_CMD_SET_MODE_INDEX;
  cmd.mode_index = normalizeModeIndex(mode_index);
  sendUiCommand(cmd);
}

void queueApplyRadioSettings(const RadioRuntimeSettings& radio) {
  UiCommand cmd = {};
  cmd.type = UI_CMD_APPLY_RADIO_SETTINGS;
  cmd.radio = radio;
  sendUiCommand(cmd);
}

void initBacklightPwm() {
  if (gBacklightPwmReady) {
    return;
  }
  pinMode(PIN_TFT_BL, OUTPUT);
  ledcSetup(UI_BACKLIGHT_PWM_CHANNEL, UI_BACKLIGHT_PWM_FREQ_HZ, UI_BACKLIGHT_PWM_BITS);
  ledcAttachPin(PIN_TFT_BL, UI_BACKLIGHT_PWM_CHANNEL);
  gBacklightPwmReady = true;
}

void setBacklightDuty(uint8_t duty) {
  initBacklightPwm();
  const uint8_t clamped = clampU8(duty, 0, kBacklightDutyMax);
  const uint8_t hw_duty = kBacklightActiveHigh ? clamped : static_cast<uint8_t>(kBacklightDutyMax - clamped);
  ledcWrite(UI_BACKLIGHT_PWM_CHANNEL, hw_duty);
}

void backlightOffEarly() {
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, kBacklightActiveHigh ? LOW : HIGH);
}

void applyLandscapeFlipWorkaroundIfNeeded() {
#if defined(ST7789_DRIVER)
  if (TFT_FORCE_LANDSCAPE_FLIP_WITH_ROT1 && (TFT_ROTATION == 1)) {
    const uint8_t madctl = static_cast<uint8_t>(TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);
    gTft.writecommand(TFT_MADCTL);
    gTft.writedata(madctl);
  }
#endif
}

void applySt7789PanelFormatAndInversion() {
#if defined(ST7789_DRIVER)
  gTft.writecommand(TFT_COLMOD);
  gTft.writedata(0x55);  // RGB565
  gTft.writecommand(UI_TFT_INVERT_COLORS ? TFT_INVON : TFT_INVOFF);
#endif
}

void displayBeginRobust() {
  backlightOffEarly();
  delay(kTftPowerSettleDelayMs);

  for (uint8_t pass = 0; pass < kTftInitPasses; ++pass) {
    gTft.init();
    gTft.setRotation(TFT_ROTATION);
    applyLandscapeFlipWorkaroundIfNeeded();
    applySt7789PanelFormatAndInversion();
    gTft.fillScreen(TFT_BLACK);
    delay(kTftRetryGapDelayMs);
  }
}

void lvFlush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  const uint32_t w = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  const uint32_t h = static_cast<uint32_t>(area->y2 - area->y1 + 1);

  gTft.startWrite();
  gTft.setAddrWindow(area->x1, area->y1, w, h);
  gTft.pushColors(reinterpret_cast<uint16_t*>(color_p), w * h, kLvglFlushSwapBytes);
  gTft.endWrite();

  lv_disp_flush_ready(disp);
}

void initLvgl() {
  lv_init();

  const size_t px = static_cast<size_t>(kScreenW) * kLvBufferLines;
  const size_t bytes = px * sizeof(lv_color_t);

  gBuf1 = static_cast<lv_color_t*>(heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  gBuf2 = static_cast<lv_color_t*>(heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (gBuf1 == nullptr) {
    gBuf1 = static_cast<lv_color_t*>(malloc(bytes));
  }
  if (gBuf1 == nullptr) {
    LOG_ERR("LVGL buffer allocation failed");
    while (true) {
      delay(1000);
    }
  }

  lv_disp_draw_buf_init(&gDrawBuf, gBuf1, gBuf2, px);
  lv_disp_drv_init(&gDispDrv);
  gDispDrv.hor_res = kScreenW;
  gDispDrv.ver_res = kScreenH;
  gDispDrv.draw_buf = &gDrawBuf;
  gDispDrv.flush_cb = lvFlush;
  (void)lv_disp_drv_register(&gDispDrv);
}

void initStyles() {
  if (gStylesInit) {
    return;
  }

  lv_style_init(&gStyleCard);
  lv_style_set_bg_color(&gStyleCard, col(C_CARD));
  lv_style_set_bg_opa(&gStyleCard, LV_OPA_COVER);
  lv_style_set_border_color(&gStyleCard, col(C_BORDER));
  lv_style_set_border_width(&gStyleCard, 1);
  lv_style_set_radius(&gStyleCard, 8);
  lv_style_set_pad_all(&gStyleCard, 5);

  lv_style_init(&gStyleBtn);
  lv_style_set_bg_color(&gStyleBtn, col(C_CARD));
  lv_style_set_bg_opa(&gStyleBtn, LV_OPA_COVER);
  lv_style_set_border_color(&gStyleBtn, col(C_BORDER));
  lv_style_set_border_width(&gStyleBtn, 1);
  lv_style_set_radius(&gStyleBtn, 6);
  lv_style_set_text_color(&gStyleBtn, col(C_TEXT));

  gStylesInit = true;
}

void telemetryPush(TelemKind kind, const char* text) {
  if ((text == nullptr) || (text[0] == '\0')) {
    return;
  }

  TelemetryEntry& slot = gTelem.entries[gTelem.head];
  slot.kind = kind;
  snprintf(slot.text, sizeof(slot.text), "%s", text);

  gTelem.head = static_cast<uint8_t>((gTelem.head + 1U) % kTelemetryCapacity);
  if (gTelem.count < kTelemetryCapacity) {
    ++gTelem.count;
  }
}

void refreshDashboardPreview() {
  // Dashboard no longer has telemetry preview rows; keep hook for compatibility.
}

void refreshTelemetryLog() {
  if (gW.telem_log_label == nullptr) {
    return;
  }

  gTelemText[0] = '\0';
  if (gTelem.count == 0U) {
    lv_label_set_text(gW.telem_log_label, "Telemetry stream is empty.");
    return;
  }

  uint8_t selected[kTelemetryCapacity] = {};
  uint8_t selected_count = 0;

  for (uint8_t i = 0; i < gTelem.count; ++i) {
    int idx = static_cast<int>(gTelem.head) - 1 - static_cast<int>(i);
    while (idx < 0) {
      idx += kTelemetryCapacity;
    }

    const TelemetryEntry& e = gTelem.entries[static_cast<uint8_t>(idx)];
    if (!filterMatch(gUi.filter, e.kind)) {
      continue;
    }

    selected[selected_count++] = static_cast<uint8_t>(idx);
    if (selected_count >= kTelemetryVisibleMax) {
      break;
    }
  }

  if (selected_count == 0U) {
    lv_label_set_text(gW.telem_log_label, "No lines for this filter.");
    return;
  }

  size_t pos = 0;
  for (int i = static_cast<int>(selected_count) - 1; i >= 0; --i) {
    const TelemetryEntry& e = gTelem.entries[selected[i]];
    const int written = snprintf(gTelemText + pos,
                                 sizeof(gTelemText) - pos,
                                 "[%s] %s\n",
                                 kindTag(e.kind),
                                 e.text);
    if (written <= 0) {
      break;
    }

    const size_t w = static_cast<size_t>(written);
    if (w >= (sizeof(gTelemText) - pos)) {
      pos = sizeof(gTelemText) - 1U;
      break;
    }
    pos += w;
  }

  lv_label_set_text(gW.telem_log_label, gTelemText);
  lv_obj_scroll_to_y(gW.telem_log_container, LV_COORD_MAX, LV_ANIM_OFF);
}

void updateFilterButtonsVisual() {
  const lv_color_t active = col(C_CARD_ACTIVE);
  const lv_color_t idle = col(C_CARD);

  lv_obj_set_style_bg_color(gW.telem_filter_all, gUi.filter == TelemFilter::ALL ? active : idle, 0);
  lv_obj_set_style_bg_color(gW.telem_filter_link, gUi.filter == TelemFilter::LINK ? active : idle, 0);
  lv_obj_set_style_bg_color(gW.telem_filter_gps, gUi.filter == TelemFilter::GPS ? active : idle, 0);
  lv_obj_set_style_bg_color(gW.telem_filter_warn, gUi.filter == TelemFilter::WARN ? active : idle, 0);
}

void refreshSettingsEditorLabels() {
  if (gW.settings_bright_value != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "Brightness: %u%%", static_cast<unsigned>(percentFromDuty(gUi.awake_backlight_duty)));
    lv_label_set_text(lv_obj_get_child(gW.settings_bright_value, 0), t);
  }
  if (gW.settings_bright_sleep_enable != nullptr) {
    lv_label_set_text(lv_obj_get_child(gW.settings_bright_sleep_enable, 0),
                      gUi.sleep_enabled ? "Sleep: ENABLED" : "Sleep: DISABLED");
  }
  if (gW.settings_bright_sleep_timeout != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "Sleep Timeout: %lus", static_cast<unsigned long>(gUi.sleep_timeout_ms / 1000UL));
    lv_label_set_text(lv_obj_get_child(gW.settings_bright_sleep_timeout, 0), t);
  }
  if (gW.settings_bright_apply != nullptr) {
    lv_label_set_text(lv_obj_get_child(gW.settings_bright_apply, 0), "Apply Changes");
    lv_obj_set_style_text_color(gW.settings_bright_apply, gUi.brightness_apply_ok ? col(C_GREEN) : col(C_TEXT), 0);
  }

  if (!gUi.radio_edit_loaded) {
    return;
  }

  if (gW.settings_ctrl_channel != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "CTRL CH: %u", static_cast<unsigned>(gUi.radio_edit.ctrl_channel));
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_channel, 0), t);
  }
  if (gW.settings_ctrl_dr != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "CTRL DR: %s", dataRateLabel(gUi.radio_edit.ctrl_data_rate));
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_dr, 0), t);
  }
  if (gW.settings_ctrl_pa != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "CTRL PA: %s", paLabel(gUi.radio_edit.ctrl_pa_level));
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_pa, 0), t);
  }
  if (gW.settings_ctrl_ack != nullptr) {
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_ack, 0),
                      gUi.radio_edit.ctrl_auto_ack ? "CTRL ACK: ON" : "CTRL ACK: OFF");
  }
  if (gW.settings_ctrl_retry_delay != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "CTRL Retry Delay: %u", static_cast<unsigned>(gUi.radio_edit.ctrl_retry_delay));
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_retry_delay, 0), t);
  }
  if (gW.settings_ctrl_retry_count != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "CTRL Retry Count: %u", static_cast<unsigned>(gUi.radio_edit.ctrl_retry_count));
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_retry_count, 0), t);
  }

  if (gW.settings_telm_channel != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "TELM CH: %u", static_cast<unsigned>(gUi.radio_edit.telm_channel));
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_channel, 0), t);
  }
  if (gW.settings_telm_dr != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "TELM DR: %s", dataRateLabel(gUi.radio_edit.telm_data_rate));
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_dr, 0), t);
  }
  if (gW.settings_telm_pa != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "TELM PA: %s", paLabel(gUi.radio_edit.telm_pa_level));
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_pa, 0), t);
  }
  if (gW.settings_telm_ack != nullptr) {
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_ack, 0),
                      gUi.radio_edit.telm_auto_ack ? "TELM ACK: ON" : "TELM ACK: OFF");
  }
  if (gW.settings_telm_retry_delay != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "TELM Retry Delay: %u", static_cast<unsigned>(gUi.radio_edit.telm_retry_delay));
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_retry_delay, 0), t);
  }
  if (gW.settings_telm_retry_count != nullptr) {
    char t[48];
    snprintf(t, sizeof(t), "TELM Retry Count: %u", static_cast<unsigned>(gUi.radio_edit.telm_retry_count));
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_retry_count, 0), t);
  }
  if (gW.settings_ctrl_apply != nullptr) {
    lv_label_set_text(lv_obj_get_child(gW.settings_ctrl_apply, 0), "Apply Changes");
    lv_obj_set_style_text_color(gW.settings_ctrl_apply, gUi.radio_apply_ok ? col(C_GREEN) : col(C_TEXT), 0);
  }
  if (gW.settings_telm_apply != nullptr) {
    lv_label_set_text(lv_obj_get_child(gW.settings_telm_apply, 0), "Apply Changes");
    lv_obj_set_style_text_color(gW.settings_telm_apply, gUi.radio_apply_ok ? col(C_GREEN) : col(C_TEXT), 0);
  }
}

void showSettingsSubpage(SettingsSubpage sub) {
  gUi.settings_subpage = sub;
  if (gW.settings_list != nullptr) {
    if (sub == SettingsSubpage::HOME) {
      lv_obj_clear_flag(gW.settings_list, LV_OBJ_FLAG_HIDDEN);
      lv_obj_scroll_to_y(gW.settings_list, 0, LV_ANIM_OFF);
    } else {
      lv_obj_add_flag(gW.settings_list, LV_OBJ_FLAG_HIDDEN);
    }
  }
  if (gW.settings_sub_brightness != nullptr) {
    if (sub == SettingsSubpage::BRIGHTNESS) {
      lv_obj_clear_flag(gW.settings_sub_brightness, LV_OBJ_FLAG_HIDDEN);
      lv_obj_scroll_to_y(gW.settings_sub_brightness, 0, LV_ANIM_OFF);
    } else {
      lv_obj_add_flag(gW.settings_sub_brightness, LV_OBJ_FLAG_HIDDEN);
    }
  }
  if (gW.settings_sub_ctrl != nullptr) {
    if (sub == SettingsSubpage::NRF_CTRL) {
      lv_obj_clear_flag(gW.settings_sub_ctrl, LV_OBJ_FLAG_HIDDEN);
      lv_obj_scroll_to_y(gW.settings_sub_ctrl, 0, LV_ANIM_OFF);
    } else {
      lv_obj_add_flag(gW.settings_sub_ctrl, LV_OBJ_FLAG_HIDDEN);
    }
  }
  if (gW.settings_sub_telm != nullptr) {
    if (sub == SettingsSubpage::NRF_TELM) {
      lv_obj_clear_flag(gW.settings_sub_telm, LV_OBJ_FLAG_HIDDEN);
      lv_obj_scroll_to_y(gW.settings_sub_telm, 0, LV_ANIM_OFF);
    } else {
      lv_obj_add_flag(gW.settings_sub_telm, LV_OBJ_FLAG_HIDDEN);
    }
  }
  refreshSettingsEditorLabels();
}

void scrollFocusIntoView() {
  lv_obj_t* focused = objectForFocus(gUi.focus);
  if (focused == nullptr) {
    return;
  }
  lv_obj_scroll_to_view_recursive(focused, LV_ANIM_OFF);
}

void markUserActivity(uint32_t now_ms) {
  gUi.last_user_activity_ms = now_ms;
}

void showSleepOverlay(bool show) {
  if (gW.sleep_overlay == nullptr) {
    return;
  }
  if (show) {
    lv_obj_clear_flag(gW.sleep_overlay, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(gW.sleep_overlay, LV_OBJ_FLAG_HIDDEN);
  }
}

void enterSleepMode(uint32_t now_ms) {
  if (gUi.sleeping || !gUi.sleep_enabled) {
    return;
  }
  gUi.sleeping = true;
  markUserActivity(now_ms);
  showSleepOverlay(true);
  setBacklightDuty(10);
}

void wakeFromSleep(uint32_t now_ms) {
  if (!gUi.sleeping) {
    return;
  }
  gUi.sleeping = false;
  markUserActivity(now_ms);
  showSleepOverlay(false);
  setBacklightDuty(gUi.awake_backlight_duty);
}

bool hasWakeInput(const SharedState& s) {
  const int16_t lx = s.joystick.left_x_mapped;
  const int16_t rx = s.joystick.right_x_mapped;
  const int16_t ry = s.joystick.right_y_mapped;
  return s.joystick.left_button_pressed ||
         (lx >= kWakeAxisThreshold) || (lx <= -kWakeAxisThreshold) ||
         (rx >= kWakeAxisThreshold) || (rx <= -kWakeAxisThreshold) ||
         (ry >= kWakeAxisThreshold) || (ry <= -kWakeAxisThreshold);
}

void queueCurrentBrightness() {
  queueBrightnessDuty(gUi.awake_backlight_duty);
  gUi.brightness_dirty = false;
  gUi.brightness_apply_ok = true;
  refreshSettingsEditorLabels();
  telemetryPush(TelemKind::INFO, "Brightness settings applied");
  refreshTelemetryLog();
}

void queueCurrentRadioSettings() {
  if (!gUi.radio_edit_loaded) {
    return;
  }
  queueApplyRadioSettings(gUi.radio_edit);
  gUi.radio_dirty = false;
  gUi.radio_apply_ok = true;
  refreshSettingsEditorLabels();
  telemetryPush(TelemKind::INFO, "NRF settings apply queued");
  refreshTelemetryLog();
}

void stepFocusTo(UiFocusId id) {
  gUi.focus = id;
  scrollFocusIntoView();
  refreshFocusVisuals();
}

void stepSettingValue(UiFocusId focus, int8_t dir) {
  if (dir == 0) {
    return;
  }

  bool changed = false;
  switch (focus) {
    case UiFocusId::SETTINGS_BRIGHT_VALUE: {
      int next = static_cast<int>(gUi.awake_backlight_duty) + ((dir > 0) ? 6 : -6);
      if (next < 8) {
        next = 8;
      } else if (next > static_cast<int>(kBacklightDutyMax)) {
        next = static_cast<int>(kBacklightDutyMax);
      }
      const uint8_t duty = static_cast<uint8_t>(next);
      if (duty != gUi.awake_backlight_duty) {
        gUi.awake_backlight_duty = duty;
        gUi.brightness_dirty = true;
        gUi.brightness_apply_ok = false;
        changed = true;
        if (!gUi.sleeping) {
          setBacklightDuty(duty);
        }
      }
      break;
    }
    case UiFocusId::SETTINGS_BRIGHT_SLEEP_ENABLE:
      gUi.sleep_enabled = !gUi.sleep_enabled;
      gUi.brightness_dirty = true;
      gUi.brightness_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_BRIGHT_SLEEP_TIMEOUT: {
      int32_t timeout = static_cast<int32_t>(gUi.sleep_timeout_ms);
      timeout += (dir > 0) ? 15000 : -15000;
      if (timeout < 15000) {
        timeout = 300000;
      } else if (timeout > 300000) {
        timeout = 15000;
      }
      if (static_cast<uint32_t>(timeout) != gUi.sleep_timeout_ms) {
        gUi.sleep_timeout_ms = static_cast<uint32_t>(timeout);
        gUi.brightness_dirty = true;
        gUi.brightness_apply_ok = false;
        changed = true;
      }
      break;
    }
    case UiFocusId::SETTINGS_CTRL_CHANNEL: {
      int next = static_cast<int>(gUi.radio_edit.ctrl_channel) + ((dir > 0) ? 1 : -1);
      if (next < 0) {
        next = 125;
      } else if (next > 125) {
        next = 0;
      }
      gUi.radio_edit.ctrl_channel = static_cast<uint8_t>(next);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    }
    case UiFocusId::SETTINGS_CTRL_DR:
      gUi.radio_edit.ctrl_data_rate = cycleDataRate(gUi.radio_edit.ctrl_data_rate, dir);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_CTRL_PA:
      gUi.radio_edit.ctrl_pa_level = cyclePaLevel(gUi.radio_edit.ctrl_pa_level, dir);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_CTRL_ACK:
      gUi.radio_edit.ctrl_auto_ack = !gUi.radio_edit.ctrl_auto_ack;
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_CTRL_RETRY_DELAY: {
      int next = static_cast<int>(gUi.radio_edit.ctrl_retry_delay) + ((dir > 0) ? 1 : -1);
      if (next < 0) {
        next = 15;
      } else if (next > 15) {
        next = 0;
      }
      gUi.radio_edit.ctrl_retry_delay = static_cast<uint8_t>(next);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    }
    case UiFocusId::SETTINGS_CTRL_RETRY_COUNT: {
      int next = static_cast<int>(gUi.radio_edit.ctrl_retry_count) + ((dir > 0) ? 1 : -1);
      if (next < 0) {
        next = 15;
      } else if (next > 15) {
        next = 0;
      }
      gUi.radio_edit.ctrl_retry_count = static_cast<uint8_t>(next);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    }
    case UiFocusId::SETTINGS_TELM_CHANNEL: {
      int next = static_cast<int>(gUi.radio_edit.telm_channel) + ((dir > 0) ? 1 : -1);
      if (next < 0) {
        next = 125;
      } else if (next > 125) {
        next = 0;
      }
      gUi.radio_edit.telm_channel = static_cast<uint8_t>(next);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    }
    case UiFocusId::SETTINGS_TELM_DR:
      gUi.radio_edit.telm_data_rate = cycleDataRate(gUi.radio_edit.telm_data_rate, dir);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_TELM_PA:
      gUi.radio_edit.telm_pa_level = cyclePaLevel(gUi.radio_edit.telm_pa_level, dir);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_TELM_ACK:
      gUi.radio_edit.telm_auto_ack = !gUi.radio_edit.telm_auto_ack;
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    case UiFocusId::SETTINGS_TELM_RETRY_DELAY: {
      int next = static_cast<int>(gUi.radio_edit.telm_retry_delay) + ((dir > 0) ? 1 : -1);
      if (next < 0) {
        next = 15;
      } else if (next > 15) {
        next = 0;
      }
      gUi.radio_edit.telm_retry_delay = static_cast<uint8_t>(next);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    }
    case UiFocusId::SETTINGS_TELM_RETRY_COUNT: {
      int next = static_cast<int>(gUi.radio_edit.telm_retry_count) + ((dir > 0) ? 1 : -1);
      if (next < 0) {
        next = 15;
      } else if (next > 15) {
        next = 0;
      }
      gUi.radio_edit.telm_retry_count = static_cast<uint8_t>(next);
      gUi.radio_dirty = true;
      gUi.radio_apply_ok = false;
      changed = true;
      break;
    }
    default:
      break;
  }

  if (changed) {
    refreshSettingsEditorLabels();
    refreshFocusVisuals();
  }
}

bool focusSupportsValueStep(UiFocusId focus) {
  switch (focus) {
    case UiFocusId::SETTINGS_BRIGHT_VALUE:
    case UiFocusId::SETTINGS_BRIGHT_SLEEP_ENABLE:
    case UiFocusId::SETTINGS_BRIGHT_SLEEP_TIMEOUT:
    case UiFocusId::SETTINGS_CTRL_CHANNEL:
    case UiFocusId::SETTINGS_CTRL_DR:
    case UiFocusId::SETTINGS_CTRL_PA:
    case UiFocusId::SETTINGS_CTRL_ACK:
    case UiFocusId::SETTINGS_CTRL_RETRY_DELAY:
    case UiFocusId::SETTINGS_CTRL_RETRY_COUNT:
    case UiFocusId::SETTINGS_TELM_CHANNEL:
    case UiFocusId::SETTINGS_TELM_DR:
    case UiFocusId::SETTINGS_TELM_PA:
    case UiFocusId::SETTINGS_TELM_ACK:
    case UiFocusId::SETTINGS_TELM_RETRY_DELAY:
    case UiFocusId::SETTINGS_TELM_RETRY_COUNT:
      return true;
    default:
      return false;
  }
}

void applyFocusBorder(lv_obj_t* obj, bool focused) {
  if (obj == nullptr) {
    return;
  }
  lv_obj_set_style_border_color(obj, focused ? col(C_TEXT) : col(C_BORDER), 0);
  lv_obj_set_style_border_width(obj, focused ? 2 : 1, 0);
}

void applyNavVisual(UiPageId page, lv_obj_t* obj) {
  if (obj == nullptr) {
    return;
  }
  const bool active = (gUi.page == page);
  const bool highlighted = (gUi.focus == tabFocusForPage(page));
  lv_obj_set_style_bg_color(obj, active ? col(C_CARD_ACTIVE) : col(C_CARD), 0);
  applyFocusBorder(obj, highlighted);
}

void refreshFocusVisuals() {
  applyNavVisual(UiPageId::DASHBOARD, gW.nav_dash);
  applyNavVisual(UiPageId::TELEMETRY, gW.nav_telem);
  applyNavVisual(UiPageId::FLIGHT, gW.nav_flight);
  applyNavVisual(UiPageId::DIAGNOSTICS, gW.nav_diag);
  applyNavVisual(UiPageId::CALIBRATION, gW.nav_cal);
  applyNavVisual(UiPageId::SETTINGS, gW.nav_settings);

  updateFilterButtonsVisual();
  applyFocusBorder(gW.telem_filter_all, gUi.focus == UiFocusId::TELEM_FILTER_ALL);
  applyFocusBorder(gW.telem_filter_link, gUi.focus == UiFocusId::TELEM_FILTER_LINK);
  applyFocusBorder(gW.telem_filter_gps, gUi.focus == UiFocusId::TELEM_FILTER_GPS);
  applyFocusBorder(gW.telem_filter_warn, gUi.focus == UiFocusId::TELEM_FILTER_WARN);
  applyFocusBorder(gW.flight_mode_next, gUi.focus == UiFocusId::FLIGHT_MODE_NEXT);
  applyFocusBorder(gW.flight_takeoff, gUi.focus == UiFocusId::FLIGHT_TAKEOFF);
  applyFocusBorder(gW.diag_clear, gUi.focus == UiFocusId::DIAG_CLEAR_CHART);
  applyFocusBorder(gW.cal_set_center, gUi.focus == UiFocusId::CAL_SET_CENTER);
  applyFocusBorder(gW.cal_save_template, gUi.focus == UiFocusId::CAL_SAVE_TEMPLATE);
  applyFocusBorder(gW.settings_page_back, gUi.focus == UiFocusId::SETTINGS_PAGE_BACK);
  applyFocusBorder(gW.settings_home_brightness, gUi.focus == UiFocusId::SETTINGS_HOME_BRIGHTNESS);
  applyFocusBorder(gW.settings_home_ctrl, gUi.focus == UiFocusId::SETTINGS_HOME_CTRL);
  applyFocusBorder(gW.settings_home_telm, gUi.focus == UiFocusId::SETTINGS_HOME_TELM);
  applyFocusBorder(gW.settings_bright_value, gUi.focus == UiFocusId::SETTINGS_BRIGHT_VALUE);
  applyFocusBorder(gW.settings_bright_sleep_enable, gUi.focus == UiFocusId::SETTINGS_BRIGHT_SLEEP_ENABLE);
  applyFocusBorder(gW.settings_bright_sleep_timeout, gUi.focus == UiFocusId::SETTINGS_BRIGHT_SLEEP_TIMEOUT);
  applyFocusBorder(gW.settings_bright_apply, gUi.focus == UiFocusId::SETTINGS_BRIGHT_APPLY);
  applyFocusBorder(gW.settings_bright_back, gUi.focus == UiFocusId::SETTINGS_BRIGHT_BACK);
  applyFocusBorder(gW.settings_ctrl_channel, gUi.focus == UiFocusId::SETTINGS_CTRL_CHANNEL);
  applyFocusBorder(gW.settings_ctrl_dr, gUi.focus == UiFocusId::SETTINGS_CTRL_DR);
  applyFocusBorder(gW.settings_ctrl_pa, gUi.focus == UiFocusId::SETTINGS_CTRL_PA);
  applyFocusBorder(gW.settings_ctrl_ack, gUi.focus == UiFocusId::SETTINGS_CTRL_ACK);
  applyFocusBorder(gW.settings_ctrl_retry_delay, gUi.focus == UiFocusId::SETTINGS_CTRL_RETRY_DELAY);
  applyFocusBorder(gW.settings_ctrl_retry_count, gUi.focus == UiFocusId::SETTINGS_CTRL_RETRY_COUNT);
  applyFocusBorder(gW.settings_ctrl_apply, gUi.focus == UiFocusId::SETTINGS_CTRL_APPLY);
  applyFocusBorder(gW.settings_ctrl_back, gUi.focus == UiFocusId::SETTINGS_CTRL_BACK);
  applyFocusBorder(gW.settings_telm_channel, gUi.focus == UiFocusId::SETTINGS_TELM_CHANNEL);
  applyFocusBorder(gW.settings_telm_dr, gUi.focus == UiFocusId::SETTINGS_TELM_DR);
  applyFocusBorder(gW.settings_telm_pa, gUi.focus == UiFocusId::SETTINGS_TELM_PA);
  applyFocusBorder(gW.settings_telm_ack, gUi.focus == UiFocusId::SETTINGS_TELM_ACK);
  applyFocusBorder(gW.settings_telm_retry_delay, gUi.focus == UiFocusId::SETTINGS_TELM_RETRY_DELAY);
  applyFocusBorder(gW.settings_telm_retry_count, gUi.focus == UiFocusId::SETTINGS_TELM_RETRY_COUNT);
  applyFocusBorder(gW.settings_telm_apply, gUi.focus == UiFocusId::SETTINGS_TELM_APPLY);
  applyFocusBorder(gW.settings_telm_back, gUi.focus == UiFocusId::SETTINGS_TELM_BACK);
  refreshSettingsEditorLabels();
}

void ensureFocusValidForPage() {
  if (!focusIsInCurrentList(gUi.focus)) {
    gUi.focus = defaultFocusForPage(gUi.page);
  }
}

void setPage(UiPageId page) {
  const bool changed = (gUi.page != page);
  const UiPageId previous_page = gUi.page;
  gUi.page = page;
  if (changed || !focusIsInCurrentList(gUi.focus)) {
    gUi.focus = defaultFocusForPage(page);
  }

  if ((previous_page == UiPageId::SETTINGS) && (page != UiPageId::SETTINGS)) {
    gUi.brightness_apply_ok = false;
    gUi.radio_apply_ok = false;
  }

  lv_obj_add_flag(gW.page_dashboard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(gW.page_telemetry, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(gW.page_flight, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(gW.page_diagnostics, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(gW.page_calibration, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(gW.page_settings, LV_OBJ_FLAG_HIDDEN);

  if (page == UiPageId::DASHBOARD) {
    lv_obj_clear_flag(gW.page_dashboard, LV_OBJ_FLAG_HIDDEN);
  } else if (page == UiPageId::TELEMETRY) {
    lv_obj_clear_flag(gW.page_telemetry, LV_OBJ_FLAG_HIDDEN);
  } else if (page == UiPageId::FLIGHT) {
    lv_obj_clear_flag(gW.page_flight, LV_OBJ_FLAG_HIDDEN);
  } else if (page == UiPageId::DIAGNOSTICS) {
    lv_obj_clear_flag(gW.page_diagnostics, LV_OBJ_FLAG_HIDDEN);
  } else if (page == UiPageId::CALIBRATION) {
    lv_obj_clear_flag(gW.page_calibration, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(gW.page_settings, LV_OBJ_FLAG_HIDDEN);
  }

  if (page == UiPageId::SETTINGS) {
    showSettingsSubpage(gUi.settings_subpage);
  } else {
    showSettingsSubpage(SettingsSubpage::HOME);
  }

  lv_label_set_text(gW.title, pageTitle(page));
  ensureFocusValidForPage();
  refreshFocusVisuals();
}

void setTelemetryFilter(TelemFilter filter) {
  gUi.filter = filter;
  updateFilterButtonsVisual();
  refreshTelemetryLog();
  refreshFocusVisuals();
}

void cycleModeAction() {
  const uint8_t current = gLastStateValid ? gLastState.modeIndex : 0U;
  const uint8_t next_mode = normalizeModeIndex(static_cast<uint8_t>(current + 1U));
  queueModeIndex(next_mode);

  char line[64];
  snprintf(line, sizeof(line), "Mode -> %s", modeLabel(next_mode));
  telemetryPush(TelemKind::INFO, line);
  refreshTelemetryLog();
  refreshDashboardPreview();
}

void triggerTakeoffAction() {
  if (!gLastStateValid || (gLastState.armedState != ARMED_STATE_ARMED)) {
    telemetryPush(TelemKind::WARN, "WARN TAKEOFF blocked: not armed");
  } else {
    telemetryPush(TelemKind::WARN, "WARN TAKEOFF command queued");
  }
  refreshTelemetryLog();
  refreshDashboardPreview();
}

void setFocus(UiFocusId focus) {
  gUi.focus = focus;
  scrollFocusIntoView();
  refreshFocusVisuals();
}

void stepFocus(int8_t dir) {
  FocusItem list[kFocusListCapacity] = {};
  const uint8_t count = buildFocusList(list, kFocusListCapacity);
  if (count == 0U) {
    return;
  }

  int idx = -1;
  for (uint8_t i = 0; i < count; ++i) {
    if (list[i].id == gUi.focus) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    idx = 0;
  }

  idx += (dir > 0) ? 1 : -1;
  if (idx < 0) {
    idx = static_cast<int>(count) - 1;
  } else if (idx >= static_cast<int>(count)) {
    idx = 0;
  }

  setFocus(list[idx].id);
}

void activateFocusedItem() {
  switch (gUi.focus) {
    case UiFocusId::TAB_DASH:
    case UiFocusId::TAB_TELEM:
    case UiFocusId::TAB_FLIGHT:
    case UiFocusId::TAB_DIAG:
    case UiFocusId::TAB_CAL:
    case UiFocusId::TAB_SETTINGS:
      setPage(pageForTabFocus(gUi.focus));
      break;
    case UiFocusId::TELEM_FILTER_ALL:
      setTelemetryFilter(TelemFilter::ALL);
      break;
    case UiFocusId::TELEM_FILTER_LINK:
      setTelemetryFilter(TelemFilter::LINK);
      break;
    case UiFocusId::TELEM_FILTER_GPS:
      setTelemetryFilter(TelemFilter::GPS);
      break;
    case UiFocusId::TELEM_FILTER_WARN:
      setTelemetryFilter(TelemFilter::WARN);
      break;
    case UiFocusId::FLIGHT_MODE_NEXT:
      cycleModeAction();
      break;
    case UiFocusId::FLIGHT_TAKEOFF:
      triggerTakeoffAction();
      break;
    case UiFocusId::DIAG_CLEAR_CHART:
      if (gW.diag_chart != nullptr) {
        lv_chart_set_all_value(gW.diag_chart, gW.diag_series_ctrl, 0);
        lv_chart_set_all_value(gW.diag_chart, gW.diag_series_telm, 0);
        lv_chart_refresh(gW.diag_chart);
        gUi.diag_ctrl_ema = 0;
        gUi.diag_telm_ema = 0;
      }
      telemetryPush(TelemKind::INFO, "Diagnostics chart cleared");
      refreshTelemetryLog();
      break;
    case UiFocusId::CAL_SET_CENTER:
      telemetryPush(TelemKind::INFO, "Calibration template: set center requested");
      refreshTelemetryLog();
      break;
    case UiFocusId::CAL_SAVE_TEMPLATE:
      telemetryPush(TelemKind::INFO, "Calibration template: save requested");
      refreshTelemetryLog();
      break;
    case UiFocusId::SETTINGS_PAGE_BACK:
      showSettingsSubpage(SettingsSubpage::HOME);
      setPage(UiPageId::DASHBOARD);
      break;
    case UiFocusId::SETTINGS_HOME_BRIGHTNESS:
      showSettingsSubpage(SettingsSubpage::BRIGHTNESS);
      stepFocusTo(UiFocusId::SETTINGS_BRIGHT_VALUE);
      break;
    case UiFocusId::SETTINGS_HOME_CTRL:
      showSettingsSubpage(SettingsSubpage::NRF_CTRL);
      stepFocusTo(UiFocusId::SETTINGS_CTRL_CHANNEL);
      break;
    case UiFocusId::SETTINGS_HOME_TELM:
      showSettingsSubpage(SettingsSubpage::NRF_TELM);
      stepFocusTo(UiFocusId::SETTINGS_TELM_CHANNEL);
      break;
    case UiFocusId::SETTINGS_BRIGHT_APPLY:
      queueCurrentBrightness();
      break;
    case UiFocusId::SETTINGS_BRIGHT_BACK:
      showSettingsSubpage(SettingsSubpage::HOME);
      stepFocusTo(UiFocusId::SETTINGS_HOME_BRIGHTNESS);
      break;
    case UiFocusId::SETTINGS_CTRL_APPLY:
      queueCurrentRadioSettings();
      break;
    case UiFocusId::SETTINGS_CTRL_BACK:
      showSettingsSubpage(SettingsSubpage::HOME);
      stepFocusTo(UiFocusId::SETTINGS_HOME_CTRL);
      break;
    case UiFocusId::SETTINGS_TELM_APPLY:
      queueCurrentRadioSettings();
      break;
    case UiFocusId::SETTINGS_TELM_BACK:
      showSettingsSubpage(SettingsSubpage::HOME);
      stepFocusTo(UiFocusId::SETTINGS_HOME_TELM);
      break;
    default:
      if (focusSupportsValueStep(gUi.focus)) {
        stepSettingValue(gUi.focus, 1);
      }
      break;
  }
}

void handlePageNavInput(const SharedState& s, uint32_t now_ms) {
  if (gUi.sleeping) {
    if (hasWakeInput(s)) {
      wakeFromSleep(now_ms);
      gUi.nav_axis_dir = 0;
      gUi.edit_axis_dir = 0;
      gUi.nav_btn_latched = s.joystick.left_button_pressed;
    }
    return;
  }

  const bool user_active = s.joystick.left_button_pressed ||
                           (s.joystick.left_x_mapped >= kWakeAxisThreshold) ||
                           (s.joystick.left_x_mapped <= -kWakeAxisThreshold) ||
                           (s.joystick.right_x_mapped >= kWakeAxisThreshold) ||
                           (s.joystick.right_x_mapped <= -kWakeAxisThreshold) ||
                           (s.joystick.right_y_mapped >= kWakeAxisThreshold) ||
                           (s.joystick.right_y_mapped <= -kWakeAxisThreshold);
  if (user_active) {
    markUserActivity(now_ms);
  }

  if (isArmingCombo(s)) {
    gUi.nav_axis_dir = 0;
    gUi.edit_axis_dir = 0;
    return;
  }

  const int16_t axis = s.joystick.left_x_mapped;
  int8_t dir = 0;
  if (axis >= kNavAxisStepThreshold) {
    dir = 1;
  } else if (axis <= -kNavAxisStepThreshold) {
    dir = -1;
  }

  const int16_t abs_axis = (axis >= 0) ? axis : static_cast<int16_t>(-axis);
  if (abs_axis <= kNavAxisReleaseThreshold) {
    gUi.nav_axis_dir = 0;
  }

  int8_t nav_dir = dir;
  if (gUi.page == UiPageId::SETTINGS) {
    nav_dir = -nav_dir;
  }

  if (nav_dir != 0) {
    const bool direction_changed = (gUi.nav_axis_dir != nav_dir);
    const bool repeat_ready = (now_ms - gUi.nav_last_step_ms) >= kNavStepRepeatMs;
    if (direction_changed || repeat_ready) {
      gUi.nav_axis_dir = nav_dir;
      gUi.nav_last_step_ms = now_ms;
      stepFocus(nav_dir);
    }
  }

  int16_t edit_axis = s.joystick.right_x_mapped;
  const int16_t ry = s.joystick.right_y_mapped;
  if (((ry >= 0) ? ry : static_cast<int16_t>(-ry)) > ((edit_axis >= 0) ? edit_axis : static_cast<int16_t>(-edit_axis))) {
    edit_axis = ry;
  }
  int8_t edit_dir = 0;
  if (edit_axis >= kEditAxisStepThreshold) {
    edit_dir = 1;
  } else if (edit_axis <= -kEditAxisStepThreshold) {
    edit_dir = -1;
  }
  const int16_t edit_abs_axis = (edit_axis >= 0) ? edit_axis : static_cast<int16_t>(-edit_axis);
  if (edit_abs_axis <= kEditAxisReleaseThreshold) {
    gUi.edit_axis_dir = 0;
  }

  if ((edit_dir != 0) && focusSupportsValueStep(gUi.focus)) {
    const bool direction_changed = (gUi.edit_axis_dir != edit_dir);
    const bool repeat_ready = (now_ms - gUi.edit_last_step_ms) >= kEditStepRepeatMs;
    if (direction_changed || repeat_ready) {
      gUi.edit_axis_dir = edit_dir;
      gUi.edit_last_step_ms = now_ms;
      stepSettingValue(gUi.focus, edit_dir);
    }
  }

  const bool btn_pressed = s.joystick.left_button_pressed;
  if (btn_pressed && !gUi.nav_btn_latched) {
    gUi.nav_btn_latched = true;
    markUserActivity(now_ms);
    activateFocusedItem();
    return;
  }

  if (!btn_pressed) {
    gUi.nav_btn_latched = false;
  }
}

void updateFlightUi(const SharedState& s, uint32_t now_ms) {
  const bool armed = (s.armedState == ARMED_STATE_ARMED);
  const bool combo = isArmingCombo(s);

  if (armed) {
    gUi.armHoldActive = false;
    gUi.armCommitSent = false;
    lv_label_set_text(gW.flight_status, "Status: ARMED");
    lv_obj_set_style_text_color(gW.flight_status, col(C_GREEN), 0);
    lv_label_set_text(gW.flight_progress, "Armed. TAKEOFF enabled.");
    lv_bar_set_value(gW.flight_bar, 100, LV_ANIM_OFF);
    lv_obj_clear_state(gW.flight_takeoff, LV_STATE_DISABLED);
    return;
  }

  lv_obj_add_state(gW.flight_takeoff, LV_STATE_DISABLED);

  if (combo) {
    if (!gUi.armHoldActive) {
      gUi.armHoldActive = true;
      gUi.armCommitSent = false;
      gUi.armHoldStartMs = now_ms;
      if (s.armedState != ARMED_STATE_ARMING) {
        queueArmedState(ARMED_STATE_ARMING);
      }
    }

    const uint32_t held_ms = now_ms - gUi.armHoldStartMs;
    const uint16_t pct = static_cast<uint16_t>((held_ms >= kArmHoldMs)
                                                    ? 100U
                                                    : ((held_ms * 100U) / kArmHoldMs));
    lv_bar_set_value(gW.flight_bar, pct, LV_ANIM_OFF);

    char progress[48];
    snprintf(progress, sizeof(progress), "Holding combo: %u%%", static_cast<unsigned>(pct));
    lv_label_set_text(gW.flight_progress, progress);
    lv_label_set_text(gW.flight_status, "Status: ARMING");
    lv_obj_set_style_text_color(gW.flight_status, col(C_DIM), 0);

    if ((held_ms >= kArmHoldMs) && !gUi.armCommitSent) {
      gUi.armCommitSent = true;
      queueArmedState(ARMED_STATE_ARMED);
      telemetryPush(TelemKind::WARN, "ARMED: combo hold complete");
      refreshTelemetryLog();
      refreshDashboardPreview();
    }
    return;
  }

  if (gUi.armHoldActive && (s.armedState == ARMED_STATE_ARMING)) {
    queueArmedState(ARMED_STATE_DISARMED);
  }

  gUi.armHoldActive = false;
  gUi.armCommitSent = false;
  lv_label_set_text(gW.flight_status, "Status: DISARMED");
  lv_obj_set_style_text_color(gW.flight_status, col(C_RED), 0);
  lv_label_set_text(gW.flight_progress, "Hold both sticks down for 1s");
  lv_bar_set_value(gW.flight_bar, 0, LV_ANIM_OFF);
}

void applyStateToUi(const SharedState& s, uint32_t now_ms) {
  gLastState = s;
  gLastStateValid = true;

  const uint8_t live_duty = clampU8(s.settings.backlight_duty, 0, kBacklightDutyMax);
  if (!gUi.brightness_dirty) {
    gUi.awake_backlight_duty = live_duty;
    if (!gUi.sleeping) {
      setBacklightDuty(gUi.awake_backlight_duty);
    }
  }

  if (!gUi.radio_edit_loaded || !gUi.radio_dirty) {
    gUi.radio_edit = s.settings.radio;
    gUi.radio_edit_loaded = true;
  }

  lv_obj_set_style_bg_color(gW.link_dot, s.linkOk ? col(C_GREEN) : col(C_RED), 0);
  lv_label_set_text(gW.link_label, s.linkOk ? "LINK OK" : "LINK LOST");
  lv_obj_set_style_text_color(gW.link_label, s.linkOk ? col(C_GREEN) : col(C_RED), 0);

  const char* batt_icon = LV_SYMBOL_BATTERY_EMPTY;
  if (s.batteryPercent >= 90U) {
    batt_icon = LV_SYMBOL_BATTERY_FULL;
  } else if (s.batteryPercent >= 70U) {
    batt_icon = LV_SYMBOL_BATTERY_3;
  } else if (s.batteryPercent >= 45U) {
    batt_icon = LV_SYMBOL_BATTERY_2;
  } else if (s.batteryPercent >= 20U) {
    batt_icon = LV_SYMBOL_BATTERY_1;
  }
  lv_label_set_text(gW.batt_icon, batt_icon);

  char batt_text[48];
  snprintf(batt_text,
           sizeof(batt_text),
           "%u%% %.2fV",
           static_cast<unsigned>(s.batteryPercent),
           static_cast<double>(s.batteryVoltage));
  lv_label_set_text(gW.batt_label, batt_text);
  lv_obj_set_style_text_color(gW.batt_label, (s.batteryPercent > 20U) ? col(C_GREEN) : col(C_RED), 0);

  char v[64];
  snprintf(v, sizeof(v), "%u%% %.2fV", static_cast<unsigned>(s.batteryPercent), static_cast<double>(s.batteryVoltage));
  lv_label_set_text(gW.dash_battery, v);
  lv_obj_set_style_text_color(gW.dash_battery, (s.batteryPercent > 20U) ? col(C_GREEN) : col(C_RED), 0);

  if (s.gpsFix) {
    snprintf(v, sizeof(v), "FIX %.4f %.4f", static_cast<double>(s.lat), static_cast<double>(s.lon));
  } else {
    snprintf(v, sizeof(v), "NO FIX");
  }
  lv_label_set_text(gW.dash_gps, v);
  lv_obj_set_style_text_color(gW.dash_gps, s.gpsFix ? col(C_GREEN) : col(C_RED), 0);

  snprintf(v, sizeof(v), "%.1f m", static_cast<double>(s.altitudeM));
  lv_label_set_text(gW.dash_altitude, v);

  snprintf(v, sizeof(v), "%lu m", static_cast<unsigned long>(s.distanceMm / 1000UL));
  lv_label_set_text(gW.dash_distance, v);

  lv_label_set_text(gW.dash_mode, s.mode);
  lv_obj_set_style_text_color(gW.dash_mode,
                              (s.armedState == ARMED_STATE_ARMED) ? col(C_GREEN) : col(C_DIM),
                              0);

  if (s.linkOk) {
    snprintf(v, sizeof(v), "OK / %lu pkts", static_cast<unsigned long>(s.pktCount));
  } else {
    snprintf(v, sizeof(v), "LOST / %lu pkts", static_cast<unsigned long>(s.pktCount));
  }
  lv_label_set_text(gW.dash_link, v);
  lv_obj_set_style_text_color(gW.dash_link, s.linkOk ? col(C_GREEN) : col(C_RED), 0);

  if (gW.dash_fcu_status != nullptr) {
    snprintf(v,
             sizeof(v),
             "%s  %u%% %.2fV",
             s.linkOk ? "ONLINE" : "LOST",
             static_cast<unsigned>(s.batteryPercent),
             static_cast<double>(s.batteryVoltage));
    lv_label_set_text(gW.dash_fcu_status, v);
    const bool fcu_good = s.linkOk && (s.batteryPercent > 20U);
    lv_obj_set_style_text_color(gW.dash_fcu_status, fcu_good ? col(C_GREEN) : col(C_RED), 0);
  }

  if (strcmp(s.lastTelemLine, gUi.lastTelemLineSeen) != 0) {
    snprintf(gUi.lastTelemLineSeen, sizeof(gUi.lastTelemLineSeen), "%s", s.lastTelemLine);
    telemetryPush(detectKind(s.lastTelemLine), s.lastTelemLine);
    refreshTelemetryLog();
    refreshDashboardPreview();
  }

  if ((s.batteryPercent <= 15U) && ((now_ms - gUi.lastLowBattWarnMs) >= kLowBatteryWarnPeriodMs)) {
    gUi.lastLowBattWarnMs = now_ms;
    char warn[64];
    snprintf(warn,
             sizeof(warn),
             "WARN battery low: %u%% %.2fV",
             static_cast<unsigned>(s.batteryPercent),
             static_cast<double>(s.batteryVoltage));
    telemetryPush(TelemKind::WARN, warn);
    refreshTelemetryLog();
    refreshDashboardPreview();
  }

  updateFlightUi(s, now_ms);

  if (gW.diag_ctrl_state != nullptr) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "CTRL %s  age:%lums  pps:%lu",
             s.ctrl.link_ok ? "OK" : "LOST",
             static_cast<unsigned long>(s.ctrl.age_ms),
             static_cast<unsigned long>(s.ctrl_tx_pps));
    lv_label_set_text(gW.diag_ctrl_state, line);
    lv_obj_set_style_text_color(gW.diag_ctrl_state, s.ctrl.link_ok ? col(C_GREEN) : col(C_RED), 0);
  }

  if (gW.diag_telm_state != nullptr) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "TELM %s  age:%lums  pps:%lu",
             s.telm.link_ok ? "OK" : "LOST",
             static_cast<unsigned long>(s.telm.age_ms),
             static_cast<unsigned long>(s.telm_tx_pps));
    lv_label_set_text(gW.diag_telm_state, line);
    lv_obj_set_style_text_color(gW.diag_telm_state, s.telm.link_ok ? col(C_GREEN) : col(C_RED), 0);
  }

  if (gW.diag_pkt_line != nullptr) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "TX ctrl:%lu telm:%lu | RX ctrl:%lu telm:%lu",
             static_cast<unsigned long>(s.ctrl_stats.tx_count),
             static_cast<unsigned long>(s.telm_stats.tx_count),
             static_cast<unsigned long>(s.ctrl_stats.rx_ok),
             static_cast<unsigned long>(s.telm_stats.rx_ok));
    lv_label_set_text(gW.diag_pkt_line, line);
    lv_obj_set_style_text_color(gW.diag_pkt_line, col(C_TEXT), 0);
  }

  if (gW.diag_loss_line != nullptr) {
    char line[96];
    const uint32_t bad_ctrl = s.ctrl_stats.rx_bad_crc + s.ctrl_stats.rx_bad_magic + s.ctrl_stats.payload_corrupt;
    const uint32_t bad_telm = s.telm_stats.rx_bad_crc + s.telm_stats.rx_bad_magic + s.telm_stats.payload_corrupt;
    snprintf(line,
             sizeof(line),
             "gaps:%lu dup:%lu | bad ctrl:%lu telm:%lu",
             static_cast<unsigned long>(s.ctrl_stats.gaps + s.telm_stats.gaps),
             static_cast<unsigned long>(s.ctrl_stats.duplicates + s.telm_stats.duplicates),
             static_cast<unsigned long>(bad_ctrl),
             static_cast<unsigned long>(bad_telm));
    lv_label_set_text(gW.diag_loss_line, line);
    lv_obj_set_style_text_color(gW.diag_loss_line, (bad_ctrl + bad_telm) ? col(C_RED) : col(C_GREEN), 0);
  }

  if (gW.diag_mode_line != nullptr) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "mode:%s  armed:%s",
             s.mode,
             (s.armedState == ARMED_STATE_ARMED) ? "YES" : "NO");
    lv_label_set_text(gW.diag_mode_line, line);
    lv_obj_set_style_text_color(gW.diag_mode_line,
                                (s.armedState == ARMED_STATE_ARMED) ? col(C_GREEN) : col(C_DIM),
                                0);
  }

  if (gW.diag_heap_line != nullptr) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "heap:%luB  uptime:%lus",
             static_cast<unsigned long>(s.free_heap),
             static_cast<unsigned long>(s.uptime_ms / 1000UL));
    lv_label_set_text(gW.diag_heap_line, line);
    lv_obj_set_style_text_color(gW.diag_heap_line, col(C_DIM), 0);
  }

  if ((gW.diag_chart != nullptr) &&
      (gW.diag_series_ctrl != nullptr) &&
      (gW.diag_series_telm != nullptr) &&
      ((now_ms - gUi.lastDiagSampleMs) >= kDiagSamplePeriodMs)) {
    gUi.lastDiagSampleMs = now_ms;
    const int32_t ctrl_raw = static_cast<int32_t>((s.ctrl_tx_pps > 2000U) ? 2000U : s.ctrl_tx_pps);
    const int32_t telm_raw = static_cast<int32_t>((s.telm_tx_pps > 2000U) ? 2000U : s.telm_tx_pps);

    if (gUi.diag_ctrl_ema == 0) {
      gUi.diag_ctrl_ema = ctrl_raw;
    } else {
      gUi.diag_ctrl_ema = ((gUi.diag_ctrl_ema * 3) + ctrl_raw) / 4;
    }
    if (gUi.diag_telm_ema == 0) {
      gUi.diag_telm_ema = telm_raw;
    } else {
      gUi.diag_telm_ema = ((gUi.diag_telm_ema * 3) + telm_raw) / 4;
    }

    int32_t ymax = gUi.diag_ctrl_ema;
    if (gUi.diag_telm_ema > ymax) {
      ymax = gUi.diag_telm_ema;
    }
    if (ymax < 40) {
      ymax = 40;
    }
    lv_chart_set_range(gW.diag_chart, LV_CHART_AXIS_PRIMARY_Y, 0, ymax + 20);
    lv_chart_set_next_value(gW.diag_chart, gW.diag_series_ctrl, gUi.diag_ctrl_ema);
    lv_chart_set_next_value(gW.diag_chart, gW.diag_series_telm, gUi.diag_telm_ema);
    lv_chart_refresh(gW.diag_chart);
  }

  if ((gW.cal_lx_label != nullptr) && (gW.cal_lx_bar != nullptr)) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "L.X mapped:%d raw:%u",
             static_cast<int>(s.joystick.left_x_mapped),
             static_cast<unsigned>(s.joystick.left_x_raw));
    lv_label_set_text(gW.cal_lx_label, line);
    lv_bar_set_value(gW.cal_lx_bar, s.joystick.left_x_mapped, LV_ANIM_OFF);
  }

  if ((gW.cal_rx_label != nullptr) && (gW.cal_rx_bar != nullptr)) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "R.X mapped:%d raw:%u",
             static_cast<int>(s.joystick.right_x_mapped),
             static_cast<unsigned>(s.joystick.right_x_raw));
    lv_label_set_text(gW.cal_rx_label, line);
    lv_bar_set_value(gW.cal_rx_bar, s.joystick.right_x_mapped, LV_ANIM_OFF);
  }

  if ((gW.cal_ry_label != nullptr) && (gW.cal_ry_bar != nullptr)) {
    char line[96];
    snprintf(line,
             sizeof(line),
             "R.Y mapped:%d raw:%u",
             static_cast<int>(s.joystick.right_y_mapped),
             static_cast<unsigned>(s.joystick.right_y_raw));
    lv_label_set_text(gW.cal_ry_label, line);
    lv_bar_set_value(gW.cal_ry_bar, s.joystick.right_y_mapped, LV_ANIM_OFF);
  }

  refreshSettingsEditorLabels();

  handlePageNavInput(s, now_ms);

  if (!gUi.sleep_enabled && gUi.sleeping) {
    wakeFromSleep(now_ms);
  }
  if (!gUi.sleeping && gUi.sleep_enabled && (gUi.sleep_timeout_ms > 0U)) {
    if ((now_ms - gUi.last_user_activity_ms) >= gUi.sleep_timeout_ms) {
      enterSleepMode(now_ms);
    }
  }
}

lv_obj_t* makeButton(lv_obj_t* parent,
                     const char* txt,
                     int16_t x,
                     int16_t y,
                     int16_t w,
                     int16_t h,
                     lv_event_cb_t cb,
                     void* user_data = nullptr) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &gStyleBtn, 0);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_pos(btn, x, y);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

  lv_obj_t* label = lv_label_create(btn);
  lv_label_set_text(label, txt);
  lv_obj_set_style_text_color(label, col(C_TEXT), 0);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
  lv_obj_set_width(label, static_cast<lv_coord_t>(w - 8));
  lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
  lv_obj_center(label);
  return btn;
}

lv_obj_t* makeTile(lv_obj_t* parent,
                   int16_t x,
                   int16_t y,
                   int16_t w,
                   int16_t h,
                   const char* title,
                   lv_obj_t** value_label) {
  lv_obj_t* card = lv_obj_create(parent);
  lv_obj_remove_style_all(card);
  lv_obj_add_style(card, &gStyleCard, 0);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_size(card, w, h);
  lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* tl = lv_label_create(card);
  lv_label_set_text(tl, title);
  lv_obj_set_style_text_color(tl, col(C_DIM), 0);
  lv_obj_set_style_text_font(tl, &lv_font_montserrat_12, 0);
  lv_obj_set_width(tl, static_cast<lv_coord_t>(w - 8));
  lv_label_set_long_mode(tl, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(tl, 4, 1);

  lv_obj_t* vl = lv_label_create(card);
  lv_label_set_text(vl, "-");
  lv_obj_set_style_text_color(vl, col(C_TEXT), 0);
  lv_obj_set_style_text_font(vl, &lv_font_montserrat_14, 0);
  lv_obj_set_width(vl, static_cast<lv_coord_t>(w - 8));
  lv_label_set_long_mode(vl, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(vl, 4, 15);

  if (value_label != nullptr) {
    *value_label = vl;
  }
  return card;
}

void onNavPage(lv_event_t* e) {
  const UiPageId page = static_cast<UiPageId>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
  setPage(page);
}

void onModeToggle(lv_event_t* e) {
  (void)e;
  cycleModeAction();
  refreshFocusVisuals();
}

void onTelemetryFilter(lv_event_t* e) {
  const TelemFilter filter = static_cast<TelemFilter>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
  setTelemetryFilter(filter);
}

void onFocusActivate(lv_event_t* e) {
  const UiFocusId focus = static_cast<UiFocusId>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(e)));
  setFocus(focus);
  activateFocusedItem();
  refreshFocusVisuals();
}

void onTakeoff(lv_event_t* e) {
  (void)e;
  triggerTakeoffAction();
  refreshFocusVisuals();
}

void onDiagClearChart(lv_event_t* e) {
  (void)e;
  setFocus(UiFocusId::DIAG_CLEAR_CHART);
  activateFocusedItem();
  refreshFocusVisuals();
}

void onCalSetCenter(lv_event_t* e) {
  (void)e;
  setFocus(UiFocusId::CAL_SET_CENTER);
  activateFocusedItem();
  refreshFocusVisuals();
}

void onCalSaveTemplate(lv_event_t* e) {
  (void)e;
  setFocus(UiFocusId::CAL_SAVE_TEMPLATE);
  activateFocusedItem();
  refreshFocusVisuals();
}

void buildUi() {
  initStyles();
  constexpr int16_t kPagePad = 4;
  constexpr int16_t kColGap = 8;
  constexpr int16_t kRowGap = 4;

  constexpr int16_t kDashTileW = (kScreenW - (kPagePad * 2) - kColGap) / 2;
  constexpr int16_t kDashTileH = 38;
  constexpr int16_t kDashRow1Y = kPagePad;
  constexpr int16_t kDashRow2Y = kDashRow1Y + kDashTileH + kRowGap;
  constexpr int16_t kDashRow3Y = kDashRow2Y + kDashTileH + kRowGap;
  constexpr int16_t kDashBottomY = kDashRow3Y + kDashTileH + kRowGap;
  constexpr int16_t kDashBottomH = kContentH - kDashBottomY - kPagePad;
  constexpr int16_t kDashFcuW = 208;
  constexpr int16_t kDashPropW = kScreenW - (kPagePad * 2) - kColGap - kDashFcuW;

  lv_obj_t* scr = lv_scr_act();
  lv_obj_remove_style_all(scr);
  lv_obj_set_style_bg_color(scr, col(C_BG), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  gW.topbar = lv_obj_create(scr);
  lv_obj_remove_style_all(gW.topbar);
  lv_obj_set_size(gW.topbar, kScreenW, kTopBarH);
  lv_obj_set_pos(gW.topbar, 0, 0);
  lv_obj_set_style_bg_color(gW.topbar, col(C_TOP), 0);
  lv_obj_set_style_bg_opa(gW.topbar, LV_OPA_COVER, 0);
  lv_obj_clear_flag(gW.topbar, LV_OBJ_FLAG_SCROLLABLE);

  gW.link_dot = lv_obj_create(gW.topbar);
  lv_obj_remove_style_all(gW.link_dot);
  lv_obj_set_size(gW.link_dot, 8, 8);
  lv_obj_set_pos(gW.link_dot, 8, 12);
  lv_obj_set_style_radius(gW.link_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(gW.link_dot, col(C_RED), 0);

  gW.link_label = lv_label_create(gW.topbar);
  lv_label_set_text(gW.link_label, "LINK");
  lv_obj_set_style_text_color(gW.link_label, col(C_DIM), 0);
  lv_obj_set_style_text_font(gW.link_label, &lv_font_montserrat_12, 0);
  lv_obj_set_width(gW.link_label, 72);
  lv_label_set_long_mode(gW.link_label, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.link_label, 22, 8);

  gW.title = lv_label_create(gW.topbar);
  lv_label_set_text(gW.title, "Dashboard");
  lv_obj_set_style_text_color(gW.title, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.title, &lv_font_montserrat_14, 0);
  lv_obj_set_width(gW.title, 126);
  lv_label_set_long_mode(gW.title, LV_LABEL_LONG_DOT);
  lv_obj_align(gW.title, LV_ALIGN_CENTER, 0, 0);

  gW.batt_icon = lv_label_create(gW.topbar);
  lv_label_set_text(gW.batt_icon, LV_SYMBOL_BATTERY_FULL);
  lv_obj_set_style_text_color(gW.batt_icon, col(C_TEXT), 0);
  lv_obj_set_pos(gW.batt_icon, kScreenW - 98, 7);

  gW.batt_label = lv_label_create(gW.topbar);
  lv_label_set_text(gW.batt_label, "--% --.--V");
  lv_obj_set_style_text_color(gW.batt_label, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.batt_label, &lv_font_montserrat_12, 0);
  lv_obj_set_width(gW.batt_label, 74);
  lv_label_set_long_mode(gW.batt_label, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.batt_label, kScreenW - 74, 8);

  lv_obj_t* nav = lv_obj_create(scr);
  lv_obj_remove_style_all(nav);
  lv_obj_set_size(nav, kScreenW, kQuickNavH);
  lv_obj_set_pos(nav, 0, kTopBarH);
  lv_obj_set_style_bg_opa(nav, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(nav, LV_OBJ_FLAG_SCROLLABLE);

  constexpr int16_t kNavPad = 2;
  constexpr int16_t kNavGap = 2;
  constexpr int16_t kNavBtnW = (kScreenW - (kNavPad * 2) - (kNavGap * 5)) / 6;
  constexpr int16_t kNavBtnH = 24;
  const int16_t nav_y = 1;

  gW.nav_dash = makeButton(nav,
                           "Dash",
                           kNavPad,
                           nav_y,
                           kNavBtnW,
                           kNavBtnH,
                           onNavPage,
                           reinterpret_cast<void*>(static_cast<uintptr_t>(UiPageId::DASHBOARD)));
  gW.nav_telem = makeButton(nav,
                            "Telem",
                            kNavPad + kNavBtnW + kNavGap,
                            nav_y,
                            kNavBtnW,
                            kNavBtnH,
                            onNavPage,
                            reinterpret_cast<void*>(static_cast<uintptr_t>(UiPageId::TELEMETRY)));
  gW.nav_flight = makeButton(nav,
                             "Flight",
                             kNavPad + ((kNavBtnW + kNavGap) * 2),
                             nav_y,
                             kNavBtnW,
                             kNavBtnH,
                             onNavPage,
                             reinterpret_cast<void*>(static_cast<uintptr_t>(UiPageId::FLIGHT)));
  gW.nav_diag = makeButton(nav,
                           "Diag",
                           kNavPad + ((kNavBtnW + kNavGap) * 3),
                           nav_y,
                           kNavBtnW,
                           kNavBtnH,
                           onNavPage,
                           reinterpret_cast<void*>(static_cast<uintptr_t>(UiPageId::DIAGNOSTICS)));
  gW.nav_cal = makeButton(nav,
                          "Cal",
                          kNavPad + ((kNavBtnW + kNavGap) * 4),
                          nav_y,
                          kNavBtnW,
                          kNavBtnH,
                          onNavPage,
                          reinterpret_cast<void*>(static_cast<uintptr_t>(UiPageId::CALIBRATION)));
  gW.nav_settings = makeButton(nav,
                               "Setup",
                               kNavPad + ((kNavBtnW + kNavGap) * 5),
                               nav_y,
                               kNavBtnW,
                               kNavBtnH,
                               onNavPage,
                               reinterpret_cast<void*>(static_cast<uintptr_t>(UiPageId::SETTINGS)));

  gW.content = lv_obj_create(scr);
  lv_obj_remove_style_all(gW.content);
  lv_obj_set_size(gW.content, kScreenW, kContentH);
  lv_obj_set_pos(gW.content, 0, kContentY);
  lv_obj_set_style_bg_opa(gW.content, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.content, LV_OBJ_FLAG_SCROLLABLE);

  gW.page_dashboard = lv_obj_create(gW.content);
  lv_obj_remove_style_all(gW.page_dashboard);
  lv_obj_set_size(gW.page_dashboard, kScreenW, kContentH);
  lv_obj_set_pos(gW.page_dashboard, 0, 0);
  lv_obj_set_style_bg_opa(gW.page_dashboard, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.page_dashboard, LV_OBJ_FLAG_SCROLLABLE);

  makeTile(gW.page_dashboard, kPagePad, kDashRow1Y, kDashTileW, kDashTileH, "Battery", &gW.dash_battery);
  makeTile(gW.page_dashboard,
           kPagePad + kDashTileW + kColGap,
           kDashRow1Y,
           kDashTileW,
           kDashTileH,
           "GPS",
           &gW.dash_gps);
  makeTile(gW.page_dashboard, kPagePad, kDashRow2Y, kDashTileW, kDashTileH, "Altitude", &gW.dash_altitude);
  makeTile(gW.page_dashboard,
           kPagePad + kDashTileW + kColGap,
           kDashRow2Y,
           kDashTileW,
           kDashTileH,
           "Distance",
           &gW.dash_distance);
  makeTile(gW.page_dashboard, kPagePad, kDashRow3Y, kDashTileW, kDashTileH, "Mode", &gW.dash_mode);
  makeTile(gW.page_dashboard,
           kPagePad + kDashTileW + kColGap,
           kDashRow3Y,
           kDashTileW,
           kDashTileH,
           "Link Quality",
           &gW.dash_link);

  gW.dash_fcu_card = lv_obj_create(gW.page_dashboard);
  lv_obj_remove_style_all(gW.dash_fcu_card);
  lv_obj_add_style(gW.dash_fcu_card, &gStyleCard, 0);
  lv_obj_set_pos(gW.dash_fcu_card, kPagePad, kDashBottomY);
  lv_obj_set_size(gW.dash_fcu_card, kDashFcuW, kDashBottomH);
  lv_obj_clear_flag(gW.dash_fcu_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* dash_fcu_title = lv_label_create(gW.dash_fcu_card);
  lv_label_set_text(dash_fcu_title, "FCU Battery");
  lv_obj_set_style_text_color(dash_fcu_title, col(C_DIM), 0);
  lv_obj_set_style_text_font(dash_fcu_title, &lv_font_montserrat_12, 0);
  lv_obj_set_pos(dash_fcu_title, 4, 1);

  gW.dash_fcu_status = lv_label_create(gW.dash_fcu_card);
  lv_label_set_text(gW.dash_fcu_status, "ONLINE");
  lv_obj_set_style_text_color(gW.dash_fcu_status, col(C_GREEN), 0);
  lv_obj_set_style_text_font(gW.dash_fcu_status, &lv_font_montserrat_16, 0);
  lv_obj_set_width(gW.dash_fcu_status, static_cast<lv_coord_t>(kDashFcuW - 8));
  lv_label_set_long_mode(gW.dash_fcu_status, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.dash_fcu_status, 4, 16);

  gW.dash_prop_card = lv_obj_create(gW.page_dashboard);
  lv_obj_remove_style_all(gW.dash_prop_card);
  lv_obj_add_style(gW.dash_prop_card, &gStyleCard, 0);
  lv_obj_set_pos(gW.dash_prop_card, kPagePad + kDashFcuW + kColGap, kDashBottomY);
  lv_obj_set_size(gW.dash_prop_card, kDashPropW, kDashBottomH);
  lv_obj_clear_flag(gW.dash_prop_card, LV_OBJ_FLAG_SCROLLABLE);

  gW.dash_prop_spinner = lv_label_create(gW.dash_prop_card);
  lv_label_set_text(gW.dash_prop_spinner, "Firmware v1");
  lv_obj_set_style_text_color(gW.dash_prop_spinner, col(C_DIM), 0);
  lv_obj_set_style_text_font(gW.dash_prop_spinner, &lv_font_montserrat_12, 0);
  lv_obj_align(gW.dash_prop_spinner, LV_ALIGN_CENTER, 0, 8);

  gW.page_telemetry = lv_obj_create(gW.content);
  lv_obj_remove_style_all(gW.page_telemetry);
  lv_obj_set_size(gW.page_telemetry, kScreenW, kContentH);
  lv_obj_set_pos(gW.page_telemetry, 0, 0);
  lv_obj_set_style_bg_opa(gW.page_telemetry, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.page_telemetry, LV_OBJ_FLAG_SCROLLABLE);

  gW.telem_filter_all = makeButton(gW.page_telemetry,
                                    "ALL",
                                    kNavPad,
                                    4,
                                    kNavBtnW,
                                    22,
                                    onTelemetryFilter,
                                    reinterpret_cast<void*>(static_cast<uintptr_t>(TelemFilter::ALL)));
  gW.telem_filter_link = makeButton(gW.page_telemetry,
                                     "LINK",
                                     kNavPad + kNavBtnW + kNavGap,
                                     4,
                                     kNavBtnW,
                                     22,
                                     onTelemetryFilter,
                                     reinterpret_cast<void*>(static_cast<uintptr_t>(TelemFilter::LINK)));
  gW.telem_filter_gps = makeButton(gW.page_telemetry,
                                    "GPS",
                                    kNavPad + ((kNavBtnW + kNavGap) * 2),
                                    4,
                                    kNavBtnW,
                                    22,
                                    onTelemetryFilter,
                                    reinterpret_cast<void*>(static_cast<uintptr_t>(TelemFilter::GPS)));
  gW.telem_filter_warn = makeButton(gW.page_telemetry,
                                     "WARN",
                                     kNavPad + ((kNavBtnW + kNavGap) * 3),
                                     4,
                                     kNavBtnW,
                                     22,
                                     onTelemetryFilter,
                                     reinterpret_cast<void*>(static_cast<uintptr_t>(TelemFilter::WARN)));

  gW.telem_log_container = lv_obj_create(gW.page_telemetry);
  lv_obj_remove_style_all(gW.telem_log_container);
  lv_obj_add_style(gW.telem_log_container, &gStyleCard, 0);
  lv_obj_set_pos(gW.telem_log_container, 4, 30);
  lv_obj_set_size(gW.telem_log_container, 312, 146);
  lv_obj_set_scroll_dir(gW.telem_log_container, LV_DIR_VER);

  gW.telem_log_label = lv_label_create(gW.telem_log_container);
  lv_obj_set_width(gW.telem_log_label, 300);
  lv_label_set_long_mode(gW.telem_log_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(gW.telem_log_label, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.telem_log_label, &lv_font_montserrat_12, 0);
  lv_obj_set_pos(gW.telem_log_label, 2, 2);
  lv_label_set_text(gW.telem_log_label, "Telemetry stream is empty.");

  gW.page_flight = lv_obj_create(gW.content);
  lv_obj_remove_style_all(gW.page_flight);
  lv_obj_set_size(gW.page_flight, kScreenW, kContentH);
  lv_obj_set_pos(gW.page_flight, 0, 0);
  lv_obj_set_style_bg_opa(gW.page_flight, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.page_flight, LV_OBJ_FLAG_SCROLLABLE);

  gW.flight_status = lv_label_create(gW.page_flight);
  lv_obj_set_style_text_color(gW.flight_status, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.flight_status, &lv_font_montserrat_16, 0);
  lv_obj_set_width(gW.flight_status, 304);
  lv_label_set_long_mode(gW.flight_status, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.flight_status, 8, 8);
  lv_label_set_text(gW.flight_status, "Status: DISARMED");

  gW.flight_combo = lv_label_create(gW.page_flight);
  lv_obj_set_style_text_color(gW.flight_combo, col(C_DIM), 0);
  lv_obj_set_style_text_font(gW.flight_combo, &lv_font_montserrat_12, 0);
  lv_obj_set_width(gW.flight_combo, 304);
  lv_label_set_long_mode(gW.flight_combo, LV_LABEL_LONG_WRAP);
  lv_obj_set_pos(gW.flight_combo, 8, 32);
  lv_label_set_text(gW.flight_combo, "Arming combo: hold both sticks down");

  gW.flight_bar = lv_bar_create(gW.page_flight);
  lv_obj_set_size(gW.flight_bar, 304, 18);
  lv_obj_set_pos(gW.flight_bar, 8, 70);
  lv_bar_set_range(gW.flight_bar, 0, 100);
  lv_bar_set_value(gW.flight_bar, 0, LV_ANIM_OFF);

  gW.flight_progress = lv_label_create(gW.page_flight);
  lv_obj_set_style_text_color(gW.flight_progress, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.flight_progress, &lv_font_montserrat_12, 0);
  lv_obj_set_width(gW.flight_progress, 304);
  lv_label_set_long_mode(gW.flight_progress, LV_LABEL_LONG_WRAP);
  lv_obj_set_pos(gW.flight_progress, 8, 94);
  lv_label_set_text(gW.flight_progress, "Hold combo for 1 second to arm");

  gW.flight_mode_next = makeButton(gW.page_flight, "NEXT MODE", 8, 132, 146, 36, onModeToggle, nullptr);
  gW.flight_takeoff = makeButton(gW.page_flight, "TAKEOFF", 166, 132, 146, 36, onTakeoff, nullptr);
  lv_obj_add_state(gW.flight_takeoff, LV_STATE_DISABLED);

  gW.page_diagnostics = lv_obj_create(gW.content);
  lv_obj_remove_style_all(gW.page_diagnostics);
  lv_obj_set_size(gW.page_diagnostics, kScreenW, kContentH);
  lv_obj_set_pos(gW.page_diagnostics, 0, 0);
  lv_obj_set_style_bg_opa(gW.page_diagnostics, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.page_diagnostics, LV_OBJ_FLAG_SCROLLABLE);

  gW.diag_ctrl_state = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_ctrl_state, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_ctrl_state, col(C_TEXT), 0);
  lv_obj_set_width(gW.diag_ctrl_state, 304);
  lv_label_set_long_mode(gW.diag_ctrl_state, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_ctrl_state, 8, 6);
  lv_label_set_text(gW.diag_ctrl_state, "CTRL --");

  gW.diag_telm_state = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_telm_state, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_telm_state, col(C_TEXT), 0);
  lv_obj_set_width(gW.diag_telm_state, 304);
  lv_label_set_long_mode(gW.diag_telm_state, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_telm_state, 8, 22);
  lv_label_set_text(gW.diag_telm_state, "TELM --");

  gW.diag_pkt_line = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_pkt_line, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_pkt_line, col(C_TEXT), 0);
  lv_obj_set_width(gW.diag_pkt_line, 304);
  lv_label_set_long_mode(gW.diag_pkt_line, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_pkt_line, 8, 40);
  lv_label_set_text(gW.diag_pkt_line, "TX/RX --");

  gW.diag_loss_line = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_loss_line, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_loss_line, col(C_DIM), 0);
  lv_obj_set_width(gW.diag_loss_line, 304);
  lv_label_set_long_mode(gW.diag_loss_line, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_loss_line, 8, 56);
  lv_label_set_text(gW.diag_loss_line, "loss/bad --");

  gW.diag_mode_line = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_mode_line, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_mode_line, col(C_TEXT), 0);
  lv_obj_set_width(gW.diag_mode_line, 196);
  lv_label_set_long_mode(gW.diag_mode_line, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_mode_line, 8, 72);
  lv_label_set_text(gW.diag_mode_line, "mode: --");

  gW.diag_heap_line = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_heap_line, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_heap_line, col(C_DIM), 0);
  lv_obj_set_width(gW.diag_heap_line, 304);
  lv_label_set_long_mode(gW.diag_heap_line, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_heap_line, 8, 88);
  lv_label_set_text(gW.diag_heap_line, "heap/uptime --");

  gW.diag_axis_y = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_axis_y, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_axis_y, col(C_DIM), 0);
  lv_obj_set_width(gW.diag_axis_y, 80);
  lv_label_set_long_mode(gW.diag_axis_y, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_axis_y, 8, 100);
  lv_label_set_text(gW.diag_axis_y, "PPS");

  gW.diag_axis_x = lv_label_create(gW.page_diagnostics);
  lv_obj_set_style_text_font(gW.diag_axis_x, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.diag_axis_x, col(C_DIM), 0);
  lv_obj_set_width(gW.diag_axis_x, 100);
  lv_label_set_long_mode(gW.diag_axis_x, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.diag_axis_x, 216, 100);
  lv_label_set_text(gW.diag_axis_x, "Time");

  gW.diag_chart = lv_chart_create(gW.page_diagnostics);
  lv_obj_set_size(gW.diag_chart, 304, 60);
  lv_obj_set_pos(gW.diag_chart, 8, 116);
  lv_chart_set_type(gW.diag_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(gW.diag_chart, 48);
  lv_chart_set_range(gW.diag_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
  lv_chart_set_update_mode(gW.diag_chart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_obj_set_style_bg_color(gW.diag_chart, col(C_CARD), 0);
  lv_obj_set_style_border_color(gW.diag_chart, col(C_BORDER), 0);
  lv_obj_set_style_line_width(gW.diag_chart, 2, LV_PART_ITEMS);
  gW.diag_series_ctrl = lv_chart_add_series(gW.diag_chart, col(C_GREEN), LV_CHART_AXIS_PRIMARY_Y);
  gW.diag_series_telm = lv_chart_add_series(gW.diag_chart, col(C_RED), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_value(gW.diag_chart, gW.diag_series_ctrl, 0);
  lv_chart_set_all_value(gW.diag_chart, gW.diag_series_telm, 0);

  gW.diag_clear = makeButton(gW.page_diagnostics, "CLR CHART", 212, 70, 100, 22, onDiagClearChart, nullptr);

  gW.page_calibration = lv_obj_create(gW.content);
  lv_obj_remove_style_all(gW.page_calibration);
  lv_obj_set_size(gW.page_calibration, kScreenW, kContentH);
  lv_obj_set_pos(gW.page_calibration, 0, 0);
  lv_obj_set_style_bg_opa(gW.page_calibration, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.page_calibration, LV_OBJ_FLAG_SCROLLABLE);

  gW.cal_lx_label = lv_label_create(gW.page_calibration);
  lv_obj_set_style_text_font(gW.cal_lx_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.cal_lx_label, col(C_TEXT), 0);
  lv_obj_set_width(gW.cal_lx_label, 304);
  lv_label_set_long_mode(gW.cal_lx_label, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.cal_lx_label, 8, 8);
  lv_label_set_text(gW.cal_lx_label, "L.X --");

  gW.cal_lx_bar = lv_bar_create(gW.page_calibration);
  lv_obj_set_size(gW.cal_lx_bar, 304, 12);
  lv_obj_set_pos(gW.cal_lx_bar, 8, 24);
  lv_bar_set_range(gW.cal_lx_bar, -512, 512);
  lv_bar_set_value(gW.cal_lx_bar, 0, LV_ANIM_OFF);

  gW.cal_rx_label = lv_label_create(gW.page_calibration);
  lv_obj_set_style_text_font(gW.cal_rx_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.cal_rx_label, col(C_TEXT), 0);
  lv_obj_set_width(gW.cal_rx_label, 304);
  lv_label_set_long_mode(gW.cal_rx_label, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.cal_rx_label, 8, 42);
  lv_label_set_text(gW.cal_rx_label, "R.X --");

  gW.cal_rx_bar = lv_bar_create(gW.page_calibration);
  lv_obj_set_size(gW.cal_rx_bar, 304, 12);
  lv_obj_set_pos(gW.cal_rx_bar, 8, 58);
  lv_bar_set_range(gW.cal_rx_bar, -512, 512);
  lv_bar_set_value(gW.cal_rx_bar, 0, LV_ANIM_OFF);

  gW.cal_ry_label = lv_label_create(gW.page_calibration);
  lv_obj_set_style_text_font(gW.cal_ry_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.cal_ry_label, col(C_TEXT), 0);
  lv_obj_set_width(gW.cal_ry_label, 304);
  lv_label_set_long_mode(gW.cal_ry_label, LV_LABEL_LONG_DOT);
  lv_obj_set_pos(gW.cal_ry_label, 8, 76);
  lv_label_set_text(gW.cal_ry_label, "R.Y --");

  gW.cal_ry_bar = lv_bar_create(gW.page_calibration);
  lv_obj_set_size(gW.cal_ry_bar, 304, 12);
  lv_obj_set_pos(gW.cal_ry_bar, 8, 92);
  lv_bar_set_range(gW.cal_ry_bar, -512, 512);
  lv_bar_set_value(gW.cal_ry_bar, 0, LV_ANIM_OFF);

  gW.cal_pid_label = lv_label_create(gW.page_calibration);
  lv_obj_set_style_text_font(gW.cal_pid_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gW.cal_pid_label, col(C_DIM), 0);
  lv_obj_set_width(gW.cal_pid_label, 304);
  lv_label_set_long_mode(gW.cal_pid_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_pos(gW.cal_pid_label, 8, 110);
  lv_label_set_text(gW.cal_pid_label, "PID template (Roll/Pitch/Yaw): 1.00 / 1.00 / 1.00");

  gW.cal_set_center = makeButton(gW.page_calibration, "SET CENTER", 8, 146, 146, 26, onCalSetCenter, nullptr);
  gW.cal_save_template = makeButton(gW.page_calibration, "SAVE TEMPLATE", 166, 146, 146, 26, onCalSaveTemplate, nullptr);

  gW.page_settings = lv_obj_create(gW.content);
  lv_obj_remove_style_all(gW.page_settings);
  lv_obj_set_size(gW.page_settings, kScreenW, kContentH);
  lv_obj_set_pos(gW.page_settings, 0, 0);
  lv_obj_set_style_bg_opa(gW.page_settings, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(gW.page_settings, LV_OBJ_FLAG_SCROLLABLE);

  constexpr int16_t kSetupBackY = 4;
  constexpr int16_t kSetupBackH = 22;
  constexpr int16_t kSetupListY = 30;
  constexpr int16_t kSetupListH = 146;
  gW.settings_page_back = makeButton(gW.page_settings,
                                     "BACK",
                                     4,
                                     kSetupBackY,
                                     72,
                                     kSetupBackH,
                                     onFocusActivate,
                                     reinterpret_cast<void*>(static_cast<uintptr_t>(UiFocusId::SETTINGS_PAGE_BACK)));

  gW.settings_list = lv_list_create(gW.page_settings);
  lv_obj_set_size(gW.settings_list, 312, kSetupListH);
  lv_obj_set_pos(gW.settings_list, 4, kSetupListY);
  lv_obj_set_style_bg_color(gW.settings_list, col(C_CARD), 0);
  lv_obj_set_style_border_color(gW.settings_list, col(C_BORDER), 0);
  lv_obj_set_style_border_width(gW.settings_list, 1, 0);
  lv_obj_set_style_pad_row(gW.settings_list, 4, 0);
  lv_obj_set_style_text_color(gW.settings_list, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.settings_list, &lv_font_montserrat_12, 0);

  auto addListRow = [](lv_obj_t* list, const char* text, UiFocusId focus) {
    lv_obj_t* btn = lv_list_add_btn(list, nullptr, text);
    lv_obj_set_style_bg_color(btn, col(C_CARD), 0);
    lv_obj_set_style_border_color(btn, col(C_BORDER), 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_text_color(btn, col(C_TEXT), 0);
    lv_obj_set_style_text_font(btn, &lv_font_montserrat_12, 0);
    lv_obj_add_event_cb(btn,
                        onFocusActivate,
                        LV_EVENT_CLICKED,
                        reinterpret_cast<void*>(static_cast<uintptr_t>(focus)));
    return btn;
  };

  lv_list_add_text(gW.settings_list, "Setup");
  gW.settings_home_brightness = addListRow(gW.settings_list, "Brightness", UiFocusId::SETTINGS_HOME_BRIGHTNESS);
  gW.settings_home_ctrl = addListRow(gW.settings_list, "NRF CTRL", UiFocusId::SETTINGS_HOME_CTRL);
  gW.settings_home_telm = addListRow(gW.settings_list, "NRF TELEM", UiFocusId::SETTINGS_HOME_TELM);

  gW.settings_sub_brightness = lv_list_create(gW.page_settings);
  lv_obj_set_size(gW.settings_sub_brightness, 312, kSetupListH);
  lv_obj_set_pos(gW.settings_sub_brightness, 4, kSetupListY);
  lv_obj_set_style_bg_color(gW.settings_sub_brightness, col(C_CARD), 0);
  lv_obj_set_style_border_color(gW.settings_sub_brightness, col(C_BORDER), 0);
  lv_obj_set_style_border_width(gW.settings_sub_brightness, 1, 0);
  lv_obj_set_style_pad_row(gW.settings_sub_brightness, 4, 0);
  lv_obj_set_style_text_color(gW.settings_sub_brightness, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.settings_sub_brightness, &lv_font_montserrat_12, 0);
  lv_list_add_text(gW.settings_sub_brightness, "Brightness");
  gW.settings_bright_value = addListRow(gW.settings_sub_brightness, "Brightness: --", UiFocusId::SETTINGS_BRIGHT_VALUE);
  gW.settings_bright_sleep_enable = addListRow(gW.settings_sub_brightness,
                                                "Sleep: --",
                                                UiFocusId::SETTINGS_BRIGHT_SLEEP_ENABLE);
  gW.settings_bright_sleep_timeout = addListRow(gW.settings_sub_brightness,
                                                 "Sleep Timeout: --",
                                                 UiFocusId::SETTINGS_BRIGHT_SLEEP_TIMEOUT);
  gW.settings_bright_apply = addListRow(gW.settings_sub_brightness, "Apply Changes", UiFocusId::SETTINGS_BRIGHT_APPLY);
  gW.settings_bright_back = addListRow(gW.settings_sub_brightness, "Back", UiFocusId::SETTINGS_BRIGHT_BACK);

  gW.settings_sub_ctrl = lv_list_create(gW.page_settings);
  lv_obj_set_size(gW.settings_sub_ctrl, 312, kSetupListH);
  lv_obj_set_pos(gW.settings_sub_ctrl, 4, kSetupListY);
  lv_obj_set_style_bg_color(gW.settings_sub_ctrl, col(C_CARD), 0);
  lv_obj_set_style_border_color(gW.settings_sub_ctrl, col(C_BORDER), 0);
  lv_obj_set_style_border_width(gW.settings_sub_ctrl, 1, 0);
  lv_obj_set_style_pad_row(gW.settings_sub_ctrl, 4, 0);
  lv_obj_set_style_text_color(gW.settings_sub_ctrl, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.settings_sub_ctrl, &lv_font_montserrat_12, 0);
  lv_list_add_text(gW.settings_sub_ctrl, "NRF CTRL");
  lv_obj_t* ctrl_info = lv_list_add_text(gW.settings_sub_ctrl, "Stress TX mode is compile-time");
  lv_obj_set_style_text_color(ctrl_info, col(C_DIM), 0);
  gW.settings_ctrl_channel = addListRow(gW.settings_sub_ctrl, "CTRL CH: --", UiFocusId::SETTINGS_CTRL_CHANNEL);
  gW.settings_ctrl_dr = addListRow(gW.settings_sub_ctrl, "CTRL DR: --", UiFocusId::SETTINGS_CTRL_DR);
  gW.settings_ctrl_pa = addListRow(gW.settings_sub_ctrl, "CTRL PA: --", UiFocusId::SETTINGS_CTRL_PA);
  gW.settings_ctrl_ack = addListRow(gW.settings_sub_ctrl, "CTRL ACK: --", UiFocusId::SETTINGS_CTRL_ACK);
  gW.settings_ctrl_retry_delay = addListRow(gW.settings_sub_ctrl,
                                             "CTRL Retry Delay: --",
                                             UiFocusId::SETTINGS_CTRL_RETRY_DELAY);
  gW.settings_ctrl_retry_count = addListRow(gW.settings_sub_ctrl,
                                             "CTRL Retry Count: --",
                                             UiFocusId::SETTINGS_CTRL_RETRY_COUNT);
  gW.settings_ctrl_apply = addListRow(gW.settings_sub_ctrl, "Apply Changes", UiFocusId::SETTINGS_CTRL_APPLY);
  gW.settings_ctrl_back = addListRow(gW.settings_sub_ctrl, "Back", UiFocusId::SETTINGS_CTRL_BACK);

  gW.settings_sub_telm = lv_list_create(gW.page_settings);
  lv_obj_set_size(gW.settings_sub_telm, 312, kSetupListH);
  lv_obj_set_pos(gW.settings_sub_telm, 4, kSetupListY);
  lv_obj_set_style_bg_color(gW.settings_sub_telm, col(C_CARD), 0);
  lv_obj_set_style_border_color(gW.settings_sub_telm, col(C_BORDER), 0);
  lv_obj_set_style_border_width(gW.settings_sub_telm, 1, 0);
  lv_obj_set_style_pad_row(gW.settings_sub_telm, 4, 0);
  lv_obj_set_style_text_color(gW.settings_sub_telm, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.settings_sub_telm, &lv_font_montserrat_12, 0);
  lv_list_add_text(gW.settings_sub_telm, "NRF TELEM");
  gW.settings_telm_channel = addListRow(gW.settings_sub_telm, "TELM CH: --", UiFocusId::SETTINGS_TELM_CHANNEL);
  gW.settings_telm_dr = addListRow(gW.settings_sub_telm, "TELM DR: --", UiFocusId::SETTINGS_TELM_DR);
  gW.settings_telm_pa = addListRow(gW.settings_sub_telm, "TELM PA: --", UiFocusId::SETTINGS_TELM_PA);
  gW.settings_telm_ack = addListRow(gW.settings_sub_telm, "TELM ACK: --", UiFocusId::SETTINGS_TELM_ACK);
  gW.settings_telm_retry_delay = addListRow(gW.settings_sub_telm,
                                             "TELM Retry Delay: --",
                                             UiFocusId::SETTINGS_TELM_RETRY_DELAY);
  gW.settings_telm_retry_count = addListRow(gW.settings_sub_telm,
                                             "TELM Retry Count: --",
                                             UiFocusId::SETTINGS_TELM_RETRY_COUNT);
  gW.settings_telm_apply = addListRow(gW.settings_sub_telm, "Apply Changes", UiFocusId::SETTINGS_TELM_APPLY);
  gW.settings_telm_back = addListRow(gW.settings_sub_telm, "Back", UiFocusId::SETTINGS_TELM_BACK);

  gW.sleep_overlay = lv_obj_create(scr);
  lv_obj_remove_style_all(gW.sleep_overlay);
  lv_obj_set_size(gW.sleep_overlay, kScreenW, kScreenH);
  lv_obj_set_pos(gW.sleep_overlay, 0, 0);
  lv_obj_set_style_bg_color(gW.sleep_overlay, col(0x000000), 0);
  lv_obj_set_style_bg_opa(gW.sleep_overlay, LV_OPA_80, 0);
  lv_obj_clear_flag(gW.sleep_overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(gW.sleep_overlay, LV_OBJ_FLAG_HIDDEN);

  gW.sleep_label = lv_label_create(gW.sleep_overlay);
  lv_obj_set_style_text_color(gW.sleep_label, col(C_TEXT), 0);
  lv_obj_set_style_text_font(gW.sleep_label, &lv_font_montserrat_16, 0);
  lv_obj_set_width(gW.sleep_label, 280);
  lv_label_set_long_mode(gW.sleep_label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(gW.sleep_label, "Sleep Mode\nMove stick or press button to wake");
  lv_obj_align(gW.sleep_label, LV_ALIGN_CENTER, 0, 0);

  setPage(UiPageId::DASHBOARD);
  gUi.filter = TelemFilter::ALL;
  updateFilterButtonsVisual();
  gUi.settings_subpage = SettingsSubpage::HOME;
  gUi.radio_edit_loaded = false;
  gUi.radio_dirty = false;
  gUi.brightness_dirty = false;
  gUi.radio_apply_ok = false;
  gUi.brightness_apply_ok = false;
  gUi.awake_backlight_duty = dutyFromPercent(UI_DEFAULT_BRIGHTNESS_PCT);
  gUi.last_user_activity_ms = millis();
  gUi.sleeping = false;
  showSleepOverlay(false);
  refreshSettingsEditorLabels();
}

void uiInit() {
  displayBeginRobust();
  initBacklightPwm();
  setBacklightDuty(dutyFromPercent(UI_DEFAULT_BRIGHTNESS_PCT));

  initLvgl();
  buildUi();

  telemetryPush(TelemKind::INFO, "UI boot complete");
  refreshTelemetryLog();
  refreshDashboardPreview();

  SharedState s = {};
  if (fetchState(&s)) {
    setBacklightDuty(s.settings.backlight_duty);
    applyStateToUi(s, millis());
  }

  LOG_INF("UI task ready (new LVGL page model)");
}

void uiLoop() {
  uiInit();
  uint32_t last_tick = millis();

  for (;;) {
    const uint32_t now_ms = millis();
    const uint32_t delta = static_cast<uint32_t>(now_ms - last_tick);
    if (delta > 0U) {
      lv_tick_inc(delta);
      last_tick = now_ms;
    }

    if ((now_ms - gUi.lastStatePollMs) >= kStatePollPeriodMs) {
      gUi.lastStatePollMs = now_ms;
      SharedState snapshot = {};
      if (fetchState(&snapshot)) {
        applyStateToUi(snapshot, now_ms);
      }
    }

    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

void uiTaskEntry(void* p) {
  (void)p;
  uiLoop();
}

}  // namespace

void uiPrepareDisplayBoot() {
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, kBacklightActiveHigh ? LOW : HIGH);
}

bool uiStart(SharedState* shared_state, SemaphoreHandle_t state_mutex, QueueHandle_t command_queue) {
  if ((shared_state == nullptr) || (state_mutex == nullptr) || (command_queue == nullptr)) {
    return false;
  }
  if (gCtx.running) {
    return true;
  }

  gCtx.state = shared_state;
  gCtx.mutex = state_mutex;
  gCtx.cmdq = command_queue;

  const BaseType_t ok = xTaskCreatePinnedToCore(
      uiTaskEntry,
      "ui_task",
      12288,
      nullptr,
      1,
      &gCtx.task,
      0);

  gCtx.running = (ok == pdPASS);
  if (!gCtx.running) {
    LOG_ERR("Failed to create UI task");
  }
  return gCtx.running;
}
