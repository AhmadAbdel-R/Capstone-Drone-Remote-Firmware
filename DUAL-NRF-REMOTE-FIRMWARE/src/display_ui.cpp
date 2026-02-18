#include "display_ui.h"

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "log.h"
#include "pins.h"
#include "radio_link.h"

namespace {

TFT_eSPI tft;
bool ui_ready = false;
uint32_t last_ui_update_ms = 0;
uint32_t last_pps_update_ms = 0;
uint32_t last_ctrl_tx_count = 0;
uint32_t ctrl_tx_pps = 0;

constexpr uint8_t kRowCount = 5;
constexpr uint16_t kRowX = 8;
constexpr uint16_t kRowHeight = 24;
constexpr uint16_t kRowY[kRowCount] = {40, 66, 92, 118, 144};

char row_cache[kRowCount][80] = {};
uint16_t color_cache[kRowCount] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

void setBacklight(bool on) {
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, on ? HIGH : LOW);
}

void applyLandscapeFlipWorkaroundIfNeeded() {
#if defined(ST7789_DRIVER)
  if (TFT_FORCE_LANDSCAPE_FLIP_WITH_ROT1 && (TFT_ROTATION == 1)) {
    // Keep rotation=1 window mapping (no left-gap) but use flipped landscape MADCTL.
    const uint8_t madctl = static_cast<uint8_t>(TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);
    tft.writecommand(TFT_MADCTL);
    tft.writedata(madctl);
  }
#endif
}

void applySt7789PanelFormatAndInversion() {
#if defined(ST7789_DRIVER)
  // Re-assert panel mode after rotation/MADCTL overrides.
  tft.writecommand(TFT_COLMOD);
  tft.writedata(0x55);  // 16-bit/pixel (RGB565)
  tft.writecommand(UI_TFT_INVERT_COLORS ? TFT_INVON : TFT_INVOFF);
#endif
}

void drawRow(uint8_t row, const char* text, uint16_t color, bool force = false) {
  if (row >= kRowCount) {
    return;
  }

  if (!force && (strcmp(row_cache[row], text) == 0) && (color_cache[row] == color)) {
    return;
  }

  tft.fillRect(0, kRowY[row], 320, kRowHeight, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(kRowX, kRowY[row] + 4);
  tft.print(text);

  strncpy(row_cache[row], text, sizeof(row_cache[row]) - 1);
  row_cache[row][sizeof(row_cache[row]) - 1] = '\0';
  color_cache[row] = color;
}

void updateCtrlTxPps(uint32_t now_ms, uint32_t ctrl_tx_count) {
  if (last_pps_update_ms == 0) {
    last_pps_update_ms = now_ms;
    last_ctrl_tx_count = ctrl_tx_count;
    ctrl_tx_pps = 0;
    return;
  }

  const uint32_t dt_ms = static_cast<uint32_t>(now_ms - last_pps_update_ms);
  if (dt_ms == 0) {
    return;
  }

  const uint32_t delta = static_cast<uint32_t>(ctrl_tx_count - last_ctrl_tx_count);
  ctrl_tx_pps = static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000ULL) / dt_ms);

  last_pps_update_ms = now_ms;
  last_ctrl_tx_count = ctrl_tx_count;
}

}  // namespace

void displayUiInit() {
  // TFT_eSPI uses global SPI on ESP32. Configure for VSPI pin map.
  SPI.begin(PIN_TFT_SCK, PIN_TFT_MISO, PIN_TFT_MOSI, PIN_TFT_CS);

  setBacklight(true);

  tft.init();
  tft.setRotation(TFT_ROTATION);
  applyLandscapeFlipWorkaroundIfNeeded();
  applySt7789PanelFormatAndInversion();
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(8, 10);
  tft.print("DUAL NRF LINK TEST");

  memset(row_cache, 0, sizeof(row_cache));
  for (uint8_t i = 0; i < kRowCount; ++i) {
    color_cache[i] = 0xFFFF;
  }

  ui_ready = true;
  LOG_INF("Display init: ST7789 320x240 rot=%u", static_cast<unsigned>(TFT_ROTATION));
}

void displayUiTick(uint16_t local_battery_mv) {
  if (!ui_ready) {
    return;
  }

  const uint32_t now = millis();
  if ((now - last_ui_update_ms) < UI_REFRESH_PERIOD_MS) {
    return;
  }
  last_ui_update_ms = now;

  RadioLinkUiSnapshot link = {};
  RadioLinkStatsSnapshot ctrl_stats = {};
  RadioLinkStatsSnapshot telm_stats = {};
  radioLinkGetUiSnapshot(&link);
  radioLinkGetStatsSnapshot(&ctrl_stats, &telm_stats);

  updateCtrlTxPps(now, ctrl_stats.tx_count);

  char row0[80];
  char row1[80];
  char row2[80];
  char row3[80];
  char row4[80];
  uint16_t row2_color = TFT_WHITE;

  snprintf(row0, sizeof(row0), "Role: %s  Mode: %s",
           link.is_remote_role ? "REMOTE" : "FCU",
           link.stress_mode_enabled ? "STRESS" : "NORMAL");

  snprintf(row1, sizeof(row1), "CTRL TX: %lu (%lu pps)",
           static_cast<unsigned long>(ctrl_stats.tx_count),
           static_cast<unsigned long>(ctrl_tx_pps));

  if (link.is_remote_role) {
    snprintf(row2, sizeof(row2), "TELM link: %s", link.telm_link_ok ? "OK" : "LOST");
    row2_color = link.telm_link_ok ? TFT_GREEN : TFT_RED;
    if (link.telm_valid) {
      snprintf(row3, sizeof(row3), "Last TELM seq: %lu",
               static_cast<unsigned long>(link.telm_seq));
    } else {
      snprintf(row3, sizeof(row3), "Last TELM seq: n/a");
    }
  } else {
    snprintf(row2, sizeof(row2), "TELM link: TX role");
    row2_color = TFT_WHITE;
    snprintf(row3, sizeof(row3), "Last TELM seq: tx=%lu",
             static_cast<unsigned long>(telm_stats.tx_count));
  }

  snprintf(row4, sizeof(row4), "Local batt: %u.%03u V",
           static_cast<unsigned>(local_battery_mv / 1000U),
           static_cast<unsigned>(local_battery_mv % 1000U));

  drawRow(0, row0, TFT_WHITE);
  drawRow(1, row1, TFT_YELLOW);
  drawRow(2, row2, row2_color);
  drawRow(3, row3, TFT_CYAN);
  drawRow(4, row4, TFT_MAGENTA);
}
