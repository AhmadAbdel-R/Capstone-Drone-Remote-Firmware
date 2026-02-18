#include "input.h"

#include <Arduino.h>

#include "config.h"
#include "pins.h"

namespace {

constexpr uint8_t kMaxMedianSamples = 15;
constexpr int32_t kRawCenter = 2048;
constexpr int32_t kRawHalfSpan = 2048;
constexpr int16_t kMappedSpan = 512;
constexpr uint8_t kBatteryEmaAlphaNum = 1;
constexpr uint8_t kBatteryEmaAlphaDen = 5;

struct AxisFilterState {
  bool initialized = false;
  int32_t ema_raw = 0;
  int16_t mapped = 0;
};

AxisFilterState gLeftXFilter = {};
AxisFilterState gRightXFilter = {};
AxisFilterState gRightYFilter = {};

bool gBatteryInitialized = false;
uint32_t gBatteryMvEma = 0;

struct BatteryPoint {
  uint16_t mv;
  uint8_t pct;
};

constexpr BatteryPoint kBatteryCurve[] = {
    {3300, 0},
    {3500, 5},
    {3600, 12},
    {3700, 25},
    {3750, 35},
    {3800, 48},
    {3850, 60},
    {3900, 72},
    {3950, 82},
    {4000, 90},
    {4100, 96},
    {4200, 100},
};

uint16_t readAdcOnceSettled(uint8_t pin) {
  (void)analogRead(pin);
  return static_cast<uint16_t>(analogRead(pin));
}

void sortU16(uint16_t* data, uint8_t len) {
  if (data == nullptr) {
    return;
  }
  for (uint8_t i = 1; i < len; ++i) {
    const uint16_t key = data[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && data[j] > key) {
      data[j + 1] = data[j];
      --j;
    }
    data[j + 1] = key;
  }
}

uint16_t readAdcMedian(uint8_t pin) {
  uint8_t n = INPUT_ADC_MEDIAN_SAMPLES;
  if (n < 3U) {
    n = 3U;
  } else if (n > kMaxMedianSamples) {
    n = kMaxMedianSamples;
  }
  if ((n & 0x1U) == 0U) {
    n = static_cast<uint8_t>(n - 1U);
  }

  uint16_t samples[kMaxMedianSamples] = {};
  for (uint8_t i = 0; i < n; ++i) {
    samples[i] = readAdcOnceSettled(pin);
  }
  sortU16(samples, n);
  return samples[n / 2U];
}

uint16_t rawToAdcMv(uint16_t raw) {
  const uint32_t mv = (static_cast<uint32_t>(raw) * ADC_VREF_MV) / ADC_MAX_COUNT;
  return static_cast<uint16_t>(mv);
}

uint16_t readAdcMilliVoltsOnceSettled(uint8_t pin) {
#if defined(ARDUINO_ARCH_ESP32)
  // Discard the first conversion after channel/switching effects, then read calibrated mV.
  (void)analogReadMilliVolts(pin);
  const uint32_t mv = static_cast<uint32_t>(analogReadMilliVolts(pin));
  return static_cast<uint16_t>((mv > 0xFFFFUL) ? 0xFFFFUL : mv);
#else
  return rawToAdcMv(readAdcOnceSettled(pin));
#endif
}

uint16_t readAdcMilliVoltsMedian(uint8_t pin) {
  uint8_t n = INPUT_ADC_MEDIAN_SAMPLES;
  if (n < 3U) {
    n = 3U;
  } else if (n > kMaxMedianSamples) {
    n = kMaxMedianSamples;
  }
  if ((n & 0x1U) == 0U) {
    n = static_cast<uint8_t>(n - 1U);
  }

  uint16_t samples[kMaxMedianSamples] = {};
  for (uint8_t i = 0; i < n; ++i) {
    samples[i] = readAdcMilliVoltsOnceSettled(pin);
  }
  sortU16(samples, n);
  return samples[n / 2U];
}

int16_t mapAxis(uint16_t raw) {
  int32_t centered = static_cast<int32_t>(raw) - kRawCenter;
  centered = (centered * kMappedSpan) / kRawHalfSpan;
  if (centered > kMappedSpan) {
    centered = kMappedSpan;
  } else if (centered < -kMappedSpan) {
    centered = -kMappedSpan;
  }

  if (centered < static_cast<int32_t>(JOYSTICK_DEADZONE_COUNTS) &&
      centered > -static_cast<int32_t>(JOYSTICK_DEADZONE_COUNTS)) {
    centered = 0;
  }
  return static_cast<int16_t>(centered);
}

int16_t applyAxisFilter(uint16_t raw, AxisFilterState* state) {
  if (state == nullptr) {
    return mapAxis(raw);
  }

  if (!state->initialized) {
    state->initialized = true;
    state->ema_raw = static_cast<int32_t>(raw);
    state->mapped = mapAxis(raw);
    return state->mapped;
  }

  static_assert(INPUT_AXIS_EMA_ALPHA_DEN > 0, "INPUT_AXIS_EMA_ALPHA_DEN must be > 0.");
  const int32_t delta_raw = static_cast<int32_t>(raw) - state->ema_raw;
  state->ema_raw += (delta_raw * static_cast<int32_t>(INPUT_AXIS_EMA_ALPHA_NUM)) /
                    static_cast<int32_t>(INPUT_AXIS_EMA_ALPHA_DEN);

  const int16_t target = mapAxis(static_cast<uint16_t>(state->ema_raw));
  int16_t delta_map = static_cast<int16_t>(target - state->mapped);

  if (delta_map > INPUT_AXIS_SLEW_STEP) {
    delta_map = INPUT_AXIS_SLEW_STEP;
  } else if (delta_map < -INPUT_AXIS_SLEW_STEP) {
    delta_map = -INPUT_AXIS_SLEW_STEP;
  }

  state->mapped = static_cast<int16_t>(state->mapped + delta_map);
  return state->mapped;
}

uint16_t readBatteryInternalMv() {
  const uint32_t adc_mv = readAdcMilliVoltsMedian(PIN_BATTERY_ADC);

  const uint32_t divider_num = BATTERY_DIVIDER_R1_OHM + BATTERY_DIVIDER_R2_OHM;
  const uint32_t batt_mv = (adc_mv * divider_num) / BATTERY_DIVIDER_R2_OHM;
  const uint32_t calibrated_mv = (batt_mv * ADC_BATTERY_CAL_SCALE_NUM) / ADC_BATTERY_CAL_SCALE_DEN;
  const uint16_t bounded_mv = static_cast<uint16_t>((calibrated_mv > 0xFFFFUL) ? 0xFFFFUL : calibrated_mv);

  // Direct ADC-based battery estimate with EMA smoothing.
  static_assert(kBatteryEmaAlphaDen > 0, "kBatteryEmaAlphaDen must be > 0.");
  if (!gBatteryInitialized) {
    gBatteryInitialized = true;
    gBatteryMvEma = bounded_mv;
  } else {
    const int32_t delta = static_cast<int32_t>(bounded_mv) - static_cast<int32_t>(gBatteryMvEma);
    gBatteryMvEma += (delta * static_cast<int32_t>(kBatteryEmaAlphaNum)) /
                     static_cast<int32_t>(kBatteryEmaAlphaDen);
  }

  return static_cast<uint16_t>(gBatteryMvEma);
}

}  // namespace

void inputInit() {
  analogReadResolution(12);

  // Battery sense uses a 1:2 divider (pins.h), so 4.2V cell -> ~2.1V ADC input.
  // 6 dB attenuation is a better fit for this range than 11 dB.
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_6db);
  analogSetPinAttenuation(PIN_LEFT_JOY_X, ADC_11db);
  analogSetPinAttenuation(PIN_RIGHT_JOY_X, ADC_11db);
  analogSetPinAttenuation(PIN_RIGHT_JOY_Y, ADC_11db);

  pinMode(PIN_LEFT_JOY_BTN, INPUT_PULLUP);
}

uint16_t inputReadBatteryMv() {
  return readBatteryInternalMv();
}

uint8_t inputBatteryPercent(uint16_t battery_mv) {
  if (battery_mv <= kBatteryCurve[0].mv) {
    return kBatteryCurve[0].pct;
  }

  const size_t last = (sizeof(kBatteryCurve) / sizeof(kBatteryCurve[0])) - 1U;
  if (battery_mv >= kBatteryCurve[last].mv) {
    return kBatteryCurve[last].pct;
  }

  for (size_t i = 1; i <= last; ++i) {
    const BatteryPoint lo = kBatteryCurve[i - 1];
    const BatteryPoint hi = kBatteryCurve[i];
    if (battery_mv <= hi.mv) {
      const uint32_t range_mv = static_cast<uint32_t>(hi.mv - lo.mv);
      if (range_mv == 0U) {
        return hi.pct;
      }

      const uint32_t pos_mv = static_cast<uint32_t>(battery_mv - lo.mv);
      const int32_t delta_pct = static_cast<int32_t>(hi.pct) - static_cast<int32_t>(lo.pct);
      const int32_t pct = static_cast<int32_t>(lo.pct) +
                          static_cast<int32_t>((static_cast<int64_t>(delta_pct) * pos_mv) /
                                               static_cast<int64_t>(range_mv));
      if (pct < 0) {
        return 0;
      }
      if (pct > 100) {
        return 100;
      }
      return static_cast<uint8_t>(pct);
    }
  }

  return 0;
}

void inputReadSnapshot(InputSnapshot* out) {
  if (out == nullptr) {
    return;
  }

  const uint16_t batt_mv = readBatteryInternalMv();
  const uint16_t left_x = readAdcMedian(PIN_LEFT_JOY_X);
  const uint16_t right_x = readAdcMedian(PIN_RIGHT_JOY_X);
  const uint16_t right_y = readAdcMedian(PIN_RIGHT_JOY_Y);

  out->battery_mv = batt_mv;
  out->battery_pct = inputBatteryPercent(batt_mv);

  out->left_x_raw = left_x;
  out->right_x_raw = right_x;
  out->right_y_raw = right_y;

  out->left_x_mapped = applyAxisFilter(left_x, &gLeftXFilter);
  // Physical joystick is rotated 90deg in this build: logical X comes from physical Y and vice versa.
  out->right_x_mapped = applyAxisFilter(right_y, &gRightXFilter);
  out->right_y_mapped = applyAxisFilter(right_x, &gRightYFilter);

  out->left_button_pressed = (digitalRead(PIN_LEFT_JOY_BTN) == LOW);
}
