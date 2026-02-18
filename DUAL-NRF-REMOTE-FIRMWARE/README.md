# DUAL-NRF Remote Firmware

This project runs on an ESP32-based remote controller with:
- 2.0 inch TFT display (LVGL UI)
- Dual-nRF24 radio links (control + telemetry)
- Two joysticks + button
- Battery ADC measurement

It is built with PlatformIO and the Arduino framework.

## 1) What This Firmware Does (Non-Coder Version)

Think of this firmware as 4 jobs running together:

1. Read your physical inputs  
   It reads joystick positions, button state, and battery voltage.

2. Handle radio links  
   It sends/receives RF packets and tracks link health.

3. Keep a shared "live status" model  
   A central shared struct (`SharedState`) stores everything the UI needs.

4. Draw/update the display UI  
   The screen shows pages for Dashboard, Telemetry, Flight, Diagnostics, Calibration, and Setup.

## 2) Project Layout

### Root
- `platformio.ini`: build config, libraries, flags.
- `README.md`: this file.

### include/
- `config.h`: tuning values, defaults, constants.
- `state.h`: shared state model and UI command types.
- `input.h`: input snapshot types + input API.
- `radio_link.h`: radio runtime settings + radio APIs.
- `ui.h`: UI task API.
- `lv_conf.h`: LVGL configuration and fonts.

### src/
- `main.cpp`: app entrypoint, task setup, state publishing, command processing.
- `input.cpp`: ADC + joystick processing (includes battery voltage/percent estimation).
- `inputs.cpp`: compatibility wrapper.
- `radio_link.cpp`: RF24 link implementation and runtime settings application.
- `ui.cpp`: full LVGL UI implementation and joystick navigation logic.

### src/pages/
These are page module marker headers for repository organization:
- `src/pages/dashboard_page.h`
- `src/pages/telemetry_page.h`
- `src/pages/flight_page.h`
- `src/pages/diagnostics_page.h`
- `src/pages/calibration_page.h`
- `src/pages/settings_page.h`

Current active code is still centralized in `src/ui.cpp`.

### reference/
- `reference/fcu_reference_template.cpp`: FCU-side reciprocal example code.
  - Intentionally **not built** (outside `/src`).
  - Wrapped in `#if 0` for reference-only use.

## 3) UI Pages Explained

## Dashboard
- Shows main status: battery, GPS, altitude, distance, mode, link quality.
- Includes FCU status card and firmware label.

## Telemetry
- Shows incoming text feed.
- Filter buttons: ALL / LINK / GPS / WARN.

## Flight
- Arm status and progress bar.
- Arming combo: hold both sticks down.
- Mode control and guarded takeoff action.

## Diagnostics
- Detailed link/counter lines.
- Smoothed TX-rate chart.
- Axis labels (`PPS` and `Time`) added without clutter.

## Calibration
- Live stick mapped/raw values.
- Visual bars for each axis.
- Template actions.

## Setup
- Dedicated setup workspace.
- Back button required to leave Setup and return to normal page navigation.
- Subpages:
  - Brightness
  - NRF CTRL
  - NRF TELEM
- Apply buttons:
  - White by default
  - Green after successful apply trigger
  - Reset to white after exiting Setup and coming back

## 4) Navigation Model

- Left stick: moves focus (selection highlight).
- Left stick button: activates focused item.
- Right stick: edits numeric/toggle setting values in Setup subpages.

Setup behavior is intentionally isolated:
- While inside Setup, top tab cycling is disabled from stick navigation.
- Use Setup `BACK` to exit Setup first, then choose other pages.

## 5) Battery Reading (ADC -> Real Percent)

Battery reading is now based on actual ADC voltage path:

1. ADC millivolts read (`analogReadMilliVolts` on ESP32, median filtered).
2. Divider compensation applied using:
   - `BATTERY_DIVIDER_R1_OHM`
   - `BATTERY_DIVIDER_R2_OHM`
3. Optional calibration factor applied:
   - `ADC_BATTERY_CAL_SCALE_NUM`
   - `ADC_BATTERY_CAL_SCALE_DEN`
4. EMA smoothing applied to reduce noise.
5. Percent computed from voltage curve in `src/input.cpp` (`kBatteryCurve`).

This is not a random/synthetic percentage; it is ADC-derived and then curve-mapped.

## 6) Build and Run

From project root:

```powershell
platformio run
```

Upload (example):

```powershell
platformio run -t upload
```

Monitor serial:

```powershell
platformio device monitor -b 115200
```

## 7) How Main Data Flow Works

1. `input.cpp` captures sticks/button/battery.
2. `radio_link.cpp` captures RF link status/stats/runtime.
3. `main.cpp` combines those into `SharedState`.
4. `ui.cpp` polls `SharedState` and redraws UI.
5. UI actions send `UiCommand` messages back to `main.cpp`.
6. `main.cpp` applies commands (arm/mode/brightness/radio settings).

## 8) Safe Extension Tips

- Add new user-visible settings to:
  - `state.h` (`UiCommand` or settings model)
  - `main.cpp` command processor
  - `ui.cpp` controls + labels
- Keep UI changes non-blocking; UI runs as a FreeRTOS task.
- Keep battery and joystick filtering in `input.cpp` to avoid duplicated logic.

