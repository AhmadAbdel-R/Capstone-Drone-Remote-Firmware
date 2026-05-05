# Capstone Drone Remote Firmware

This repository contains firmware for the custom drone remote controller.

The active firmware project is currently in:

- `DUAL-NRF-REMOTE-FIRMWARE/`

## Main Capabilities

- ESP32-based remote firmware
- Dual nRF24 links (control + telemetry)
- TFT/LVGL user interface
- Joystick and battery input handling

## Quick Start

1. Open `DUAL-NRF-REMOTE-FIRMWARE` in PlatformIO.
2. Build and upload for your target board.
3. Open serial monitor and verify link/UI startup.

## Why This Layout

The root repo tracks portfolio and project-level history.
The nested project folder keeps firmware sources isolated and reproducible.
