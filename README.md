# Capstone Drone Remote Firmware

Firmware repository for the handheld drone remote.

## Active Firmware Folder

- `DUAL-NRF-REMOTE-FIRMWARE/`

## Stack

- ESP32-based firmware
- dual nRF24 links (control + telemetry)
- TFT UI (LVGL)
- joystick/button/battery input pipeline

## Notes

- Root repo tracks history and portfolio context.
- The nested folder is the buildable PlatformIO project.

## Build

1. Open `DUAL-NRF-REMOTE-FIRMWARE` in PlatformIO.
2. Build and upload for your board target.
3. Monitor serial at `115200` for runtime diagnostics.
