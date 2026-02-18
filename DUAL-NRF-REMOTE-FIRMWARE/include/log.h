#pragma once

#include <Arduino.h>

#include "config.h"

#define LOG_OK(fmt, ...)   do { Serial.printf("[OK] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_WARN(fmt, ...) do { Serial.printf("[!] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERR(fmt, ...)  do { Serial.printf("[X] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_INF(fmt, ...)  do { Serial.printf("[i] " fmt "\n", ##__VA_ARGS__); } while (0)

#if DEBUG_LOG_ENABLE
#define LOG_DBG(fmt, ...)  do { Serial.printf("[dbg] " fmt "\n", ##__VA_ARGS__); } while (0)
#else
#define LOG_DBG(fmt, ...)  do { } while (0)
#endif
