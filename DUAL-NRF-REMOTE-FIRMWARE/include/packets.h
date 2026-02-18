#pragma once

#include <stdint.h>

// Packed layout must match on both endpoints.
struct __attribute__((packed)) ControlPacket {
  uint8_t magic;
  uint16_t seq;
  int16_t throttle;
  int16_t steer;
  uint16_t buttons;
  uint32_t t_ms_sent;
  uint16_t crc16;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t magic;
  uint16_t seq;
  uint16_t battery_mv;
  int16_t temp_c_x10;
  uint16_t flags;
  uint32_t t_ms_sent;
  uint16_t crc16;
};

struct __attribute__((packed)) BigControlPacket {
  uint8_t magic;
  uint16_t seq;
  uint32_t t_ms_sent;
  uint8_t pattern[23];
  uint16_t crc16;
};

struct __attribute__((packed)) BigTelemetryPacket {
  uint8_t magic;
  uint16_t seq;
  uint32_t t_ms_sent;
  uint8_t pattern[23];
  uint16_t crc16;
};

struct __attribute__((packed)) StressPacket {
  uint8_t magic;
  uint8_t stream_id;
  uint32_t seq;
  uint32_t t_ms_sent;
  uint8_t pattern[20];
  uint16_t crc16;
};

static_assert(sizeof(ControlPacket) <= 32, "ControlPacket must be <= 32 bytes.");
static_assert(sizeof(TelemetryPacket) <= 32, "TelemetryPacket must be <= 32 bytes.");
static_assert(sizeof(BigControlPacket) == 32, "BigControlPacket must be 32 bytes.");
static_assert(sizeof(BigTelemetryPacket) == 32, "BigTelemetryPacket must be 32 bytes.");
static_assert(sizeof(StressPacket) <= 32, "StressPacket must be <= 32 bytes.");
