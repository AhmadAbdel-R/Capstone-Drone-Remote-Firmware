#pragma once

#include <stddef.h>
#include <stdint.h>

inline uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

template <typename T>
inline uint16_t calcPacketCrc(const T& pkt) {
  return crc16Ccitt(reinterpret_cast<const uint8_t*>(&pkt), sizeof(T) - sizeof(pkt.crc16));
}

template <typename T>
inline void finalizePacketCrc(T& pkt) {
  pkt.crc16 = calcPacketCrc(pkt);
}

template <typename T>
inline bool isPacketCrcOk(const T& pkt) {
  return calcPacketCrc(pkt) == pkt.crc16;
}
