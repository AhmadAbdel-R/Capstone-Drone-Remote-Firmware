#pragma once

#include <stdint.h>
#include <RF24.h>

struct RadioLinkUiSnapshot {
  bool is_remote_role = false;
  bool stress_mode_enabled = false;
  bool big_packet_mode_enabled = false;

  bool ctrl_link_ok = false;
  bool telm_link_ok = false;

  bool ctrl_valid = false;
  bool telm_valid = false;

  uint32_t ctrl_seq = 0;
  uint32_t telm_seq = 0;

  uint32_t ctrl_age_ms = 0;
  uint32_t telm_age_ms = 0;
};

struct RadioLinkStatsSnapshot {
  uint32_t tx_count = 0;
  uint32_t tx_fail = 0;
  uint32_t tx_bytes = 0;

  uint32_t rx_ok = 0;
  uint32_t rx_bad_crc = 0;
  uint32_t rx_bad_magic = 0;
  uint32_t payload_corrupt = 0;
  uint32_t rx_bytes = 0;

  uint32_t last_seq = 0;
  uint32_t highest_seq = 0;
  uint32_t gaps = 0;
  uint32_t duplicates = 0;
  uint32_t seq_resets = 0;
};

struct RadioRuntimeSettings {
  uint8_t ctrl_channel = 0;
  rf24_datarate_e ctrl_data_rate = RF24_250KBPS;
  rf24_crclength_e ctrl_crc_len = RF24_CRC_16;
  uint8_t ctrl_pa_level = RF24_PA_MAX;
  bool ctrl_auto_ack = false;
  uint8_t ctrl_retry_delay = 0;
  uint8_t ctrl_retry_count = 0;

  uint8_t telm_channel = 0;
  rf24_datarate_e telm_data_rate = RF24_250KBPS;
  rf24_crclength_e telm_crc_len = RF24_CRC_16;
  uint8_t telm_pa_level = RF24_PA_MAX;
  bool telm_auto_ack = true;
  uint8_t telm_retry_delay = 0;
  uint8_t telm_retry_count = 0;
};

void radioLinkInit();
void radioLinkTick();

bool radioLinkIsRemoteRole();
bool radioLinkStressModeEnabled();
bool radioLinkBigPacketModeEnabled();

void radioLinkGetUiSnapshot(RadioLinkUiSnapshot* out);
void radioLinkGetStatsSnapshot(RadioLinkStatsSnapshot* ctrl_out, RadioLinkStatsSnapshot* telm_out);
void radioLinkGetChipStatus(bool* ctrl_connected, bool* telm_connected);

void radioLinkGetRuntimeSettings(RadioRuntimeSettings* out);
bool radioLinkApplyRuntimeSettings(const RadioRuntimeSettings* in, bool* requires_restart);
