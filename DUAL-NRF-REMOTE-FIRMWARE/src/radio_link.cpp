
#include "radio_link.h"

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <stdio.h>

#include "config.h"
#include "crc.h"
#include "log.h"
#include "packets.h"
#include "pins.h"

SPIClass hspi(HSPI);

RF24 radioCtrl(PIN_CTRL_CE, PIN_CTRL_CSN);
RF24 radioTelm(PIN_TELM_CE, PIN_TELM_CSN);

// Remote CTRL TX -> ADDR_CTRL, FCU CTRL RX <- ADDR_CTRL.
// FCU TELM TX -> ADDR_TELM, Remote TELM RX <- ADDR_TELM.
constexpr uint8_t ADDR_CTRL[5] = {'C', 'T', 'R', 'L', '1'};
constexpr uint8_t ADDR_TELM[5] = {'T', 'E', 'L', 'M', '1'};

static_assert(sizeof(ADDR_CTRL) == 5, "ADDR_CTRL must be 5 bytes.");
static_assert(sizeof(ADDR_TELM) == 5, "ADDR_TELM must be 5 bytes.");

constexpr uint8_t MAGIC_CTRL = 0xA1;
constexpr uint8_t MAGIC_TELM = 0xB2;
constexpr uint8_t STREAM_ID_CTRL = 1;
constexpr uint8_t STREAM_ID_TELM = 2;

// Packet structs are declared in include/packets.h.

constexpr uint8_t CTRL_PAYLOAD_SIZE =
#if STRESS_TEST_ENABLE
    static_cast<uint8_t>(sizeof(StressPacket));
#elif TEST_MODE_BIG_PACKET
    static_cast<uint8_t>(sizeof(BigControlPacket));
#else
    static_cast<uint8_t>(sizeof(ControlPacket));
#endif

constexpr uint8_t TELM_PAYLOAD_SIZE =
#if STRESS_TEST_ENABLE
    static_cast<uint8_t>(sizeof(StressPacket));
#elif TEST_MODE_BIG_PACKET
    static_cast<uint8_t>(sizeof(BigTelemetryPacket));
#else
    static_cast<uint8_t>(sizeof(TelemetryPacket));
#endif

struct RadioConfig {
  uint8_t channel;
  rf24_datarate_e data_rate;
  rf24_crclength_e crc_length;
  uint8_t pa_level;
  uint8_t address_width;
  bool dynamic_payloads;
  bool auto_ack;
  uint8_t retry_delay;
  uint8_t retry_count;
  uint8_t payload_size;
};

constexpr bool CTRL_CFG_DYNAMIC = (CTRL_DYNAMIC_PAYLOADS != 0);
constexpr bool TELM_CFG_DYNAMIC = (TELM_DYNAMIC_PAYLOADS != 0);
constexpr bool CTRL_CFG_AUTO_ACK = (CTRL_AUTO_ACK != 0);
constexpr bool TELM_CFG_AUTO_ACK = (TELM_AUTO_ACK != 0);

constexpr RadioConfig CTRL_CFG = {
    RF_CHANNEL_CTRL,
    RF_DATA_RATE_CTRL,
    RF_CRC_LEN_CTRL,
    RF_PA_LEVEL_CTRL,
    RF_ADDR_WIDTH_CTRL,
    CTRL_CFG_DYNAMIC,
    CTRL_CFG_AUTO_ACK,
    CTRL_RETRY_DELAY,
    CTRL_RETRY_COUNT,
    CTRL_PAYLOAD_SIZE};

constexpr RadioConfig TELM_CFG = {
    RF_CHANNEL_TELM,
    RF_DATA_RATE_TELM,
    RF_CRC_LEN_TELM,
    RF_PA_LEVEL_TELM,
    RF_ADDR_WIDTH_TELM,
    TELM_CFG_DYNAMIC,
    TELM_CFG_AUTO_ACK,
    TELM_RETRY_DELAY,
    TELM_RETRY_COUNT,
    TELM_PAYLOAD_SIZE};

RadioRuntimeSettings gRuntimeSettings = {
    CTRL_CFG.channel,
    CTRL_CFG.data_rate,
    CTRL_CFG.crc_length,
    CTRL_CFG.pa_level,
    CTRL_CFG.auto_ack,
    CTRL_CFG.retry_delay,
    CTRL_CFG.retry_count,
    TELM_CFG.channel,
    TELM_CFG.data_rate,
    TELM_CFG.crc_length,
    TELM_CFG.pa_level,
    TELM_CFG.auto_ack,
    TELM_CFG.retry_delay,
    TELM_CFG.retry_count};

// ---------------- runtime state ----------------

struct StreamStats {
  // TX counters.
  uint32_t tx_count = 0;
  uint32_t tx_fail = 0;
  uint32_t tx_bytes = 0;

  // RX counters.
  uint32_t rx_ok = 0;
  uint32_t rx_bad_crc = 0;
  uint32_t rx_bad_magic = 0;
  uint32_t payload_corrupt = 0;
  uint32_t rx_bytes = 0;

  // Sequence tracking.
  bool have_last_seq = false;
  uint32_t last_seq = 0;
  uint32_t highest_seq = 0;
  uint32_t gaps = 0;
  uint32_t duplicates = 0;
  uint32_t seq_resets = 0;
  bool near_end_seen = false;

  // RX timing.
  bool rx_started = false;
  uint32_t rx_start_ms = 0;
  uint32_t rx_end_ms = 0;
  uint32_t last_rx_ms = 0;
  uint32_t last_valid_rx_ms = 0;
  uint32_t iat_count = 0;
  uint32_t iat_sum = 0;
  uint32_t iat_min = UINT32_MAX;
  uint32_t iat_max = 0;

  // TX timing.
  bool tx_started = false;
  uint32_t tx_start_ms = 0;
  uint32_t tx_end_ms = 0;
  bool tx_finished = false;
};

struct CtrlSnapshot {
  bool valid = false;
  uint32_t seq = 0;
  int16_t throttle = 0;
  int16_t steer = 0;
  uint16_t buttons = 0;
  uint32_t rx_local_ms = 0;
};

struct TelmSnapshot {
  bool valid = false;
  uint32_t seq = 0;
  uint16_t battery_mv = 0;
  int16_t temp_c_x10 = 0;
  uint16_t flags = 0;
  uint32_t rx_local_ms = 0;
};

struct PublicRadioState {
  bool chip_connected = false;
  uint8_t channel = 0;
  rf24_datarate_e data_rate = RF24_1MBPS;
  rf24_crclength_e crc_len = RF24_CRC_DISABLED;
  uint8_t pa_level = RF24_PA_MIN;
  uint8_t payload_size = 0;
};

StreamStats ctrlStats;
StreamStats telmStats;
CtrlSnapshot lastCtrlPkt;
TelmSnapshot lastTelmPkt;

// Manual listening-state tracking (RF24 v1.5.0-safe).
bool gCtrlListening = false;
bool gTelmListening = false;

volatile bool gCtrlIrqPending = false;
volatile bool gTelmIrqPending = false;

// Log throttles.
uint32_t lastCtrlValidLogMs = 0;
uint32_t lastTelmValidLogMs = 0;
uint32_t lastCtrlBadLogMs = 0;
uint32_t lastTelmBadLogMs = 0;
uint32_t lastSummaryMs = 0;
uint32_t lastSummaryCalcMs = 0;
uint32_t lastCtrlTxCountSummary = 0;
uint32_t lastTelmTxCountSummary = 0;

// Normal mode runtime.
uint16_t ctrlTxSeqNormal = 0;
uint16_t telmTxSeqNormal = 0;
uint32_t lastCtrlNormalSendMs = 0;
uint32_t lastTelmNormalSendMs = 0;
bool ctrlLinkStateKnown = false;
bool telmLinkStateKnown = false;
bool lastCtrlLinkOk = false;
bool lastTelmLinkOk = false;

// Stress mode runtime.
uint32_t ctrlTxSeq = 0;
uint32_t telmTxSeq = 0;
uint32_t lastCtrlStressSendMs = 0;
uint32_t lastTelmStressSendMs = 0;
uint32_t lastStressProgressMs = 0;
uint32_t lastStressFinalReportMs = 0;
bool gStressCtrlRxDone = false;
bool gStressTelmRxDone = false;
bool gStressFinalMode = false;

enum DeviceRole : uint8_t { ROLE_FCU = 0, ROLE_REMOTE = 1 };
constexpr DeviceRole THIS_ROLE = (DEVICE_ROLE_REMOTE == 1) ? ROLE_REMOTE : ROLE_FCU;

// ---------------- utility helpers ----------------

const char* roleName(DeviceRole r) {
  return (r == ROLE_REMOTE) ? "REMOTE" : "FCU";
}

const char* boolText(bool v) {
  return v ? "YES" : "NO";
}

const char* dataRateText(rf24_datarate_e r) {
  switch (r) {
    case RF24_250KBPS: return "250kbps";
    case RF24_1MBPS: return "1Mbps";
    case RF24_2MBPS: return "2Mbps";
    default: return "unknown";
  }
}

const char* crcText(rf24_crclength_e c) {
  switch (c) {
    case RF24_CRC_DISABLED: return "OFF";
    case RF24_CRC_8: return "CRC8";
    case RF24_CRC_16: return "CRC16";
    default: return "unknown";
  }
}

const char* paText(uint8_t p) {
  switch (p) {
    case RF24_PA_MIN: return "MIN";
    case RF24_PA_LOW: return "LOW";
    case RF24_PA_HIGH: return "HIGH";
    case RF24_PA_MAX: return "MAX";
    default: return "unknown";
  }
}

uint32_t packetAgeMs(uint32_t sentMs) {
  const uint32_t age = static_cast<uint32_t>(millis() - sentMs);
  if (age > LATENCY_SANITY_MAX_MS) {
    return 0;
  }
  return age;
}

uint32_t rateFromWindow(uint32_t count, uint32_t startMs, uint32_t endMs) {
  if (count == 0) {
    return 0;
  }
  const uint32_t dt = static_cast<uint32_t>(endMs - startMs);
  if (dt == 0) {
    return 0;
  }
  return static_cast<uint32_t>((static_cast<uint64_t>(count) * 1000ULL) / dt);
}

uint32_t rateFromDelta(uint32_t countNow, uint32_t countPrev, uint32_t dtMs) {
  if (dtMs == 0) {
    return 0;
  }
  const uint32_t delta = static_cast<uint32_t>(countNow - countPrev);
  return static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000ULL) / dtMs);
}

uint16_t triangleWaveU16(uint32_t phase, uint32_t period, uint16_t maxValue) {
  if (period < 2U) {
    return 0;
  }
  const uint32_t half = period / 2U;
  if (half == 0U) {
    return 0;
  }

  const uint32_t p = phase % period;
  if (p <= half) {
    return static_cast<uint16_t>((static_cast<uint64_t>(p) * maxValue) / half);
  }
  const uint32_t down = static_cast<uint32_t>(period - p);
  return static_cast<uint16_t>((static_cast<uint64_t>(down) * maxValue) / half);
}

int16_t triangleWaveS16(uint32_t phase, uint32_t period, int16_t amplitude) {
  const uint16_t up = triangleWaveU16(phase, period, static_cast<uint16_t>(amplitude * 2));
  return static_cast<int16_t>(static_cast<int32_t>(up) - static_cast<int32_t>(amplitude));
}

bool ctrlLinkOkNow(uint32_t nowMs) {
  if (ctrlStats.rx_ok == 0) {
    return false;
  }
  const uint32_t age = static_cast<uint32_t>(nowMs - ctrlStats.last_valid_rx_ms);
  return age < CTRL_LINK_TIMEOUT_MS;
}

bool telmLinkOkNow(uint32_t nowMs) {
  if (telmStats.rx_ok == 0) {
    return false;
  }
  const uint32_t age = static_cast<uint32_t>(nowMs - telmStats.last_valid_rx_ms);
  return age < TELM_LINK_TIMEOUT_MS;
}

uint32_t stressNearEndStart() {
  if (STRESS_TEST_TARGET_COUNT > STRESS_NEAR_END_MARGIN) {
    return STRESS_TEST_TARGET_COUNT - STRESS_NEAR_END_MARGIN;
  }
  return 0;
}

void updateSeqTracking(StreamStats& s, uint32_t seq) {
  if (!s.have_last_seq) {
    s.have_last_seq = true;
    s.last_seq = seq;
    s.highest_seq = seq;
    return;
  }

  const uint32_t delta = static_cast<uint32_t>(seq - s.last_seq);
  if (delta == 0) {
    s.duplicates++;
  } else if (delta == 1) {
    // In order.
  } else if (delta < SEQ_RESET_THRESHOLD) {
    s.gaps += (delta - 1U);
  } else {
    // Huge jump is treated as sender reset/restart, not a 65k loss burst.
    s.seq_resets++;
  }

  s.last_seq = seq;
  if (seq > s.highest_seq) {
    s.highest_seq = seq;
  }
}

void recordTx(StreamStats& s, uint8_t bytes, bool writeOk, bool autoAckEnabled, uint32_t nowMs) {
  if (!s.tx_started) {
    s.tx_started = true;
    s.tx_start_ms = nowMs;
  }
  s.tx_end_ms = nowMs;
  s.tx_count++;
  s.tx_bytes += bytes;
  if (autoAckEnabled && !writeOk) {
    s.tx_fail++;
  }
}

void recordRxValid(StreamStats& s, uint8_t bytes, uint32_t seq, uint32_t nowMs) {
  if (!s.rx_started) {
    s.rx_started = true;
    s.rx_start_ms = nowMs;
  }
  s.rx_end_ms = nowMs;
  s.last_rx_ms = nowMs;
  s.rx_ok++;
  s.rx_bytes += bytes;

  if (s.last_valid_rx_ms != 0) {
    const uint32_t iat = static_cast<uint32_t>(nowMs - s.last_valid_rx_ms);
    s.iat_count++;
    s.iat_sum += iat;
    if (iat < s.iat_min) {
      s.iat_min = iat;
    }
    if (iat > s.iat_max) {
      s.iat_max = iat;
    }
  }
  s.last_valid_rx_ms = nowMs;

  updateSeqTracking(s, seq);
#if STRESS_TEST_ENABLE
  if (seq >= stressNearEndStart()) {
    s.near_end_seen = true;
  }
#endif
}

PublicRadioState readPublicRadioState(RF24& radio) {
  PublicRadioState s;
  s.chip_connected = radio.isChipConnected();
  s.channel = radio.getChannel();
  s.data_rate = radio.getDataRate();
  s.crc_len = radio.getCRCLength();
  s.pa_level = radio.getPALevel();
  s.payload_size = radio.getPayloadSize();
  return s;
}

bool runtimeMatchesExpected(const PublicRadioState& rt, const RadioConfig& exp) {
  // Compare only fields exposed by RF24 public getters.
  return rt.chip_connected &&
         (rt.channel == exp.channel) &&
         (rt.data_rate == exp.data_rate) &&
         (rt.crc_len == exp.crc_length) &&
         (rt.pa_level == exp.pa_level) &&
         (rt.payload_size == exp.payload_size);
}

void printExpectedConfig(const char* name, const RadioConfig& cfg) {
  LOG_INF("%s expected: ch=%u rate=%s crc=%s pa=%s aw=%u payload=%u dyn=%s ack=%s retries=%u/%u",
          name,
          cfg.channel,
          dataRateText(cfg.data_rate),
          crcText(cfg.crc_length),
          paText(cfg.pa_level),
          cfg.address_width,
          cfg.payload_size,
          cfg.dynamic_payloads ? "ON" : "OFF",
          cfg.auto_ack ? "ON" : "OFF",
          cfg.retry_delay,
          cfg.retry_count);
}

void printRuntimeSummary(const char* name, RF24& radio, const RadioConfig& cfg, bool listeningFlag) {
  const PublicRadioState rt = readPublicRadioState(radio);
  LOG_INF("%s runtime: chip=%s listen=%s ch=%u rate=%s crc=%s pa=%s payload=%u",
          name,
          boolText(rt.chip_connected),
          boolText(listeningFlag),
          rt.channel,
          dataRateText(rt.data_rate),
          crcText(rt.crc_len),
          paText(rt.pa_level),
          rt.payload_size);

  if (runtimeMatchesExpected(rt, cfg)) {
    LOG_OK("%s getter-check: PASS", name);
  } else {
    LOG_ERR("%s getter-check: FAIL", name);
  }
}
// CRC helpers are declared in include/crc.h.

void fillBigPattern(uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    data[i] = static_cast<uint8_t>(i & 0xFFU);
  }
}

bool checkBigPattern(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] != static_cast<uint8_t>(i & 0xFFU)) {
      return false;
    }
  }
  return true;
}

void fillStressPattern(uint8_t* data, size_t len, uint32_t seq, uint8_t streamId) {
  for (size_t i = 0; i < len; ++i) {
    data[i] = static_cast<uint8_t>((seq + static_cast<uint32_t>(streamId) * 17U + static_cast<uint32_t>(i)) & 0xFFU);
  }
}

bool checkStressPattern(const uint8_t* data, size_t len, uint32_t seq, uint8_t streamId) {
  for (size_t i = 0; i < len; ++i) {
    const uint8_t expected = static_cast<uint8_t>((seq + static_cast<uint32_t>(streamId) * 17U + static_cast<uint32_t>(i)) & 0xFFU);
    if (data[i] != expected) {
      return false;
    }
  }
  return true;
}

// ---------------- radio config ----------------

void prepRadioPinsAfterBootDelay() {
  delay(RADIO_BOOT_DELAY_MS);

  // Strap-sensitive radio pins: keep both radios disabled before begin().
  pinMode(PIN_CTRL_CE, OUTPUT);
  pinMode(PIN_TELM_CE, OUTPUT);
  digitalWrite(PIN_CTRL_CE, LOW);
  digitalWrite(PIN_TELM_CE, LOW);

  pinMode(PIN_CTRL_CSN, OUTPUT);
  pinMode(PIN_TELM_CSN, OUTPUT);
  digitalWrite(PIN_CTRL_CSN, HIGH);
  digitalWrite(PIN_TELM_CSN, HIGH);

  pinMode(PIN_CTRL_IRQ, INPUT_PULLUP);
  pinMode(PIN_TELM_IRQ, INPUT_PULLUP);
}

bool applyRadioConfig(RF24& radio, const RadioConfig& cfg) {
#if defined(RF24_SPI_PTR)
  if (!radio.begin(&hspi)) {
#else
  if (!radio.begin()) {
#endif
    return false;
  }

  radio.setAddressWidth(cfg.address_width);
  radio.setChannel(cfg.channel);
  radio.setDataRate(cfg.data_rate);
  radio.setCRCLength(cfg.crc_length);
  radio.setPALevel(static_cast<rf24_pa_dbm_e>(cfg.pa_level));
  radio.setAutoAck(cfg.auto_ack);
  radio.setRetries(cfg.retry_delay, cfg.retry_count);

  if (cfg.dynamic_payloads) {
    radio.enableDynamicPayloads();
  } else {
    radio.disableDynamicPayloads();
  }

  radio.setPayloadSize(cfg.payload_size);
  return true;
}

void configureCtrlRadio(DeviceRole role) {
  if (!applyRadioConfig(radioCtrl, CTRL_CFG)) {
    LOG_ERR("CTRL radio init/config failed");
    while (true) {
      delay(1000);
    }
  }

  if (role == ROLE_FCU) {
    // FCU: CTRL listens for commands.
    radioCtrl.openReadingPipe(1, ADDR_CTRL);
    radioCtrl.startListening();
    gCtrlListening = true;
  } else {
    // Remote: CTRL transmits commands.
    radioCtrl.stopListening();
    radioCtrl.openWritingPipe(ADDR_CTRL);
    gCtrlListening = false;
  }
}

void configureTelmRadio(DeviceRole role) {
  if (!applyRadioConfig(radioTelm, TELM_CFG)) {
    LOG_ERR("TELM radio init/config failed");
    while (true) {
      delay(1000);
    }
  }

  if (role == ROLE_REMOTE) {
    // Remote: TELM listens for telemetry.
    radioTelm.openReadingPipe(1, ADDR_TELM);
    radioTelm.startListening();
    gTelmListening = true;
  } else {
    // FCU: TELM transmits telemetry.
    radioTelm.stopListening();
    radioTelm.openWritingPipe(ADDR_TELM);
    gTelmListening = false;
  }
}

void ensureCtrlRadioRxMode() {
  if (!gCtrlListening) {
    radioCtrl.startListening();
    gCtrlListening = true;
  }
}

void ensureCtrlRadioTxMode() {
  if (gCtrlListening) {
    radioCtrl.stopListening();
    gCtrlListening = false;
  }
}

void ensureTelmRadioRxMode() {
  if (!gTelmListening) {
    radioTelm.startListening();
    gTelmListening = true;
  }
}

void ensureTelmRadioTxMode() {
  if (gTelmListening) {
    radioTelm.stopListening();
    gTelmListening = false;
  }
}

void IRAM_ATTR onCtrlIrq() {
  gCtrlIrqPending = true;
}

void IRAM_ATTR onTelmIrq() {
  gTelmIrqPending = true;
}

// ---------------- RX log helpers ----------------

void maybeLogCtrlInvalid(const char* reason, bool severe = false) {
  const uint32_t now = millis();
  if ((now - lastCtrlBadLogMs) < BAD_RX_PRINT_MIN_INTERVAL_MS) {
    return;
  }
  lastCtrlBadLogMs = now;
  if (severe) {
    LOG_ERR("[CTRL RX] %s", reason);
  } else {
    LOG_WARN("[CTRL RX] %s", reason);
  }
}

void maybeLogTelmInvalid(const char* reason, bool severe = false) {
  const uint32_t now = millis();
  if ((now - lastTelmBadLogMs) < BAD_RX_PRINT_MIN_INTERVAL_MS) {
    return;
  }
  lastTelmBadLogMs = now;
  if (severe) {
    LOG_ERR("[TELM RX] %s", reason);
  } else {
    LOG_WARN("[TELM RX] %s", reason);
  }
}

// ---------------- normal mode TX / link events ----------------

void fillNormalCtrlPacket(ControlPacket& pkt, uint16_t seq, uint32_t nowMs) {
  pkt.magic = MAGIC_CTRL;
  pkt.seq = seq;
  pkt.throttle = static_cast<int16_t>(triangleWaveU16(nowMs, 4000UL, 1023U));
  pkt.steer = triangleWaveS16(static_cast<uint32_t>(nowMs + 750UL), 3000UL, 512);

  uint16_t buttons = 0;
  if (((nowMs / 500UL) & 0x1U) != 0U) {
    buttons |= 0x0001U;
  }
  if (((nowMs / 2000UL) & 0x1U) != 0U) {
    buttons |= 0x0002U;
  }
  pkt.buttons = buttons;
  pkt.t_ms_sent = nowMs;
  finalizePacketCrc(pkt);
}

void fillNormalTelmPacket(TelemetryPacket& pkt, uint16_t seq, uint32_t nowMs) {
  pkt.magic = MAGIC_TELM;
  pkt.seq = seq;
  pkt.battery_mv = static_cast<uint16_t>(12000 + triangleWaveS16(nowMs, 20000UL, 100));
  pkt.temp_c_x10 = static_cast<int16_t>(250 + triangleWaveS16(static_cast<uint32_t>(nowMs + 3200UL), 30000UL, 20));

  const bool ctrlLinkOk = ctrlLinkOkNow(nowMs);
  uint16_t flags = 0;
  if (ctrlLinkOk) {
    flags |= 0x0001U;
  } else {
    flags |= 0x0002U;
  }
  pkt.flags = flags;
  pkt.t_ms_sent = nowMs;
  finalizePacketCrc(pkt);
}

void fillNormalBigCtrlPacket(BigControlPacket& pkt, uint16_t seq, uint32_t nowMs) {
  pkt.magic = MAGIC_CTRL;
  pkt.seq = seq;
  pkt.t_ms_sent = nowMs;
  fillBigPattern(pkt.pattern, sizeof(pkt.pattern));
  finalizePacketCrc(pkt);
}

void fillNormalBigTelmPacket(BigTelemetryPacket& pkt, uint16_t seq, uint32_t nowMs) {
  pkt.magic = MAGIC_TELM;
  pkt.seq = seq;
  pkt.t_ms_sent = nowMs;
  fillBigPattern(pkt.pattern, sizeof(pkt.pattern));
  finalizePacketCrc(pkt);
}

void sendNormalCtrlIfDue() {
  if (THIS_ROLE != ROLE_REMOTE) {
    return;
  }

  const uint32_t now = millis();
  if ((now - lastCtrlNormalSendMs) < CTRL_TX_PERIOD_MS) {
    return;
  }
  lastCtrlNormalSendMs = now;

  ensureCtrlRadioTxMode();

#if TEST_MODE_BIG_PACKET
  BigControlPacket pkt = {};
  fillNormalBigCtrlPacket(pkt, ctrlTxSeqNormal, now);
  const bool ok = radioCtrl.write(&pkt, sizeof(pkt));
  recordTx(ctrlStats, sizeof(pkt), ok, gRuntimeSettings.ctrl_auto_ack, now);
#else
  ControlPacket pkt = {};
  fillNormalCtrlPacket(pkt, ctrlTxSeqNormal, now);
  const bool ok = radioCtrl.write(&pkt, sizeof(pkt));
  recordTx(ctrlStats, sizeof(pkt), ok, gRuntimeSettings.ctrl_auto_ack, now);
#endif
  ctrlTxSeqNormal++;
}

void sendNormalTelmIfDue() {
  if (THIS_ROLE != ROLE_FCU) {
    return;
  }

  const uint32_t now = millis();
  if ((now - lastTelmNormalSendMs) < TELM_TX_PERIOD_MS) {
    return;
  }
  lastTelmNormalSendMs = now;

  ensureTelmRadioTxMode();

#if TEST_MODE_BIG_PACKET
  BigTelemetryPacket pkt = {};
  fillNormalBigTelmPacket(pkt, telmTxSeqNormal, now);
  const bool ok = radioTelm.write(&pkt, sizeof(pkt));
  recordTx(telmStats, sizeof(pkt), ok, gRuntimeSettings.telm_auto_ack, now);
#else
  TelemetryPacket pkt = {};
  fillNormalTelmPacket(pkt, telmTxSeqNormal, now);
  const bool ok = radioTelm.write(&pkt, sizeof(pkt));
  recordTx(telmStats, sizeof(pkt), ok, gRuntimeSettings.telm_auto_ack, now);
#endif
  telmTxSeqNormal++;
}

void updateNormalLinkEvents() {
  const uint32_t now = millis();
  if (THIS_ROLE == ROLE_FCU) {
    const bool linkOk = ctrlLinkOkNow(now);
    if (!ctrlLinkStateKnown || (linkOk != lastCtrlLinkOk)) {
      ctrlLinkStateKnown = true;
      lastCtrlLinkOk = linkOk;
      Serial.printf("[LINK] CTRL -> %s\n", linkOk ? "OK" : "LOST");
    }
  } else {
    const bool linkOk = telmLinkOkNow(now);
    if (!telmLinkStateKnown || (linkOk != lastTelmLinkOk)) {
      telmLinkStateKnown = true;
      lastTelmLinkOk = linkOk;
      Serial.printf("[LINK] TELM -> %s\n", linkOk ? "OK" : "LOST");
    }
  }
}

// ---------------- stress TX ----------------

#if STRESS_TEST_ENABLE
void sendStressCtrlIfDue() {
  if (THIS_ROLE != ROLE_REMOTE) {
    return;
  }
  if (ctrlStats.tx_finished) {
    return;
  }

  const uint32_t now = millis();
  if (STRESS_CTRL_SEND_INTERVAL_MS > 0UL) {
    if ((now - lastCtrlStressSendMs) < STRESS_CTRL_SEND_INTERVAL_MS) {
      return;
    }
  }
  lastCtrlStressSendMs = now;

  StressPacket pkt = {};
  pkt.magic = MAGIC_CTRL;
  pkt.stream_id = STREAM_ID_CTRL;
  pkt.seq = ctrlTxSeq;
  pkt.t_ms_sent = now;
  fillStressPattern(pkt.pattern, sizeof(pkt.pattern), pkt.seq, pkt.stream_id);
  finalizePacketCrc(pkt);

  const bool ok = radioCtrl.write(&pkt, sizeof(pkt));
  recordTx(ctrlStats, sizeof(pkt), ok, gRuntimeSettings.ctrl_auto_ack, now);

  ctrlTxSeq++;
  if (ctrlTxSeq >= STRESS_TEST_TARGET_COUNT) {
    ctrlStats.tx_finished = true;
    ctrlStats.tx_end_ms = now;
    LOG_INF("CTRL stress TX finished at seq=%lu", static_cast<unsigned long>(ctrlTxSeq - 1U));
  }
}

void sendStressTelmIfDue() {
  if (THIS_ROLE != ROLE_FCU) {
    return;
  }
  if (telmStats.tx_finished) {
    return;
  }

  const uint32_t now = millis();
  if (STRESS_TELM_SEND_INTERVAL_MS > 0UL) {
    if ((now - lastTelmStressSendMs) < STRESS_TELM_SEND_INTERVAL_MS) {
      return;
    }
  }
  lastTelmStressSendMs = now;

  StressPacket pkt = {};
  pkt.magic = MAGIC_TELM;
  pkt.stream_id = STREAM_ID_TELM;
  pkt.seq = telmTxSeq;
  pkt.t_ms_sent = now;
  fillStressPattern(pkt.pattern, sizeof(pkt.pattern), pkt.seq, pkt.stream_id);
  finalizePacketCrc(pkt);

  const bool ok = radioTelm.write(&pkt, sizeof(pkt));
  recordTx(telmStats, sizeof(pkt), ok, gRuntimeSettings.telm_auto_ack, now);

  telmTxSeq++;
  if (telmTxSeq >= STRESS_TEST_TARGET_COUNT) {
    telmStats.tx_finished = true;
    telmStats.tx_end_ms = now;
    LOG_INF("TELM stress TX finished at seq=%lu", static_cast<unsigned long>(telmTxSeq - 1U));
  }
}
#endif

// ---------------- receive functions ----------------

void recvCtrl() {
  if (THIS_ROLE != ROLE_FCU) {
    return;
  }

  ensureCtrlRadioRxMode();

  while (radioCtrl.available()) {
#if STRESS_TEST_ENABLE
    StressPacket pkt = {};
    radioCtrl.read(&pkt, sizeof(pkt));

    const bool magicOk = (pkt.magic == MAGIC_CTRL);
    const bool streamOk = (pkt.stream_id == STREAM_ID_CTRL);
    const bool crcOk = isPacketCrcOk(pkt);
    const bool patternOk = checkStressPattern(pkt.pattern, sizeof(pkt.pattern), pkt.seq, pkt.stream_id);

    if (!magicOk || !streamOk) {
      ctrlStats.rx_bad_magic++;
      maybeLogCtrlInvalid("BAD_MAGIC");
      continue;
    }
    if (!crcOk) {
      ctrlStats.rx_bad_crc++;
      maybeLogCtrlInvalid("BAD_CRC");
      continue;
    }
    if (!patternOk) {
      ctrlStats.payload_corrupt++;
      maybeLogCtrlInvalid("PAYLOAD_CORRUPT", true);
      continue;
    }

    const uint32_t now = millis();
    recordRxValid(ctrlStats, sizeof(pkt), pkt.seq, now);

    const uint32_t ageMs = packetAgeMs(pkt.t_ms_sent);
    (void)ageMs;
    lastCtrlPkt.valid = true;
    lastCtrlPkt.seq = pkt.seq;
    lastCtrlPkt.throttle = 0;
    lastCtrlPkt.steer = 0;
    lastCtrlPkt.buttons = 0;
    lastCtrlPkt.rx_local_ms = now;
#elif TEST_MODE_BIG_PACKET
    BigControlPacket pkt = {};
    radioCtrl.read(&pkt, sizeof(pkt));

    const bool magicOk = (pkt.magic == MAGIC_CTRL);
    const bool crcOk = isPacketCrcOk(pkt);
    const bool patternOk = checkBigPattern(pkt.pattern, sizeof(pkt.pattern));

    if (!magicOk) {
      ctrlStats.rx_bad_magic++;
      maybeLogCtrlInvalid("BAD_MAGIC");
      continue;
    }
    if (!crcOk) {
      ctrlStats.rx_bad_crc++;
      maybeLogCtrlInvalid("BAD_CRC");
      continue;
    }
    if (!patternOk) {
      ctrlStats.payload_corrupt++;
      maybeLogCtrlInvalid("PAYLOAD_CORRUPT", true);
      continue;
    }

    const uint32_t now = millis();
    recordRxValid(ctrlStats, sizeof(pkt), static_cast<uint32_t>(pkt.seq), now);

    const uint32_t ageMs = packetAgeMs(pkt.t_ms_sent);
    lastCtrlPkt.valid = true;
    lastCtrlPkt.seq = pkt.seq;
    lastCtrlPkt.throttle = 0;
    lastCtrlPkt.steer = 0;
    lastCtrlPkt.buttons = 0;
    lastCtrlPkt.rx_local_ms = now;

#if DEBUG_RX_PRINT_CTRL
    if ((now - lastCtrlValidLogMs) >= CTRL_RX_PRINT_MIN_INTERVAL_MS) {
      lastCtrlValidLogMs = now;
      LOG_OK("[CTRL RX] seq=%u thr=%d steer=%d btn=0x%04X age=%lums",
             pkt.seq,
             0,
             0,
             0u,
             static_cast<unsigned long>(ageMs));
    }
#endif
#else
    ControlPacket pkt = {};
    radioCtrl.read(&pkt, sizeof(pkt));

    const bool magicOk = (pkt.magic == MAGIC_CTRL);
    const bool crcOk = isPacketCrcOk(pkt);

    if (!magicOk) {
      ctrlStats.rx_bad_magic++;
      maybeLogCtrlInvalid("BAD_MAGIC");
      continue;
    }
    if (!crcOk) {
      ctrlStats.rx_bad_crc++;
      maybeLogCtrlInvalid("BAD_CRC");
      continue;
    }

    const uint32_t now = millis();
    recordRxValid(ctrlStats, sizeof(pkt), static_cast<uint32_t>(pkt.seq), now);

    const uint32_t ageMs = packetAgeMs(pkt.t_ms_sent);
    lastCtrlPkt.valid = true;
    lastCtrlPkt.seq = pkt.seq;
    lastCtrlPkt.throttle = pkt.throttle;
    lastCtrlPkt.steer = pkt.steer;
    lastCtrlPkt.buttons = pkt.buttons;
    lastCtrlPkt.rx_local_ms = now;

#if DEBUG_RX_PRINT_CTRL
    if ((now - lastCtrlValidLogMs) >= CTRL_RX_PRINT_MIN_INTERVAL_MS) {
      lastCtrlValidLogMs = now;
      LOG_OK("[CTRL RX] seq=%u thr=%d steer=%d btn=0x%04X age=%lums",
             pkt.seq,
             pkt.throttle,
             pkt.steer,
             pkt.buttons,
             static_cast<unsigned long>(ageMs));
    }
#endif
#endif
  }
}
void recvTelm() {
  if (THIS_ROLE != ROLE_REMOTE) {
    return;
  }

  ensureTelmRadioRxMode();

  while (radioTelm.available()) {
#if STRESS_TEST_ENABLE
    StressPacket pkt = {};
    radioTelm.read(&pkt, sizeof(pkt));

    const bool magicOk = (pkt.magic == MAGIC_TELM);
    const bool streamOk = (pkt.stream_id == STREAM_ID_TELM);
    const bool crcOk = isPacketCrcOk(pkt);
    const bool patternOk = checkStressPattern(pkt.pattern, sizeof(pkt.pattern), pkt.seq, pkt.stream_id);

    if (!magicOk || !streamOk) {
      telmStats.rx_bad_magic++;
      maybeLogTelmInvalid("BAD_MAGIC");
      continue;
    }
    if (!crcOk) {
      telmStats.rx_bad_crc++;
      maybeLogTelmInvalid("BAD_CRC");
      continue;
    }
    if (!patternOk) {
      telmStats.payload_corrupt++;
      maybeLogTelmInvalid("PAYLOAD_CORRUPT", true);
      continue;
    }

    const uint32_t now = millis();
    recordRxValid(telmStats, sizeof(pkt), pkt.seq, now);

    const uint32_t ageMs = packetAgeMs(pkt.t_ms_sent);
    (void)ageMs;
    lastTelmPkt.valid = true;
    lastTelmPkt.seq = pkt.seq;
    lastTelmPkt.battery_mv = 0;
    lastTelmPkt.temp_c_x10 = 0;
    lastTelmPkt.flags = 0;
    lastTelmPkt.rx_local_ms = now;
#elif TEST_MODE_BIG_PACKET
    BigTelemetryPacket pkt = {};
    radioTelm.read(&pkt, sizeof(pkt));

    const bool magicOk = (pkt.magic == MAGIC_TELM);
    const bool crcOk = isPacketCrcOk(pkt);
    const bool patternOk = checkBigPattern(pkt.pattern, sizeof(pkt.pattern));

    if (!magicOk) {
      telmStats.rx_bad_magic++;
      maybeLogTelmInvalid("BAD_MAGIC");
      continue;
    }
    if (!crcOk) {
      telmStats.rx_bad_crc++;
      maybeLogTelmInvalid("BAD_CRC");
      continue;
    }
    if (!patternOk) {
      telmStats.payload_corrupt++;
      maybeLogTelmInvalid("PAYLOAD_CORRUPT", true);
      continue;
    }

    const uint32_t now = millis();
    recordRxValid(telmStats, sizeof(pkt), static_cast<uint32_t>(pkt.seq), now);

    const uint32_t ageMs = packetAgeMs(pkt.t_ms_sent);
    lastTelmPkt.valid = true;
    lastTelmPkt.seq = pkt.seq;
    lastTelmPkt.battery_mv = 0;
    lastTelmPkt.temp_c_x10 = 0;
    lastTelmPkt.flags = 0;
    lastTelmPkt.rx_local_ms = now;

#if DEBUG_RX_PRINT_TELM
    if ((now - lastTelmValidLogMs) >= TELM_RX_PRINT_MIN_INTERVAL_MS) {
      lastTelmValidLogMs = now;
      LOG_OK("[TELM RX] seq=%u batt=%umV temp=%0.1fC flags=0x%04X age=%lums",
             pkt.seq,
             0u,
             0.0f,
             0u,
             static_cast<unsigned long>(ageMs));
    }
#endif
#else
    TelemetryPacket pkt = {};
    radioTelm.read(&pkt, sizeof(pkt));

    const bool magicOk = (pkt.magic == MAGIC_TELM);
    const bool crcOk = isPacketCrcOk(pkt);

    if (!magicOk) {
      telmStats.rx_bad_magic++;
      maybeLogTelmInvalid("BAD_MAGIC");
      continue;
    }
    if (!crcOk) {
      telmStats.rx_bad_crc++;
      maybeLogTelmInvalid("BAD_CRC");
      continue;
    }

    const uint32_t now = millis();
    recordRxValid(telmStats, sizeof(pkt), static_cast<uint32_t>(pkt.seq), now);

    const uint32_t ageMs = packetAgeMs(pkt.t_ms_sent);
    lastTelmPkt.valid = true;
    lastTelmPkt.seq = pkt.seq;
    lastTelmPkt.battery_mv = pkt.battery_mv;
    lastTelmPkt.temp_c_x10 = pkt.temp_c_x10;
    lastTelmPkt.flags = pkt.flags;
    lastTelmPkt.rx_local_ms = now;

#if DEBUG_RX_PRINT_TELM
    if ((now - lastTelmValidLogMs) >= TELM_RX_PRINT_MIN_INTERVAL_MS) {
      lastTelmValidLogMs = now;
      LOG_OK("[TELM RX] seq=%u batt=%umV temp=%0.1fC flags=0x%04X age=%lums",
             pkt.seq,
             pkt.battery_mv,
             static_cast<float>(pkt.temp_c_x10) / 10.0f,
             pkt.flags,
             static_cast<unsigned long>(ageMs));
    }
#endif
#endif
  }
}

void drainCtrlRxIfPending() {
  if (THIS_ROLE != ROLE_FCU) {
    return;
  }
  if (!gCtrlIrqPending && !radioCtrl.available()) {
    return;
  }
  gCtrlIrqPending = false;
  recvCtrl();
}

void drainTelmRxIfPending() {
  if (THIS_ROLE != ROLE_REMOTE) {
    return;
  }
  if (!gTelmIrqPending && !radioTelm.available()) {
    return;
  }
  gTelmIrqPending = false;
  recvTelm();
}

// ---------------- normal-mode compact summary ----------------

void printCompactSummaryOncePerSecond() {
  const uint32_t now = millis();
  if ((now - lastSummaryMs) < SUMMARY_PERIOD_MS) {
    return;
  }

  const uint32_t dtMs = (lastSummaryCalcMs == 0)
                            ? SUMMARY_PERIOD_MS
                            : static_cast<uint32_t>(now - lastSummaryCalcMs);
  lastSummaryCalcMs = now;
  lastSummaryMs = now;

  const uint32_t ctrlTxPps = rateFromDelta(ctrlStats.tx_count, lastCtrlTxCountSummary, dtMs);
  const uint32_t telmTxPps = rateFromDelta(telmStats.tx_count, lastTelmTxCountSummary, dtMs);
  lastCtrlTxCountSummary = ctrlStats.tx_count;
  lastTelmTxCountSummary = telmStats.tx_count;
  const bool ctrlChipOk = radioCtrl.isChipConnected();
  const bool telmChipOk = radioTelm.isChipConnected();

  if (THIS_ROLE == ROLE_REMOTE) {
    const bool telmLinkOk = telmLinkOkNow(now);
    if (lastTelmPkt.valid) {
      const uint32_t age = static_cast<uint32_t>(now - lastTelmPkt.rx_local_ms);
      Serial.printf("ROLE=REMOTE CTRL_CHIP=%s TELM_CHIP=%s CTRL_TX=%lupps TELM_LINK=%s lastTELMseq=%lu age_ms=%lu bad_crc=%lu bad_magic=%lu\n",
                    boolText(ctrlChipOk),
                    boolText(telmChipOk),
                    static_cast<unsigned long>(ctrlTxPps),
                    telmLinkOk ? "OK" : "LOST",
                    static_cast<unsigned long>(lastTelmPkt.seq),
                    static_cast<unsigned long>(age),
                    static_cast<unsigned long>(telmStats.rx_bad_crc),
                    static_cast<unsigned long>(telmStats.rx_bad_magic));
    } else {
      Serial.printf("ROLE=REMOTE CTRL_CHIP=%s TELM_CHIP=%s CTRL_TX=%lupps TELM_LINK=%s lastTELMseq=n/a age_ms=n/a bad_crc=%lu bad_magic=%lu\n",
                    boolText(ctrlChipOk),
                    boolText(telmChipOk),
                    static_cast<unsigned long>(ctrlTxPps),
                    telmLinkOk ? "OK" : "LOST",
                    static_cast<unsigned long>(telmStats.rx_bad_crc),
                    static_cast<unsigned long>(telmStats.rx_bad_magic));
    }
    return;
  }

  const bool ctrlLinkOk = ctrlLinkOkNow(now);
  if (lastCtrlPkt.valid) {
    const uint32_t age = static_cast<uint32_t>(now - lastCtrlPkt.rx_local_ms);
    Serial.printf("ROLE=FCU CTRL_CHIP=%s TELM_CHIP=%s CTRL_LINK=%s lastCTRLseq=%lu age_ms=%lu TELM_TX=%lupps bad_crc=%lu bad_magic=%lu\n",
                  boolText(ctrlChipOk),
                  boolText(telmChipOk),
                  ctrlLinkOk ? "OK" : "LOST",
                  static_cast<unsigned long>(lastCtrlPkt.seq),
                  static_cast<unsigned long>(age),
                  static_cast<unsigned long>(telmTxPps),
                  static_cast<unsigned long>(ctrlStats.rx_bad_crc),
                  static_cast<unsigned long>(ctrlStats.rx_bad_magic));
  } else {
    Serial.printf("ROLE=FCU CTRL_CHIP=%s TELM_CHIP=%s CTRL_LINK=%s lastCTRLseq=n/a age_ms=n/a TELM_TX=%lupps bad_crc=%lu bad_magic=%lu\n",
                  boolText(ctrlChipOk),
                  boolText(telmChipOk),
                  ctrlLinkOk ? "OK" : "LOST",
                  static_cast<unsigned long>(telmTxPps),
                  static_cast<unsigned long>(ctrlStats.rx_bad_crc),
                  static_cast<unsigned long>(ctrlStats.rx_bad_magic));
  }
}

// ---------------- stress progress/final reports ----------------

#if STRESS_TEST_ENABLE
bool isCtrlRxFinished(uint32_t nowMs) {
  if (!ctrlStats.rx_started) {
    return false;
  }
  if (ctrlStats.highest_seq >= (STRESS_TEST_TARGET_COUNT - 1U)) {
    return true;
  }
  if (ctrlStats.near_end_seen && ((nowMs - ctrlStats.last_rx_ms) >= STRESS_RX_IDLE_DONE_MS)) {
    return true;
  }
  return false;
}

bool isTelmRxFinished(uint32_t nowMs) {
  if (!telmStats.rx_started) {
    return false;
  }
  if (telmStats.highest_seq >= (STRESS_TEST_TARGET_COUNT - 1U)) {
    return true;
  }
  if (telmStats.near_end_seen && ((nowMs - telmStats.last_rx_ms) >= STRESS_RX_IDLE_DONE_MS)) {
    return true;
  }
  return false;
}

void updateStressCompletion() {
  const uint32_t now = millis();

  if ((THIS_ROLE == ROLE_FCU) && !gStressCtrlRxDone && isCtrlRxFinished(now)) {
    gStressCtrlRxDone = true;
    ctrlStats.rx_end_ms = ctrlStats.last_rx_ms;
    LOG_INF("CTRL stress RX finished (highest_seq=%lu)", static_cast<unsigned long>(ctrlStats.highest_seq));
  }

  if ((THIS_ROLE == ROLE_REMOTE) && !gStressTelmRxDone && isTelmRxFinished(now)) {
    gStressTelmRxDone = true;
    telmStats.rx_end_ms = telmStats.last_rx_ms;
    LOG_INF("TELM stress RX finished (highest_seq=%lu)", static_cast<unsigned long>(telmStats.highest_seq));
  }

  const bool localTxFinished = (THIS_ROLE == ROLE_REMOTE) ? ctrlStats.tx_finished : telmStats.tx_finished;
  const bool localRxFinished = (THIS_ROLE == ROLE_REMOTE) ? gStressTelmRxDone : gStressCtrlRxDone;

  if (!gStressFinalMode && localTxFinished && localRxFinished) {
    gStressFinalMode = true;
    LOG_OK("Stress test complete locally. Final report will repeat every 2 seconds.");
  }
}

const char* lossText(const StreamStats& s, char* out, size_t outLen) {
  const uint32_t denom = s.rx_ok + s.gaps;
  if (denom < 100U) {
    snprintf(out, outLen, "n/a");
  } else {
    const float loss = (static_cast<float>(s.gaps) * 100.0f) / static_cast<float>(denom);
    snprintf(out, outLen, "%0.2f%%", loss);
  }
  return out;
}

void printStreamFinal(const char* name, const StreamStats& s) {
  char lossBuf[16];
  const char* loss = lossText(s, lossBuf, sizeof(lossBuf));

  const uint32_t txPps = s.tx_started ? rateFromWindow(s.tx_count, s.tx_start_ms, s.tx_end_ms) : 0;
  const uint32_t txBps = s.tx_started ? rateFromWindow(s.tx_bytes, s.tx_start_ms, s.tx_end_ms) : 0;
  const uint32_t rxPps = s.rx_started ? rateFromWindow(s.rx_ok, s.rx_start_ms, s.rx_end_ms) : 0;
  const uint32_t rxBps = s.rx_started ? rateFromWindow(s.rx_bytes, s.rx_start_ms, s.rx_end_ms) : 0;

  if (s.iat_count > 0) {
    const uint32_t iatAvg = s.iat_sum / s.iat_count;
    Serial.printf("%s: tx=%lu fail=%lu rx_ok=%lu gaps=%lu dup=%lu resets=%lu bad_crc=%lu bad_magic=%lu "
                  "loss=%s tx_pps=%lu tx_Bps=%lu rx_pps=%lu rx_Bps=%lu iat_ms(min/avg/max)=%lu/%lu/%lu\n",
                  name,
                  static_cast<unsigned long>(s.tx_count),
                  static_cast<unsigned long>(s.tx_fail),
                  static_cast<unsigned long>(s.rx_ok),
                  static_cast<unsigned long>(s.gaps),
                  static_cast<unsigned long>(s.duplicates),
                  static_cast<unsigned long>(s.seq_resets),
                  static_cast<unsigned long>(s.rx_bad_crc),
                  static_cast<unsigned long>(s.rx_bad_magic),
                  loss,
                  static_cast<unsigned long>(txPps),
                  static_cast<unsigned long>(txBps),
                  static_cast<unsigned long>(rxPps),
                  static_cast<unsigned long>(rxBps),
                  static_cast<unsigned long>(s.iat_min),
                  static_cast<unsigned long>(iatAvg),
                  static_cast<unsigned long>(s.iat_max));
  } else {
    Serial.printf("%s: tx=%lu fail=%lu rx_ok=%lu gaps=%lu dup=%lu resets=%lu bad_crc=%lu bad_magic=%lu "
                  "loss=%s tx_pps=%lu tx_Bps=%lu rx_pps=%lu rx_Bps=%lu iat_ms=n/a\n",
                  name,
                  static_cast<unsigned long>(s.tx_count),
                  static_cast<unsigned long>(s.tx_fail),
                  static_cast<unsigned long>(s.rx_ok),
                  static_cast<unsigned long>(s.gaps),
                  static_cast<unsigned long>(s.duplicates),
                  static_cast<unsigned long>(s.seq_resets),
                  static_cast<unsigned long>(s.rx_bad_crc),
                  static_cast<unsigned long>(s.rx_bad_magic),
                  loss,
                  static_cast<unsigned long>(txPps),
                  static_cast<unsigned long>(txBps),
                  static_cast<unsigned long>(rxPps),
                  static_cast<unsigned long>(rxBps));
  }
}

void printStressProgressOncePerSecond() {
  const uint32_t now = millis();
  if ((now - lastStressProgressMs) < STRESS_PROGRESS_PERIOD_MS) {
    return;
  }
  lastStressProgressMs = now;

  LOG_INF("[STRESS] role=%s tx_done(local)=%s rx_done(local)=%s "
          "CTRL(tx=%lu fail=%lu rx=%lu gaps=%lu) TELM(tx=%lu fail=%lu rx=%lu gaps=%lu)",
          roleName(THIS_ROLE),
          boolText((THIS_ROLE == ROLE_REMOTE) ? ctrlStats.tx_finished : telmStats.tx_finished),
          boolText((THIS_ROLE == ROLE_REMOTE) ? gStressTelmRxDone : gStressCtrlRxDone),
          static_cast<unsigned long>(ctrlStats.tx_count),
          static_cast<unsigned long>(ctrlStats.tx_fail),
          static_cast<unsigned long>(ctrlStats.rx_ok),
          static_cast<unsigned long>(ctrlStats.gaps),
          static_cast<unsigned long>(telmStats.tx_count),
          static_cast<unsigned long>(telmStats.tx_fail),
          static_cast<unsigned long>(telmStats.rx_ok),
          static_cast<unsigned long>(telmStats.gaps));
}

void printStressFinalReportEvery2Seconds() {
  const uint32_t now = millis();
  if ((now - lastStressFinalReportMs) < STRESS_FINAL_REPORT_PERIOD_MS) {
    return;
  }
  lastStressFinalReportMs = now;

  const bool localTxFinished = (THIS_ROLE == ROLE_REMOTE) ? ctrlStats.tx_finished : telmStats.tx_finished;
  const bool localRxFinished = (THIS_ROLE == ROLE_REMOTE) ? gStressTelmRxDone : gStressCtrlRxDone;

  Serial.println("===== STRESS FINAL REPORT =====");
  Serial.printf("role=%s target=%lu local_tx_finished=%s local_rx_finished=%s\n",
                roleName(THIS_ROLE),
                static_cast<unsigned long>(STRESS_TEST_TARGET_COUNT),
                boolText(localTxFinished),
                boolText(localRxFinished));
  if (THIS_ROLE == ROLE_REMOTE) {
    Serial.println("direction: CTRL TX (REMOTE->FCU), TELM RX (FCU->REMOTE)");
  } else {
    Serial.println("direction: CTRL RX (REMOTE->FCU), TELM TX (FCU->REMOTE)");
  }
  Serial.printf("stream_done: ctrl_tx=%s ctrl_rx=%s telm_tx=%s telm_rx=%s\n",
                boolText(ctrlStats.tx_finished),
                boolText(gStressCtrlRxDone),
                boolText(telmStats.tx_finished),
                boolText(gStressTelmRxDone));
  Serial.printf("listen_flags: CTRL=%s TELM=%s\n", boolText(gCtrlListening), boolText(gTelmListening));
  printStreamFinal("CTRL", ctrlStats);
  printStreamFinal("TELM", telmStats);
  Serial.println("================================");
}
#endif

// ---------------- public API ----------------

void radioLinkInit() {
  prepRadioPinsAfterBootDelay();
  hspi.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI);

  LOG_INF("Boot role=%s stress=%s big_packet=%s",
          roleName(THIS_ROLE),
          STRESS_TEST_ENABLE ? "ON" : "OFF",
          TEST_MODE_BIG_PACKET ? "ON" : "OFF");
  LOG_INF("NRF HSPI pins: SCK=%u MISO=%u MOSI=%u @%luHz",
          PIN_NRF_SCK,
          PIN_NRF_MISO,
          PIN_NRF_MOSI,
          static_cast<unsigned long>(NRF_SPI_HZ));
  LOG_INF("CTRL CE=%u CSN=%u IRQ=%u | TELM CE=%u CSN=%u IRQ=%u",
          PIN_CTRL_CE,
          PIN_CTRL_CSN,
          PIN_CTRL_IRQ,
          PIN_TELM_CE,
          PIN_TELM_CSN,
          PIN_TELM_IRQ);

  configureCtrlRadio(THIS_ROLE);
  configureTelmRadio(THIS_ROLE);

  gCtrlIrqPending = false;
  gTelmIrqPending = false;
  attachInterrupt(digitalPinToInterrupt(PIN_CTRL_IRQ), onCtrlIrq, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_TELM_IRQ), onTelmIrq, FALLING);
  LOG_INF("IRQ attached: CTRL=%u TELM=%u", PIN_CTRL_IRQ, PIN_TELM_IRQ);

  Serial.println("----- CTRL printDetails() -----");
  radioCtrl.printDetails();
  Serial.println("----- TELM printDetails() -----");
  radioTelm.printDetails();

  printExpectedConfig("CTRL", CTRL_CFG);
  printExpectedConfig("TELM", TELM_CFG);

  if (THIS_ROLE == ROLE_FCU) {
    LOG_INF("Listening plan: CTRL=YES TELM=NO");
  } else {
    LOG_INF("Listening plan: CTRL=NO TELM=YES");
  }

  printRuntimeSummary("CTRL", radioCtrl, CTRL_CFG, gCtrlListening);
  printRuntimeSummary("TELM", radioTelm, TELM_CFG, gTelmListening);

  const bool ctrlChipOk = radioCtrl.isChipConnected();
  const bool telmChipOk = radioTelm.isChipConnected();
  if (ctrlChipOk && telmChipOk) {
    LOG_OK("NRF init status: CTRL=YES TELM=YES");
  } else {
    LOG_ERR("NRF init status: CTRL=%s TELM=%s", boolText(ctrlChipOk), boolText(telmChipOk));
  }

#if STRESS_TEST_ENABLE
  LOG_INF("Stress target=%lu CTRL_interval_ms=%lu TELM_interval_ms=%lu",
          static_cast<unsigned long>(STRESS_TEST_TARGET_COUNT),
          static_cast<unsigned long>(STRESS_CTRL_SEND_INTERVAL_MS),
          static_cast<unsigned long>(STRESS_TELM_SEND_INTERVAL_MS));
#endif
}

void radioLinkTick() {
#if STRESS_TEST_ENABLE
  if (!gStressFinalMode) {
    // High-priority path: schedule CTRL TX first.
    sendStressCtrlIfDue();
    sendStressTelmIfDue();
    drainCtrlRxIfPending();
    drainTelmRxIfPending();
    updateStressCompletion();
    printStressProgressOncePerSecond();
  } else {
    // Keep draining RX in case some late packets are still in FIFOs.
    drainCtrlRxIfPending();
    drainTelmRxIfPending();
    printStressFinalReportEvery2Seconds();
  }
#else
  // High-priority path: schedule CTRL TX first.
  sendNormalCtrlIfDue();
  sendNormalTelmIfDue();
  drainCtrlRxIfPending();
  drainTelmRxIfPending();
  updateNormalLinkEvents();
  printCompactSummaryOncePerSecond();
#endif
}

bool radioLinkIsRemoteRole() {
  return THIS_ROLE == ROLE_REMOTE;
}

bool radioLinkStressModeEnabled() {
  return STRESS_TEST_ENABLE != 0;
}

bool radioLinkBigPacketModeEnabled() {
  return TEST_MODE_BIG_PACKET != 0;
}

void fillStatsSnapshot(const StreamStats& src, RadioLinkStatsSnapshot* out) {
  if (out == nullptr) {
    return;
  }

  out->tx_count = src.tx_count;
  out->tx_fail = src.tx_fail;
  out->tx_bytes = src.tx_bytes;
  out->rx_ok = src.rx_ok;
  out->rx_bad_crc = src.rx_bad_crc;
  out->rx_bad_magic = src.rx_bad_magic;
  out->payload_corrupt = src.payload_corrupt;
  out->rx_bytes = src.rx_bytes;
  out->last_seq = src.last_seq;
  out->highest_seq = src.highest_seq;
  out->gaps = src.gaps;
  out->duplicates = src.duplicates;
  out->seq_resets = src.seq_resets;
}

void applyRuntimeToRadio(RF24& radio,
                         uint8_t channel,
                         rf24_datarate_e data_rate,
                         rf24_crclength_e crc_len,
                         uint8_t pa_level,
                         bool auto_ack,
                         uint8_t retry_delay,
                         uint8_t retry_count) {
  radio.setChannel(channel);
  radio.setDataRate(data_rate);
  radio.setCRCLength(crc_len);
  radio.setPALevel(static_cast<rf24_pa_dbm_e>(pa_level));
  radio.setAutoAck(auto_ack);
  radio.setRetries(retry_delay, retry_count);
}

void radioLinkGetUiSnapshot(RadioLinkUiSnapshot* out) {
  if (out == nullptr) {
    return;
  }

  const uint32_t now = millis();
  out->is_remote_role = (THIS_ROLE == ROLE_REMOTE);
  out->stress_mode_enabled = (STRESS_TEST_ENABLE != 0);
  out->big_packet_mode_enabled = (TEST_MODE_BIG_PACKET != 0);

  out->ctrl_link_ok = ctrlLinkOkNow(now);
  out->telm_link_ok = telmLinkOkNow(now);

  out->ctrl_valid = lastCtrlPkt.valid;
  out->telm_valid = lastTelmPkt.valid;

  out->ctrl_seq = lastCtrlPkt.seq;
  out->telm_seq = lastTelmPkt.seq;

  out->ctrl_age_ms = lastCtrlPkt.valid ? static_cast<uint32_t>(now - lastCtrlPkt.rx_local_ms) : 0;
  out->telm_age_ms = lastTelmPkt.valid ? static_cast<uint32_t>(now - lastTelmPkt.rx_local_ms) : 0;
}

void radioLinkGetStatsSnapshot(RadioLinkStatsSnapshot* ctrl_out, RadioLinkStatsSnapshot* telm_out) {
  fillStatsSnapshot(ctrlStats, ctrl_out);
  fillStatsSnapshot(telmStats, telm_out);
}

void radioLinkGetChipStatus(bool* ctrl_connected, bool* telm_connected) {
  if (ctrl_connected != nullptr) {
    *ctrl_connected = radioCtrl.isChipConnected();
  }
  if (telm_connected != nullptr) {
    *telm_connected = radioTelm.isChipConnected();
  }
}

void radioLinkGetRuntimeSettings(RadioRuntimeSettings* out) {
  if (out == nullptr) {
    return;
  }
  *out = gRuntimeSettings;
}

bool radioLinkApplyRuntimeSettings(const RadioRuntimeSettings* in, bool* requires_restart) {
  if (requires_restart != nullptr) {
    *requires_restart = false;
  }
  if (in == nullptr) {
    return false;
  }

  applyRuntimeToRadio(radioCtrl,
                      in->ctrl_channel,
                      in->ctrl_data_rate,
                      in->ctrl_crc_len,
                      in->ctrl_pa_level,
                      in->ctrl_auto_ack,
                      in->ctrl_retry_delay,
                      in->ctrl_retry_count);

  applyRuntimeToRadio(radioTelm,
                      in->telm_channel,
                      in->telm_data_rate,
                      in->telm_crc_len,
                      in->telm_pa_level,
                      in->telm_auto_ack,
                      in->telm_retry_delay,
                      in->telm_retry_count);

  gRuntimeSettings = *in;
  return true;
}
