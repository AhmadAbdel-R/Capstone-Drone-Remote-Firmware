// FCU reference template (NOT BUILT)
// ----------------------------------
// This file is intentionally stored outside /src, so PlatformIO does not build it.
//
// Goal:
// - Show a simple "other side" firmware layout that can pair with the current remote.
// - Keep it as a Git reference for future testing.

#if 0

#include <Arduino.h>
#include <RF24.h>

// Example-only pins; replace with your FCU wiring.
static constexpr uint8_t PIN_RF_CE = 5;
static constexpr uint8_t PIN_RF_CSN = 17;

RF24 radio(PIN_RF_CE, PIN_RF_CSN);

struct RemoteControlPacket {
  uint32_t seq;
  int16_t left_axis;
  int16_t right_x;
  int16_t right_y;
  uint8_t mode_index;
  uint8_t arm_state;
  uint8_t battery_pct;
};

struct FcuTelemetryPacket {
  uint32_t seq;
  uint8_t link_ok;
  uint8_t gps_fix;
  float lat;
  float lon;
  float altitude_m;
  uint16_t battery_mv;
};

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // Mirror the same channel/data-rate/CRC/PA settings as the remote runtime settings.
  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);

  // Example FCU: receive control packets.
  radio.openReadingPipe(1, 0xAABBCCDD11LL);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    RemoteControlPacket in = {};
    radio.read(&in, sizeof(in));

    // 1) Parse stick values and mode/arm commands.
    // 2) Feed them into FCU control logic.
    // 3) Optionally send telemetry back to the remote side.

    Serial.printf("CTRL seq=%lu L=%d RX=%d RY=%d mode=%u arm=%u batt=%u%%\n",
                  static_cast<unsigned long>(in.seq),
                  static_cast<int>(in.left_axis),
                  static_cast<int>(in.right_x),
                  static_cast<int>(in.right_y),
                  static_cast<unsigned>(in.mode_index),
                  static_cast<unsigned>(in.arm_state),
                  static_cast<unsigned>(in.battery_pct));
  }
}

#endif  // FCU reference template

