#include <MoteusAcan2517fd.h>

#define VELOCITY 1

// MCP2517 pins for CAN FD Arduino Shield
constexpr byte MCP2517_SCK = 13;  // SCK
constexpr byte MCP2517_SDI = 11;  // SDI (MOSI)
constexpr byte MCP2517_SDO = 12;  // SDO (MISO)
constexpr byte MCP2517_CS = 9;    // CS
constexpr byte MCP2517_INT = 2;   // INT (A)

constexpr long CANFD_BITRATE = 1000ll * 1000ll;  // 1 MBit bitrate for CANFD

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

Moteus* Motor1;
Moteus* Motor2;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  SPI.begin();

  // Run CAN-FD at 1 Mbit/s for both arbitration and data.
  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_20MHz, CANFD_BITRATE, DataBitRateFactor::x1);

  // Keep the driver buffers small to fit on memory-constrained boards.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;

  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
  while (errorCode != 0) {
    Serial.print(F("CAN error 0x"));
    Serial.println(errorCode, HEX);
    delay(1000);
  }

   Motor1 = new Moteus(can, []() {
      Moteus::Options options;
      options.id = 1;
      return options;
   }());

   Motor2 = new Moteus(can, []() {
      Moteus::Options options;
      options.id = 2;
      return options;
   }());

  // Clear any faults on motor ID 1.
  Motor1->SetStop();
  Motor2->SetStop();
  Serial.println(F("motor 1 ready"));
}

void loop() {

  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;  // Pure velocity mode.
  cmd.velocity = VELOCITY;

   Motor1->SetPosition(cmd);
   Motor2->SetPosition(cmd);

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
