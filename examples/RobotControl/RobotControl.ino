#include <MoteusAcan2517fd.h>

constexpr short VELOCITY = 1;

constexpr short TEST_NUM_MOTORS = 2;
constexpr short NUM_MOTORS = 5;

// MCP2517 pins for CAN FD Arduino Shield
constexpr byte MCP2517_SCK = 13;  // SCK
constexpr byte MCP2517_SDI = 11;  // SDI (MOSI)
constexpr byte MCP2517_SDO = 12;  // SDO (MISO)
constexpr byte MCP2517_CS = 9;    // CS
constexpr byte MCP2517_INT = 2;   // INT (A)

constexpr long CANFD_BITRATE = 1000ll * 1000ll;  // 1 MBit bitrate for CANFD

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

const Moteus* motors[NUM_MOTORS] = {nullptr};

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

   for (int i=1;i<=TEST_NUM_MOTORS;i++) {
      motors[i] = new Moteus(can, [i]() {
         Moteus::Options options;
         options.id = i;
         return options;
      }());
      // Clear any faults
      motors[i]->SetStop();
   }

  Serial.println(F("motor 1 ready"));
}

void loop() {

  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;  // Pure velocity mode.
  cmd.velocity = VELOCITY;

   for(int i=1;i<=TEST_NUM_MOTORS;i++) {
      motors[i]->SetPosition(cmd);
   }
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
