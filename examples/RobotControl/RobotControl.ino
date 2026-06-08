#include <MoteusAcan2517fd.h>

constexpr short VELOCITY = 1;
constexpr float ANGLE_DEGREES = -70;
constexpr float ANGLE = ANGLE_DEGREES * PI / 180; // in radians

constexpr short TEST_NUM_MOTORS = 2;
constexpr short NUM_MOTORS = 5;
constexpr short NUM_WHEELS = 4;

// MCP2517 pins for CAN FD Arduino Shield
constexpr byte MCP2517_SCK = 13;  // SCK
constexpr byte MCP2517_SDI = 11;  // SDI (MOSI)
constexpr byte MCP2517_SDO = 12;  // SDO (MISO)
constexpr byte MCP2517_CS = 9;    // CS
constexpr byte MCP2517_INT = 2;   // INT (A)

constexpr long CANFD_BITRATE = 1000ll * 1000ll;  // 1 MBit bitrate for CANFD

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

/**
   @note motor indices 0-3 are wheel motors
   @note motor index 4 is the dribbler motor
*/
const Moteus* motors[NUM_MOTORS] = {nullptr};

/**
   @brief angle of wheel relative to the front to back axis in radians
   @note order: FrontLeft, FrontRight, BackRight, FrontLeft
   @note front angle is 20 degrees and back angle is 60 degrees
*/
constexpr float angles[NUM_WHEELS] = {
   0.349066,
   -0.349066,
   1.0472,
   -1.0472,
};

/**
   @brief should the velocity of the motor be inverted rotation
*/
constexpr bool invert[NUM_WHEELS] = {
   true,
   false,
   false,
   true
};

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

   for (int i=0;i<TEST_NUM_MOTORS;i++) {
      motors[i] = new Moteus(can, [i]() {
         Moteus::Options options;
         options.id = i+1;
         return options;
      }());
      // Clear any faults
      motors[i]->SetStop();
   }
}

void loop() {

  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;  // Pure velocity mode.

   for(int i=0;i<TEST_NUM_MOTORS;i++) {
      cmd.velocity = VELOCITY * cos(angles[i] - ANGLE);
      if (invert[i]) {
         cmd.velocity *= -1;
      }
      motors[i]->SetPosition(cmd);
   }
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
