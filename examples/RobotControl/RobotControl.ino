#include <MoteusAcan2517fd.h>

constexpr short VELOCITY = 1;
constexpr float ANGLE_DEGREES = 30;
constexpr float TURN_SPEED = -90; // in degrees

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

void dash(Moteus* wheels[NUM_WHEELS], float wheelAngles[NUM_WHEELS], bool invertRotation[NUM_WHEELS], float power, float direction);

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

   turn(motors,TURN_SPEED);
   //dash(motors,angles,invert,VELOCITY,ANGLE_DEGREES);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

/**
 * @brief move the robot in a direction with a velocity power
 * @param wheels array wheel motors
 * @param wheelAngles angle of the wheels relative to front to back axis in radians
 * @param invertRotation which wheels have inverted rotation from orientation
 * @param power velocity of motor in rev/s
 * @param direction direction of motion in degrees
 */
void dash(Moteus* wheels[NUM_WHEELS], float wheelAngles[NUM_WHEELS], bool invertRotation[NUM_WHEELS], float power, float direction) {
   direction *= PI / 180; // convert direction from degrees into radians
   Moteus::PositionMode::Command cmd;
   cmd.position = NaN;  // Pure velocity mode.
   for(int i=0;i<TEST_NUM_MOTORS;i++) {
      cmd.velocity = power * cos(wheelAngles[i] - direction);
      if (invertRotation[i]) {
         cmd.velocity *= -1;
      }
      motors[i]->SetPosition(cmd);
   }
}

/**
 * @brief rotate the robot at some rotational velocity
 * @param wheels array wheel motors
 * @param speed rotational velocity in degrees/s
 * @warning the speed is not certain until tested on a robot 
 */
void turn(Moteus* wheels[NUM_WHEELS], float speed) {
   constexpr float arbitraryMultipler = 1; // this is for tuning during testing on a robot to match the specified rotational velocity
   Moteus::PositionMode::Command cmd;
   cmd.position = NaN;  // Pure velocity mode.
   for(int i=0;i<TEST_NUM_MOTORS;i++) {
      cmd.velocity = -speed * arbitraryMultipler / 120; // convert rotational speed of robot to rotational speed of motors
      motors[i]->SetPosition(cmd);
   }
}
