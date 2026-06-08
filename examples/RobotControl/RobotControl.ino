#include <MoteusAcan2517fd.h>

constexpr short VELOCITY = 1;
constexpr float TURN_SPEED = -90; // in degrees
constexpr float SHORTKICK_POWER = 10;

constexpr byte TEST_NUM_MOTORS = 2;
constexpr byte NUM_MOTORS = 5;
constexpr byte NUM_WHEELS = 4;
constexpr byte TEST_DRIBBLER_INDEX = 0; // dribbler index for testing
constexpr byte DRIBBLER_INDEX = 4; // true dribbler index
constexpr float DASH_POWER = 1;
constexpr float DASH_ANGLE = 30; // in degrees

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
   static const auto start = millis();
   auto now = millis();

   if (now - start > 10000) { // if 10s have passed
      stopLocomotion(motors);
   } else {
      dash(motors,angles,invert,DASH_POWER,DASH_ANGLE);
   }

}

/**
 * @brief move the robot in a direction with a velocity power
 * @param wheels array wheel motors
 * @param wheelAngles angle of the wheels relative to front to back axis in radians
 * @param invertRotation which wheels have inverted rotation from orientation
 * @param power velocity of motor in rev/s
 * @param direction direction of motion in degrees
 */
void dash(Moteus* wheels[NUM_WHEELS], float wheelAngles[NUM_WHEELS], bool invertRotation[NUM_WHEELS], float dashPower, float direction) {
   direction *= PI / 180; // convert direction from degrees into radians
   Moteus::PositionMode::Command cmd;
   cmd.position = NaN;  // Pure velocity mode.
   for(int i=0;i<TEST_NUM_MOTORS;i++) {
      cmd.velocity = dashPower * cos(wheelAngles[i] - direction);
      if (invertRotation[i]) {
         cmd.velocity *= -1;
      }
      motors[i]->SetPosition(cmd);
   }
}

/**
 * @brief rotate the robot at some rotational velocity
 * @param wheels array wheel motors
 * @param turnSpeed rotational velocity in degrees/s
 * @warning the speed is not certain until tested on a robot 
 */
void turn(Moteus* wheels[NUM_WHEELS], float turnSpeed) {
   constexpr float arbitraryMultipler = 1; // this is for tuning during testing on a robot to match the specified rotational velocity
   Moteus::PositionMode::Command cmd;
   cmd.position = NaN;  // Pure velocity mode.
   for(int i=0;i<TEST_NUM_MOTORS;i++) {
      // convert from deg/s to wheel rev/s
      // deg/s / 360deg * robot circumference / robot circumference * 3 wheel revs per robot circumference = wheel rev/s
      cmd.velocity = -turnSpeed * arbitraryMultipler / 120; // convert rotational speed of robot to rotational speed of motors
      motors[i]->SetPosition(cmd);
   }
}

/**
 * @brief turn the dribbler motor on
 * @param motors array of motors
 * @warning the optimal dribbler speed is unknown until robot testing
*/
void dribblerCatch(Moteus* motors[NUM_MOTORS]) {
  constexpr float invertDribblerRotation = 1; // change to -1 if dribbler spins the wrong way
  constexpr float dribblerSpeed = 10; // this is for tuning the optimal dribbler speed during testing
  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;  // Pure velocity mode.
  cmd.velocity = invertDribblerRotation * dribblerSpeed;
  motors[TEST_DRIBBLER_INDEX]->SetPosition(cmd);
}

/**
 * @brief apply brake to the dribbler motor
 * @param motors array of motors
*/
void stopDribbler(Moteus* motors[NUM_MOTORS]) {
  motors[TEST_DRIBBLER_INDEX]->SetBrake();
}

/**
 * @brief apply brake to all wheel motors
 * @param motors array of wheel motors
*/
void stopLocomotion(Moteus* motors[NUM_WHEELS]) {
  for(int i=0;i<NUM_WHEELS;i++) {
    motors[i]->SetBrake();
  }
}

/**
 * @brief spin the dribbler backwards to spit the ball forward
 * @param motors array of motors
 * @param shortKickPower rotation speed of dribbler as a positive value
*/
void shortKick(Moteus* motors[NUM_MOTORS], float shortKickPower) {
  constexpr float invertDribblerRotation = -1; // change to 1 if dribbler spins the wrong way
  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;  // Pure velocity mode.
  cmd.velocity = invertDribblerRotation * shortKickPower;
  motors[TEST_DRIBBLER_INDEX]->SetPosition(cmd);
}