
#include "MotorControl.h"

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
  pinMode(KICKER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("started"));

  SPI.begin();
   
  // Don't touch anything below

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
      motors[i]->BeginStop();
   }

   // Don't touch anything above

}

void loop() {
   stop(motors, KICKER_PIN);
   //turn(motors,TURN_SPEED);
   //dash(motors,angles,invert,DASH_POWER,DASH_ANGLE);
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
      motors[i]->BeginPosition(cmd);
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
      motors[i]->BeginPosition(cmd);
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
  motors[TEST_DRIBBLER_INDEX]->BeginPosition(cmd);
}

/**
 * @brief apply brake to the dribbler motor
 * @param motors array of motors
*/
void stopDribbler(Moteus* motors[NUM_MOTORS]) {
  motors[DRIBBLER_INDEX]->BeginBrake();
}

/**
 * @brief apply brake to all wheel motors
 * @param motors array of wheel motors
*/
void stopLocomotion(Moteus* motors[NUM_WHEELS]) {
  for(int i=0;i<NUM_WHEELS;i++) {
    motors[i]->BeginBrake();
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
  motors[TEST_DRIBBLER_INDEX]->BeginPosition(cmd);
}

/**
 * @brief turn kicker solenoid off
 * @param kickerPin GPIO pin for activating kicker solenoid
 */
void stopKicker(const byte kickerPin) {
   digitalWrite(kickerPin, LOW);
}

/**
 * @brief activate kicker solenoid for 100ms to kick ball
 * @param kickerPin GPIO pin for activating kicker solenoid
 */
void kick(const byte kickerPin) {
   digitalWrite(kickerPin, HIGH);
   // delay 100ms without blocking
   digitalWrite(kickerPin, LOW);
}


/**
 * @brief stop all motors and kicker
 * @param motors array of motors
 * @param kickerPin GPIO pin for activating kicker solenoid
 */
void stop(Moteus* motors[NUM_MOTORS], const byte kickerPin) {
   stopLocomotion(motors);
   stopDribbler(motors); // commented out for testing because treating a wheel motor as a dribbler motor bugs it out
   stopKicker(kickerPin);
}