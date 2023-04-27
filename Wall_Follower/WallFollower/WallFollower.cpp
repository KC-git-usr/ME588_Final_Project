#include "WallFollower.h"

// Defining time limit
const unsigned long TIME_LIMIT = 120000; // [ms]

// Variable to hold the 2min timer
unsigned long time_since_prgm_start = 0;
unsigned long time_at_SG = 0;

// Maximum distance for the ultrasonic sensors (in cm)
const int maxDistance = 300;

const int buttonPins[] = {35, 37};
uint32_t previousMillis[WallFollower::noOfButtons];
uint8_t pressCount[WallFollower::noOfButtons];
uint8_t testCount[WallFollower::noOfButtons];


WallFollower::WallFollower(): 
frontSensor(WallFollower::frontTrigPin, WallFollower::frontEchoPin, maxDistance), 
leftSensor(WallFollower::leftTrigPin, WallFollower::leftEchoPin, 80), 
lcd(9, 8, 7, 6, 5, 4), 
encoder1(WallFollower::encoder1PinA, WallFollower::encoder1PinB), 
encoder2(WallFollower::encoder2PinA, WallFollower::encoder2PinB), 
pidController(0, 0, 0, 0, 0) {

  // Distance per encoder count
  distancePerCount = 0.5 * 3.14159 * 6.0 / 20.0;

  // Target speed
  targetSpeed = 100.0;

  // Maximum speed difference
  maxSpeedDifference = 50;

  // PID controller
  Kp = 1.0;
  Ki = 0.0;
  Kd = 0.0;
  pidController.SetOutputLimits(-maxSpeedDifference, maxSpeedDifference);
  pidController.SetMode(AUTOMATIC);
  pidController.SetSampleTime(10);
  pidController.SetTunings(Kp, Ki, Kd);

  // Wall following state
  wallFollowingState = GO_STRAIGHT;

  // Turn count
  turnCount = 0;
}

void WallFollower::setup() {
  // Set up the motor pins
  pinMode(WallFollower::motor1In1, OUTPUT);
  pinMode(WallFollower::motor1In2, OUTPUT);
  pinMode(WallFollower::motor1Pwm, OUTPUT);
  pinMode(WallFollower::motor2In1, OUTPUT);
  pinMode(WallFollower::motor2In2, OUTPUT);
  pinMode(WallFollower::motor2Pwm, OUTPUT);

  //button Setup
  for (int i = 0; i < WallFollower::noOfButtons; ++i) {
      pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Encoder pins
  pinMode(WallFollower::encoder1PinA, INPUT_PULLUP);
  pinMode(WallFollower::encoder1PinB, INPUT_PULLUP);
  pinMode(WallFollower::encoder2PinA, INPUT_PULLUP);
  pinMode(WallFollower::encoder2PinB, INPUT_PULLUP);

  // LCD display
  lcd.begin(16, 2);
  lcd.print("Wall Follower");
  lcd.setCursor(0,1);
  lcd.print(colors[color]);

  // Wait for the LCD display to initialize
  delay(1000);
}

void WallFollower::loop() {
  // Read the ultrasonic sensor distances
  int frontDistance = frontSensor.ping_cm();
  int leftDistance = leftSensor.ping_cm();

  // Determine the wall following state
  if (frontDistance < 10 || leftDistance < 10) {
    wallFollowingState = TURN_RIGHT;
  }

  // Handle the wall following state
  switch (wallFollowingState) {
    case GO_STRAIGHT:
      goStraight();
      break;
    case TURN_RIGHT:
      turnRight();
      break;
  }
}

void WallFollower::goStraight() {
  // Reset the encoder counts
  encoder1.write(0);
  encoder2.write(0);

  // Set the motor speeds to the target speed
  int leftSpeed = targetSpeed;
  int rightSpeed = targetSpeed;

  // Loop until a wall is detected
  while (true) {
    // Read the ultrasonic sensor distances
    int frontDistance = frontSensor.ping_cm();
    int leftDistance = leftSensor.ping_cm();

    // Determine the wall following state
    if (frontDistance < 10 || leftDistance < 10) {
      wallFollowingState = TURN_RIGHT;
      break;
    }

    // Read the encoder counts
    long encoder1Count = encoder1.read();
    long encoder2Count = encoder2.read();

    // Calculate the average encoder count
    long averageEncoderCount = (encoder1Count + encoder2Count) / 2;

    // Calculate the distance traveled
    long distanceTraveled = averageEncoderCount * distancePerCount;

    // Calculate the speed difference using the encoder counts and PID control
    long speedDifference = calculateSpeedDifference(encoder1Count, encoder2Count);

    // Set the motor speeds based on the speed difference
    setMotorSpeeds(leftSpeed - speedDifference, rightSpeed + speedDifference);

    // Update the LCD display
    updateLCD(distanceTraveled);
  }

  // Stop the motors
  setMotorSpeeds(0, 0);
}

void WallFollower::turnRight() {
  // Reset the encoder counts
  encoder1.write(0);
  encoder2.write(0);

  // Set the motor speeds to turn right
  int leftSpeed = targetSpeed;
  int rightSpeed = -targetSpeed;

  // Loop until the robot has turned 90 degrees
  long angleTurned = 0;
  while (angleTurned < 90) {
    // Read the encoder counts
    long encoder1Count = encoder1.read();
    long encoder2Count = encoder2.read();

    // Calculate the average encoder count
    long averageEncoderCount = (encoder1Count + encoder2Count) / 2;

    // Calculate the angle turned
    angleTurned = averageEncoderCount * distancePerCount / 6.0 * 360.0;

    // Set the motor speeds to turn right
    setMotorSpeeds(leftSpeed, rightSpeed);

    // Update the LCD display
    updateLCD(angleTurned);
  }

  // Stop the motors
  setMotorSpeeds(0, 0);

  // Increment the turn count
  turnCount++;

  // Determine the wall following state
  if (turnCount < 4) {
    wallFollowingState = GO_STRAIGHT;
  } else {
    wallFollowingState = TURN_RIGHT;
    turnCount = 0;
  }
}

long WallFollower::calculateSpeedDifference(long encoder1Count, long encoder2Count) {
  // Calculate the difference between the encoder counts
  long countDifference = encoder1Count - encoder2Count;

  // Calculate the speed difference using the PID control algorithm
  double speedDifference = pidController.Compute(countDifference);

  // Convert the speed difference to an integer value
  int speedDifferenceInt = (int)speedDifference;

  // Limit the speed difference to the maximum value
  if (speedDifferenceInt > maxSpeedDifference) {
    speedDifferenceInt = maxSpeedDifference;
  } else if (speedDifferenceInt < -maxSpeedDifference) {
    speedDifferenceInt = -maxSpeedDifference;
  }

  // Return the speed difference
  return speedDifferenceInt;
}

void WallFollower::setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Set the motor speeds based on the input values
  analogWrite(WallFollower::motor1Pwm, abs(leftSpeed));
  analogWrite(WallFollower::motor2Pwm, abs(rightSpeed));
  digitalWrite(WallFollower::motor1In1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(WallFollower::motor1In2, leftSpeed > 0 ? LOW : HIGH);
  digitalWrite(WallFollower::motor2In1, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(WallFollower::motor2In2, rightSpeed > 0 ? LOW : HIGH);
}

void WallFollower::updateLCD(long distanceTraveled) {
  // Update the LCD display with the distance traveled or angle turned
  lcd.clear();
  lcd.setCursor(0, 0);
  if (wallFollowingState == GO_STRAIGHT) {
    lcd.print("Distance: ");
    lcd.print(distanceTraveled);
    lcd.print(" cm");
  } else {
    lcd.print("Angle: ");
    lcd.print(distanceTraveled);
    lcd.print(" deg");
  }
}
