#include "WallFollower.h"

// Maximum distance for the ultrasonic sensors (in cm)
const int maxDistance = 300;

const int buttonPins[] = {9, 10, 11};
uint32_t previousMillis[WallFollower::noOfButtons];
uint8_t pressCount[WallFollower::noOfButtons];
uint8_t testCount[WallFollower::noOfButtons]; 

// Initialize the ultrasonic sensors
NewPing frontSensor(WallFollower::frontTrigPin, WallFollower::frontEchoPin, maxDistance);
NewPing leftSensor(WallFollower::leftTrigPin, WallFollower::leftEchoPin, 80);

// Initialize the encoders
Encoder encoder1(WallFollower::encoder1PinA, WallFollower::encoder1PinB);
Encoder encoder2(WallFollower::encoder2PinA, WallFollower::encoder2PinB);

// PID controller parameters
const float Kp = 1.05;
const float Ki = 0.05;
const float Kd = 0.1;
float integral = 0;
float previousError = 0;
unsigned long previousTime = 0;

// Desired distance from the left wall (in cm)
const int desiredDistance = 20;
bool startGame = false;

// WallFollower::WallFollower() {
// }

WallFollower::WallFollower() : lcd(9, 8, 7, 6, 5, 4) {
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
  // Set up the serial communication
  Serial.begin(9600);
  startGame = true;
  // Set up the LCD display
  lcd.begin(16, 2);
  lcd.print("Wall Follower");
}

void WallFollower::loop() {
  //get button presses
  // debounce();

  if(startGame==true){
    // Read the distance from the ultrasonic sensors
    long frontDistance = frontSensor.ping_cm();
    long leftDistance = leftSensor.ping_cm();

    // Calculate the time since the last loop iteration
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Calculate the PID controller output
    float error = desiredDistance - leftDistance;
    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // Control the motors based on the PID output
    if (frontDistance > 35) {
      // Move forward and adjust the speed based on the PID output
      int baseSpeed = 80;
      int motor1Speed = baseSpeed + output;
      int motor2Speed = baseSpeed - output;

      // Constrain the motor speeds
      motor1Speed = constrain(motor1Speed, 0, 180);
      motor2Speed = constrain(motor2Speed, 0, 180);

      // Set the motor speeds and directions
      Serial.println("Moving forward");
      digitalWrite(WallFollower::motor1In1, HIGH);
      digitalWrite(WallFollower::motor1In2, LOW);
      analogWrite(WallFollower::motor1Pwm, motor1Speed);
      digitalWrite(WallFollower::motor2In1, HIGH);
      digitalWrite(WallFollower::motor2In2, LOW);
      analogWrite(WallFollower::motor2Pwm, motor2Speed);

      // Update the LCD display
      lcd.setCursor(0,0);
      lcd.print("State: ")
      lcd.print("Moving Forward")
      lcd.setCursor(0, 1);
      lcd.print("F:");
      lcd.print(frontDistance);
      lcd.print(" L:");
      lcd.print(leftDistance);

    } else if(frontDistance > 2 && frontDistance < 45) {
      // Update the LCD display
      lcd.setCursor(0,0);
      lcd.print("State: ")
      lcd.print("Turning")
      lcd.setCursor(0, 1);
      lcd.print("F:");
      lcd.print(frontDistance);
      lcd.print(" L:");
      lcd.print(leftDistance);
      // Stop
      Serial.println("Stoping");
      digitalWrite(WallFollower::motor1In1, LOW);
      digitalWrite(WallFollower::motor1In2, LOW);
      analogWrite(WallFollower::motor1Pwm, 0);
      digitalWrite(WallFollower::motor2In1, LOW);
      digitalWrite(WallFollower::motor2In2, LOW);
      analogWrite(WallFollower::motor2Pwm, 0);

      delay(500);
      //Turn
      while(frontDistance < 40){
        digitalWrite(WallFollower::motor1In1, HIGH);
        digitalWrite(WallFollower::motor1In2, LOW);
        analogWrite(WallFollower::motor1Pwm, 70);
        digitalWrite(WallFollower::motor2In1, LOW);
        digitalWrite(WallFollower::motor2In2, HIGH);
        analogWrite(WallFollower::motor2Pwm, 70);
        frontDistance = frontSensor.ping_cm();
        Serial.println(frontDistance);
        Serial.println("Turning Right");
        lcd.setCursor(0, 1);
        lcd.print("F:");
        lcd.print(frontDistance);
      }

      delay(500);

      // Stop
      Serial.println("Stoping");
      digitalWrite(WallFollower::motor1In1, LOW);
      digitalWrite(WallFollower::motor1In2, LOW);
      analogWrite(WallFollower::motor1Pwm, 0);
      digitalWrite(WallFollower::motor2In1, LOW);
      digitalWrite(WallFollower::motor2In2, LOW);
      analogWrite(WallFollower::motor2Pwm, 0);

      delay(1500);

      //Move forward
      Serial.println("Moving Forward");
      digitalWrite(WallFollower::motor1In1, HIGH);
      digitalWrite(WallFollower::motor1In2, LOW);
      analogWrite(WallFollower::motor1Pwm, 0);
      digitalWrite(WallFollower::motor2In1, HIGH);
      digitalWrite(WallFollower::motor2In2, LOW);
      analogWrite(WallFollower::motor2Pwm, 0);
    }

  // Print the distance and encoder values
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.print(" cm, Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Error: ");
  Serial.println(output);

  // Wait for a while
  // delay(20);
  }
}

//Button debounce
void debounce() {
  uint8_t i;
  uint32_t currentMillis = millis();
  for (i = 0; i < WallFollower::noOfButtons; ++i) {
    if (digitalRead(buttonPins[i])){            //Input is high, button not pressed or in the middle of bouncing and happens to be high
      previousMillis[i] = currentMillis;        //Set previousMillis to millis to reset timeout
      pressCount[i] = 0;                        //Set the number of times the button has been detected as pressed to 0
    }
    else if (currentMillis - previousMillis[i] > WallFollower::bounceDelay) {
      previousMillis[i] = currentMillis;        //Set previousMillis to millis to reset timeout
      ++pressCount[i];
      if (pressCount[i] == WallFollower::minButtonPress) {
        startGame = true;                             //Button has been debounced. Call function to do whatever you want done.
      }
    }
  }
}