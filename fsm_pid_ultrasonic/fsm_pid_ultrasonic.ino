#include <NewPing.h>

#define LEFT_SENSOR_TRIGGER 22
#define LEFT_SENSOR_ECHO 24
#define FRONT_SENSOR_TRIGGER 26
#define FRONT_SENSOR_ECHO 28
#define LEFT_MOTOR_PWM 6
#define LEFT_MOTOR_DIR 7
#define RIGHT_MOTOR_PWM 5
#define RIGHT_MOTOR_DIR 4
#define LEFT_ENCODER_A 18
#define LEFT_ENCODER_B 19
#define RIGHT_ENCODER_A 2
#define RIGHT_ENCODER_B 3
#define ENCODER_TICKS_PER_REV 24
#define WHEEL_DIAMETER 6.5
#define BASE_SPEED 100
#define MIN_TURN_DISTANCE 30
#define TURN_SPEED 100

NewPing leftSensor(LEFT_SENSOR_TRIGGER, LEFT_SENSOR_ECHO, 100);
NewPing frontSensor(FRONT_SENSOR_TRIGGER, FRONT_SENSOR_ECHO, 100);

volatile int leftEncoderTicks = 0;
volatile int rightEncoderTicks = 0;

float targetSpeed = BASE_SPEED;
float leftError = 0;
float leftIntegral = 0;
float leftDerivative = 0;
float lastLeftError = 0;
float rightError = 0;
float rightIntegral = 0;
float rightDerivative = 0;
float lastRightError = 0;
unsigned long lastEncoderUpdateTime = 0;

enum State {
  FOLLOW_LEFT_WALL,
  TURN_RIGHT,
};

State state = FOLLOW_LEFT_WALL;

void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B) == LOW) {
    leftEncoderTicks++;
  } else {
    leftEncoderTicks--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == LOW) {
    rightEncoderTicks++;
  } else {
    rightEncoderTicks--;
  }
}

void leftPID() {
  unsigned long now = millis();
  if (now - lastEncoderUpdateTime >= 10) {
    lastEncoderUpdateTime = now;
    float speed = (float)leftEncoderTicks * (float)WHEEL_DIAMETER * PI / (float)ENCODER_TICKS_PER_REV / 0.01;
    leftEncoderTicks = 0;
    leftError = targetSpeed - speed;
    leftIntegral += leftError * 0.01;
    leftDerivative = (leftError - lastLeftError) / 0.01;
    lastLeftError = leftError;
    float pidOutput = 0.4 * leftError + 0.2 * leftIntegral + 0.2 * leftDerivative;
    int pwm = BASE_SPEED + (int)pidOutput;
    if (pwm < 0) {
      pwm = 0;
    } else if (pwm > 255) {
      pwm = 255;
    }
    analogWrite(LEFT_MOTOR_PWM, pwm);
  }
}

void rightPID() {
  unsigned long now = millis();
  if (now - lastEncoderUpdateTime >= 10) {
    lastEncoderUpdateTime = now;
    float speed = (float)rightEncoderTicks * (float)WHEEL_DIAMETER * PI / (float)ENCODER_TICKS_PER_REV / 0.01;
    rightEncoderTicks = 0;
    rightError = targetSpeed - speed;
    rightIntegral += rightError * 0.01;
    rightDerivative = (rightError - lastRightError) / 0.01;
    lastRightError = rightError;
    float pidOutput = 0.4 * rightError + 0.2 * rightIntegral + 0.2 * rightDerivative;
    int pwm = BASE_SPEED + (int)pidOutput;
    if (pwm < 0) {
      pwm = 0;
    } else if (pwm > 255) {
      pwm = 255;
    }
    analogWrite(RIGHT_MOTOR_PWM, pwm);
  }
}

void setup() {
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
  Serial.begin(9600);
}

void loop() {
  switch (state) {
    case FOLLOW_LEFT_WALL:
      targetSpeed = BASE_SPEED;
      leftPID();
      rightPID();
      if (frontSensor.ping_cm() < MIN_TURN_DISTANCE) {
        state = TURN_RIGHT;
      } else if (leftSensor.ping_cm() > 30) {
        digitalWrite(LEFT_MOTOR_DIR, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR, LOW);
      } else if (leftSensor.ping_cm() < 20) {
        digitalWrite(LEFT_MOTOR_DIR, LOW);
        digitalWrite(RIGHT_MOTOR_DIR, HIGH);
      }
      break;

    case TURN_RIGHT:
      analogWrite(LEFT_MOTOR_PWM, 0);
      analogWrite(RIGHT_MOTOR_PWM, TURN_SPEED);
      delay(1000);
      state = FOLLOW_LEFT_WALL;
      break;
  }
}


