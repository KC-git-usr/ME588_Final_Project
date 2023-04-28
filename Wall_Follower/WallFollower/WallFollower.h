#ifndef WallFollower_h
#define WallFollower_h

#include <Arduino.h>
#include <NewPing.h>
#include <Encoder.h>
#include <LiquidCrystal.h>

class WallFollower {
public:
  WallFollower();
  LiquidCrystal lcd;
  void setup();
  void loop();

  // Motor pins
  static const int motor1In1 = 25;
  static const int motor1In2 = 27;
  static const int motor1Pwm = 2;
  static const int motor2In1 = 29;
  static const int motor2In2 = 31;
  static const int motor2Pwm = 3;

  // Hall effect encoder pins
  static const int encoder1PinA = 19;
  static const int encoder1PinB = 43;
  static const int encoder2PinA = 18;
  static const int encoder2PinB = 41;

  // Ultrasonic sensor pins
  static const int frontTrigPin = 36;
  static const int frontEchoPin = 34;
  static const int leftTrigPin = 53;
  static const int leftEchoPin = 51;
  static const int midTrigPin = 15;
  static const int midEchoPin = 14;

  //Buttons
  static const int noOfButtons = 3;
  static const int bounceDelay = 30;
  static const int minButtonPress = 1;

  //IR Digital sensor
  static const int irsensor = 46;

  //LF sensor pins
  static const int LF1 = A0;
  static const int LF2 = A1;

};

#endif
