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

  //Buttons
  static const int noOfButtons = 2;  // 0 is SG, 1 is color
  static const int bounceDelay = 30;
  static const int minButtonPress = 1;


  // LiquidCrystal LCD display pins
  // const int rs = 12;
  // const int en = 11;
  // const int d4 = 5;
  // const int d5 = 4;
  // const int d6 = 3;
  // const int d7 = 2;

};

#endif