#include <Servo.h>


const int SERVO_PIN = 7;

Servo myservo;  // create servo object to control a servo


void setup() {
  myservo.attach(SERVO_PIN);  // attaches the servo pin to the servo object
  Serial.begin(9600);
}

void loop() {
  myservo.write(0);   //Set servo's position to 0 degree
  delay(3000);
  myservo.write(180);
  delay(3000); 
}
