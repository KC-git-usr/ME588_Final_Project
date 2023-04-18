#include <LiquidCrystal.h>

// defines pins numbers
const int trigPin = 24;
const int echoPin = 25;
const int US_LED = 26;

// defines variables
long duration;
int distance1;
int distance;

const int LF1 = A0;
const int LF2 = A1;
const int LF_LED = 27;


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  USDistance();
  LineFollow();
  delay(100);
}

void USDistance(){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = (duration * 0.034 / 2); 
  distance = (distance*.4474) - 1.285;
  // Prints the distance on the Serial Monitor
  if (distance >= 150){
    Serial.println("Sensor is clear");
  }
  else{
  // Serial.print("Serial read: ");
  // Serial.println(duration);
  Serial.print("Distance: ");
  Serial.println(distance);
  
}
  if (distance <= 13){
    Serial.print("Approaching wall");
    if (distance <= 6){
      Serial.print("Robot too close to wall, back up");
      digitalWrite(US_LED, HIGH);
    }
  }
  else{
    digitalWrite(US_LED, LOW);
  }


}

void LineFollow(){
  int counter = 0;
  int lineCross1 = analogRead(LF1);
  int lineCross2 = analogRead(LF2);
  Serial.print("LF 1 value:  ");
  Serial.println(lineCross1);

  Serial.print("LF 2 value:  ");
  Serial.println(lineCross2);
  int LED = 0;
  // CHANGE THESE THRESHOLD VALUES
  if (lineCross1 <= 900 && lineCross2 <= 900){ //tune threshold for LF
    counter = counter + 1;
    Serial.println("line crossed");
    //Serial.print("counter: "); Serial.println(counter);
    LED = HIGH;
  }
  else {
    LED = LOW;
  }
  digitalWrite(LF_LED, LED);
}
