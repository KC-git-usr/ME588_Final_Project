//#include <Adafruit_LiquidCrystal.h>

#include <LiquidCrystal.h>

// defines pins numbers
const int trigPin = 53;
const int echoPin = 52;
// defines variables
long duration;
int distance;

const int LF1 = A0;
const int LF2 = A1;

//LiquidCrystal lcd(12, 11, 10, 5, 4, 3, 2);

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  //lcd.begin(16, 2);
  
}

void loop() {
  IRDistance();
  LineFollow();
  delay(1000);

  
}

void IRDistance(){
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
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  if (distance >= 150){
    Serial.println("Sensor is clear");
  }
  else{
  Serial.print("Distance: ");
  Serial.println(distance);
}
  if (distance <= 13){
    Serial.print("Approaching wall");
    if (distance <= 6){
      Serial.print("Robot too close to wall, back up");
    }
  }


}

void LineFollow(){
  int lineCross1 = analogRead(LF1);
  int lineCross2 = analogRead(LF2);
  Serial.print("LF 1 value:  ");
  Serial.println(lineCross1);

  Serial.print("LF 2 value:  ");
  Serial.println(lineCross2);
  if (lineCross1 <= 300 && lineCross2 <= 300){
    Serial.println("Line crossed");
  }
}
