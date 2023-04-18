void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
  myservo.write(0);   //Set servo's position to 0 degree
  delay(3000);
  myservo.write(180);
  delay(3000); 
}
