#include "WallFollower.h"

#define DEBUG true

// Defining time limit
const unsigned long TIME_LIMIT = 120000; // [ms]

// Variable to hold the 2min timer
unsigned long time_since_prgm_start = 0;
unsigned long time_at_SG = 0;

// Maximum distance for the ultrasonic sensors (in cm)
const int maxDistance = 300;

//Moving average array
int movingAvg[] = {0,0,0,0,0};
int avgIndx = 0;
int getFrontDist();

// Button
const int buttonPins[] = {35, 37};
uint32_t previousMillis[WallFollower::noOfButtons];
uint8_t pressCount[WallFollower::noOfButtons];
uint8_t testCount[WallFollower::noOfButtons];

// Initialize the ultrasonic sensors
NewPing frontSensor(WallFollower::frontTrigPin, WallFollower::frontEchoPin, maxDistance);
NewPing leftSensor(WallFollower::leftTrigPin, WallFollower::leftEchoPin, maxDistance);
NewPing midSensor(WallFollower::midTrigPin, WallFollower::midEchoPin, 80);

// PID controller parameters
const float Kp = 0.7;
const float Ki = 0.01;
const float Kd = 0.2;
// const float Kp = 1.2;
// const float Ki = 0.00;
// const float Kd = 0.00;
float integral = 0;
float previousError = 0;
unsigned long previousTime = 0;

// Desired distance from the left wall (in cm)
const int desiredDistance = 27;
bool startGame = false;
char colors[] = {'Y', 'G', 'B', 'R'};
uint8_t color = 0;  // {1, Y}, {2, G}, {3, B}, {4, R}

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

    // Set up the LCD display
    lcd.begin(16, 2);
    lcd.print("Wall Follower");
    lcd.setCursor(0,1);
    lcd.print(colors[color]);
}

void debounce();
void CheckTime();
void StartTimer();

void WallFollower::loop() {

    // Function to determine value of variable SG
    time_since_prgm_start = millis();
    CheckTime();

    //get button presses
    debounce();

    if (startGame == false){
        lcd.setCursor(0,1);
        lcd.print(colors[color]);
    }

    if(startGame==true){

        // Read the distance from the ultrasonic sensors
        long frontDistance = frontSensor.ping_cm();
        long midDistance = midSensor.ping_cm();
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
        if (frontDistance > 30) {
            // Move forward and adjust the speed based on the PID output
            int baseSpeed = 80;
            int motor1Speed = baseSpeed + output;
            int motor2Speed = baseSpeed - output;

            // Constrain the motor speeds
            motor1Speed = constrain(motor1Speed, 0, 100);
            motor2Speed = constrain(motor2Speed, 0, 100);

            // Set the motor speeds and directions
            // Serial.println("Moving forward");
            digitalWrite(WallFollower::motor1In1, HIGH);
            digitalWrite(WallFollower::motor1In2, LOW);
            analogWrite(WallFollower::motor1Pwm, motor1Speed);
            digitalWrite(WallFollower::motor2In1, HIGH);
            digitalWrite(WallFollower::motor2In2, LOW);
            analogWrite(WallFollower::motor2Pwm, motor2Speed);

            // Update the LCD display
            lcd.setCursor(0,0);
            lcd.print("S: ");
            lcd.print("Moving Forward");
             lcd.setCursor(0,1);
            lcd.print("F:");
            lcd.print("    ");
            lcd.setCursor(2, 1);
            lcd.print(frontDistance);
            lcd.setCursor(7,1);
            lcd.print("M:");
            lcd.print("    ");
            lcd.setCursor(9, 1);
            lcd.print(midDistance);
            lcd.setCursor(13,1);
            lcd.print("L:");
            lcd.print("    ");
            lcd.setCursor(15, 1);
            lcd.print(leftDistance);

        } 
        else if(frontDistance <= 30 ) {
            // Update the LCD display
            lcd.setCursor(0,0);
            lcd.print("S: ");
            lcd.print("               ");
            lcd.setCursor(3,0);
            lcd.print("Turning");
            lcd.setCursor(0,1);
            lcd.print("F:");
            lcd.print("    ");
            lcd.setCursor(2, 1);
            lcd.print(frontDistance);
            lcd.setCursor(7,1);
            lcd.print("M:");
            lcd.print("    ");
            lcd.setCursor(9, 1);
            lcd.print(midDistance);
            lcd.setCursor(13,1);
            lcd.print("L:");
            lcd.print("    ");
            lcd.setCursor(15, 1);
            lcd.print(leftDistance);
            
            // Stop
            digitalWrite(WallFollower::motor1In1, LOW);
            digitalWrite(WallFollower::motor1In2, LOW);
            analogWrite(WallFollower::motor1Pwm, 0);
            digitalWrite(WallFollower::motor2In1, LOW);
            digitalWrite(WallFollower::motor2In2, LOW);
            analogWrite(WallFollower::motor2Pwm, 0);

            delay(500);

            //Turn
            while(frontDistance < 30 || (midDistance < 18 && midDistance > 0)){
                digitalWrite(WallFollower::motor1In1, HIGH);
                digitalWrite(WallFollower::motor1In2, LOW);
                analogWrite(WallFollower::motor1Pwm, 80);
                digitalWrite(WallFollower::motor2In1, LOW);
                digitalWrite(WallFollower::motor2In2, HIGH);
                analogWrite(WallFollower::motor2Pwm, 80);

                frontDistance = frontSensor.ping_cm();
                midDistance = midSensor.ping_cm();

                if (DEBUG) {
                    Serial.print("Turning   ");
                    Serial.print(frontDistance);
                    Serial.print("  ");
                    Serial.println(midDistance);
                }

                lcd.setCursor(0,0);
                lcd.print("S: ");
                lcd.print("               ");
                lcd.setCursor(3,0);
                lcd.print("Turning");
                lcd.setCursor(0,1);
                lcd.print("F:");
                lcd.print("    ");
                lcd.setCursor(2, 1);
                lcd.print(frontDistance);
                lcd.setCursor(7,1);
                lcd.print("M:");
                lcd.print("    ");
                lcd.setCursor(9, 1);
                lcd.print(midDistance);
                lcd.setCursor(13,1);
                lcd.print("L:");
                lcd.print("    ");
                lcd.setCursor(15, 1);
                lcd.print(leftDistance);
            }

            // Stop
            Serial.println("Stoping");
            digitalWrite(WallFollower::motor1In1, LOW);
            digitalWrite(WallFollower::motor1In2, LOW);
            analogWrite(WallFollower::motor1Pwm, 0);
            digitalWrite(WallFollower::motor2In1, LOW);
            digitalWrite(WallFollower::motor2In2, LOW);
            analogWrite(WallFollower::motor2Pwm, 0);

        }

        if (DEBUG == true) {
            Serial.print("FD: ");
            Serial.print(frontDistance);
            Serial.print(" MD: ");
            Serial.print(midDistance);
            Serial.print(" LD: ");
            Serial.print(leftDistance);
            Serial.print(" O: ");
            Serial.println(output);
        }    
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
            if (pressCount[0] == WallFollower::minButtonPress) {
                startGame = true;                             //Button has been debounced. Set SG to true
                StartTimer();
                pressCount[0] = 0;
            }
            if (pressCount[1] == WallFollower::minButtonPress) {  // Toggle the color
                color++;
                if(color == 4)
                    color = 0;
            }
        }
    }
}


void StartTimer() {
    /*
      Function to start our game timer
    */
    time_at_SG = millis(); // record curr time to use as reference zero
}


void CheckTime() {
    /*
      Function to determine value of variable SG
    */
    if((time_at_SG > 0) && ((time_since_prgm_start - time_at_SG) >= TIME_LIMIT)) { // exceeding 2 min limit
        startGame = false;
//        lcd.clear();
//        lcd.print("TIME UP!!!");
        Serial.println("TIME UP!!!");
    } else if((time_at_SG == 0) && (startGame == false)){ // timer not started yet and input SG is still 0
        startGame = false;
    }
}

// int getFrontDist(){
//   long frontDist = frontSensor.ping_cm();
//   movingAvg[avgIndx++] = frontDist;
//   if(avgIndx == 5) {avgIndx = 0;}
//   return (int((movingAvg[0] + movingAvg[1] + movingAvg[2]+ movingAvg[3] + movingAvg[4]) / 5));
// }