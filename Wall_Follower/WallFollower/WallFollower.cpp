#include "WallFollower.h"

#define DEBUG true

// Defining time limit
const unsigned long TIME_LIMIT = 90000; // [ms]

// Variable to hold the 2min timer
unsigned long time_since_prgm_start = 0;
unsigned long time_at_SG = 0;

// Maximum distance for the ultrasonic sensors (in cm)
const int maxDistance = 300;

//
const int frontWallDetectDistance = 35;

//LF stuff
int l_threshold_ = 500;
int r_threshold_ = 500;
static int counter = 0;
static int prev_lf_rdg_1 = 0;
static int prev_lf_rdg_2 = 0;

// Button
const int buttonPins[] = {35, 37};
uint32_t previousMillis[WallFollower::noOfButtons];
uint8_t pressCount[WallFollower::noOfButtons];
uint8_t testCount[WallFollower::noOfButtons];

// Initialize the ultrasonic sensors
NewPing frontSensor(WallFollower::frontTrigPin, WallFollower::frontEchoPin, maxDistance);
NewPing leftSensor(WallFollower::leftTrigPin, WallFollower::leftEchoPin, 50);
NewPing midSensor(WallFollower::midTrigPin, WallFollower::midEchoPin, 40);

// PID controller parameters
const float Kp = 0.8;
const float Ki = 0.02;
const float Kd = 0.08;
// const float Kp = 1.2;
// const float Ki = 0.00;
// const float Kd = 0.00;
float integral = 0;
float previousError = 0;
unsigned long previousTime = 0;

int baseSpeed = 75;

// Desired distance from the left wall (in cm)
const int desiredDistance = 18;
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
    pinMode(WallFollower::irsensor, INPUT);

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
void getLF_Rdg();

long frontDistance = 0;
long midDistance = 0;
long leftDistance = 0;
int irsensor_reading = 0;

void WallFollower::loop() {

    // Function to determine value of variable SG
    time_since_prgm_start = millis();
    CheckTime();

    if (startGame == false){
        //get button presses
        debounce();
        lcd.setCursor(0,1);
        lcd.print(colors[color]);
    }

    

    if(startGame==true){
        // get LF reading
        // getLF_Rdg();
        // if(counter == 3) {
        //   startGame = false;
        //   if(DEBUG) {
        //       Serial.println("Reached home base");
        //   }
        // }

        // Read the distance from the ultrasonic sensors
        frontDistance = frontSensor.ping_cm();
        midDistance = midSensor.ping_cm();
        leftDistance = leftSensor.ping_cm();
        irsensor_reading = digitalRead(WallFollower::irsensor);
        
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

        if((frontDistance <= frontWallDetectDistance) || (midDistance < 10 && midDistance > 0)) {
                        
            // Stop
            digitalWrite(WallFollower::motor1In1, LOW);
            digitalWrite(WallFollower::motor1In2, LOW);
            analogWrite(WallFollower::motor1Pwm, 0);
            digitalWrite(WallFollower::motor2In1, LOW);
            digitalWrite(WallFollower::motor2In2, LOW);
            analogWrite(WallFollower::motor2Pwm, 0);

            delay(10);

            digitalWrite(WallFollower::motor1In1, HIGH);
            digitalWrite(WallFollower::motor1In2, LOW);
            analogWrite(WallFollower::motor1Pwm, 75);
            digitalWrite(WallFollower::motor2In1, LOW);
            digitalWrite(WallFollower::motor2In2, HIGH);
            analogWrite(WallFollower::motor2Pwm, 75);

            delay(200);

            // Stop
            // Serial.println("Stoping");
            digitalWrite(WallFollower::motor1In1, LOW);
            digitalWrite(WallFollower::motor1In2, LOW);
            analogWrite(WallFollower::motor1Pwm, 0);
            digitalWrite(WallFollower::motor2In1, LOW);
            digitalWrite(WallFollower::motor2In2, LOW);
            analogWrite(WallFollower::motor2Pwm, 0);

            delay(10);

            // Update the LCD display
            lcd.setCursor(0,0);
            lcd.print("S: ");
            lcd.print("      ");
            lcd.setCursor(2,0);
            lcd.print("Turn");
            lcd.setCursor(0,1);
            lcd.print(counter);
            // lcd.setCursor(0,1);
            // lcd.print("F:");
            // lcd.print("    ");
            // lcd.setCursor(2, 1);
            // lcd.print(frontDistance);
            // lcd.setCursor(7,1);
            // lcd.print("M:");
            // lcd.print("    ");
            // lcd.setCursor(9, 1);
            // lcd.print(midDistance);
            // lcd.setCursor(13,1);
            // lcd.print("L:");
            // lcd.print("    ");
            // lcd.setCursor(15, 1);
            // lcd.print(leftDistance);

        } else { // Control the motors based on the PID output
            // if ((frontDistance > 30) && (midDistance == 0)) {
            // Move forward and adjust the speed based on the PID output
            int motor1Speed = baseSpeed + output;
            int motor2Speed = baseSpeed - output;

            // Constrain the motor speeds
            motor1Speed = constrain(motor1Speed, 0, 120);
            motor2Speed = constrain(motor2Speed, 0, 120);

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
            lcd.print("      ");
            lcd.setCursor(2,0);
            lcd.print("Move");
            lcd.setCursor(0,1);
            lcd.print(counter);
            // lcd.setCursor(0,1);
            // lcd.print("F:");
            // lcd.print("    ");
            // lcd.setCursor(2, 1);
            // lcd.print(frontDistance);
            // lcd.setCursor(6,1);
            // lcd.print("M:");
            // lcd.print("    ");
            // lcd.setCursor(8, 1);
            // lcd.print(midDistance);
            // lcd.setCursor(11,1);
            // lcd.print("L:");
            // lcd.print("    ");
            // lcd.setCursor(13, 1);
            // lcd.print(leftDistance);

        }  

        if (DEBUG) {
            Serial.print("FD: ");
            Serial.print(frontDistance);
            Serial.print(" MD: ");
            Serial.print(midDistance);
            Serial.print(" LD: ");
            Serial.print(leftDistance);
            Serial.print(" O: ");
            Serial.print(output);
            Serial.print("  IR: ");
            Serial.println(irsensor_reading);
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
        Serial.print("TIME UP!!!  "); Serial.println(startGame);
    } else if((time_at_SG == 0) && (startGame == false)){ // timer not started yet and input SG is still 0
        startGame = false;
    }
}

void getLF_Rdg() {

    // int curr_lf_rdg_1 = analogRead(WallFollower::LF1);
    int curr_lf_rdg_2 = analogRead(WallFollower::LF2);

    if(DEBUG) {
        // Serial.print("LF 1 value:  ");
        // Serial.print(curr_lf_rdg_1);
        Serial.print("  LF 2 value:  ");
        Serial.println(curr_lf_rdg_2);
    }

    // CHANGE THESE THRESHOLD VALUES
    // if (((prev_lf_rdg_1 > l_threshold_) && (prev_lf_rdg_2 > r_threshold_)) &&
    // ((curr_lf_rdg_1 < l_threshold_) && (curr_lf_rdg_2 < r_threshold_)))  //tune threshold for LF
    //     counter++;

    if ((prev_lf_rdg_2 < r_threshold_) && (curr_lf_rdg_2 > r_threshold_))  //tune threshold for LF
        counter++;
        

    // prev_lf_rdg_1 = curr_lf_rdg_1;
    prev_lf_rdg_2 = curr_lf_rdg_2;
};
