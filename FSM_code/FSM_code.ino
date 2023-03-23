/*
  ME 588 Final Project
  Team 10

  Reference: https://docs.google.com/presentation/d/1SksHOphqhfJFn6xp7ug-bYm4VV44OZiPOhLUAs5DCw4/edit?usp=sharing
  This program will implement our core FSM logic and
  will be the base program that makes other custom
  functions calls

  TO DO:
  Verify data type for pins

  Author:
  Kumar Ramesh (ramesh64@purdue.edu)
  22nd Mar, 2023
 */

// Defining pin numbers
const char SG_pin = 2;  // Start Game pin
const char ES_pin = 3;  // Emergency Stop pin
const char LD_pin = 4;  // Line Detected pin
const char EC_pin = 5;  // Encoder Count pin
const char US_pin = 6;  // Ultra-Sonic sensor pin

// Defining FSM input variables
boolean SG = 0; // Start Game
boolean ES = 0; // Emergency Stop
boolean LD = 0; // Line Detected
boolean EC = 0; // Encoder Count
boolean US = 0; // Ultra-Sonic sensor

// Defining FSM output variables
boolean FL = 0; // Flaps
boolean MF = 0; // Move Forward
boolean RT = 0; // Right Turn

// Defining states for FSM
const byte STATE_0 = 0; // STOP
const byte STATE_1 = 1; // FORWARD
const byte STATE_2 = 2; // RIGHT TURN
const byte STATE_3 = 3; // EMERGENCY STOP

// Defining time limit
const unsigned long TIME_LIMIT = 120000; // [ms]

// Initializing the first state of the robot
int FSM_state = STATE_0;

// Variable to hold the 2min timer
unsigned long time_since_prgm_start = 0;
unsigned long time_at_SG = 0;

// Variable to read virtual input from keyboard
String input_data;


void setup() {
  Serial.begin(9600);

  while(!Serial)
    delay(10);

  Serial.println("ME 588 Project FSM virtual testing"); 
  Serial.println();
   
  PrintStatus(); 
}


void loop() {
  
  // Get virtual input from user
  GetInputData();

  // Function to determine value of variable SG
  time_since_prgm_start = millis();
  CheckTime();

  switch(FSM_state) {
    case  STATE_0:
      // perform action
      StayStationary();
      RetractFlaps();
      // transition logic
      if(SG == 0) {
          FSM_state = STATE_0;
      } else if(SG == 1){
          StartTimer();
          FSM_state = STATE_1;
      }
    break;

    case  STATE_1:
      // perform action
      DeployFlaps();
      MoveForward();
      // transition logic
      if((SG == 1) && (ES ==0) && (EC == 0) && (US == 0)) {
        FSM_state = STATE_1;
      } else if((SG == 1) && (ES == 0) && (EC == 1) && (US == 1)) {
        StayStationary();  // Stop moving before turning right
        FSM_state = STATE_2;
      } else if(ES == 1) {
        FSM_state = STATE_3;
      } else if(((SG == 1) && (LD == 1)) || (SG == 0)) {
        FSM_state = STATE_0;
      }
    break;

    case  STATE_2:
      // perform action
      DeployFlaps();
      TurnRight();
      // transition logic
      if((SG == 1) && (ES == 0) && (EC == 0) && (US == 1)) {
        FSM_state = STATE_2;
      } else if(ES == 1) {
        FSM_state = STATE_3;
      } else if((SG == 1) && (ES == 0) && (EC == 1)) {
          StayStationary();
          FSM_state = STATE_1;
      }
    break;

    case  STATE_3:
      // perform action
      StayStationary();
      // transition logic
      if((SG == 1) && (ES == 0)) {
        FSM_state = STATE_1;
      } else if(ES == 1) {
        FSM_state = STATE_3;
      } else if((SG == 0) && (ES == 0)) {
          FSM_state = STATE_0;
      }
    break;    
  }

  PrintStatus();
  delay(1000);  
}


void PrintStatus() {
  // Simply print input data and FSM status
  Serial.println("----------------------------------");
  Serial.print("Current input (SG, ES, LD, EC, US) : "); Serial.print(SG); Serial.print(ES); Serial.print(LD);
  Serial.print(EC); Serial.println(US);
  Serial.print("FSM_state : "); Serial.println(FSM_state);
  Serial.print("Time : ");
  if(SG)
    Serial.println(time_since_prgm_start - time_at_SG);
  else
    Serial.println(0);
  Serial.println();
}


void GetInputData() {
  /*
    Read all 5 inputs at once
    Example : Enter 10101 and press enter
    This will set SG = 1, ES = 0, LD = 1, EC = 0, US = 1
    NOTE : enter a 5 digit num, with 1's and 0's only
  */
  if(Serial.available() > 0) {
    input_data = Serial.readString();
    input_data.trim();
    SG = input_data[0] == '1' ? 1 : 0;
    ES = input_data[1] == '1' ? 1 : 0;
    LD = input_data[2] == '1' ? 1 : 0;
    EC = input_data[3] == '1' ? 1 : 0;
    US = input_data[4] == '1' ? 1 : 0;
    Serial.print("Received : "); Serial.println(input_data);
    // Serial.flush();
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
        SG = 0;
        Serial.println("2 min UP!");
    } else if((time_at_SG == 0) && (SG == 0)){ // timer not started yet and input SG is still 0
        SG = 0;
    } else { // game started
        SG = 1;
    }
}


void StayStationary() {
  /*
    Logic to keep the robot stationary
  */  
  if(MF || RT) { // robot not stationary
      Serial.println("Stopping robot motion");
      // add logic here
      MF = 0;
      RT = 0;
  } else {
      Serial.println("Robot is already stationary");
  }
}


void MoveForward() {
    /*
      Logic to keep the robot moving in a straight line
    */
    if(MF) { // robot moving forward
        Serial.println("Robot is already moving forward");
    } else {
        Serial.println("Making robot move forward");
        // add logic here
        MF = 1;
    }
}


void TurnRight() {
    /*
      Logic to make the robot take a 90 deg right turn
    */
    if(RT) { // robot turning right
        Serial.println("Robot is already turning right");
    } else {
        Serial.println("Making robot turn right");
        // add logic here
        RT = 1;
    }
}


void DeployFlaps() {
  /*
    Logic to deploy flaps
  */  
  if(FL) {  // flaps deployed
      Serial.println("Flaps are already deployed");
  } else {
      Serial.println("Deploying flaps");
      // add logic here
      FL = 1;
  }
}


void RetractFlaps() {
  /*
    Logic to retract flaps
  */
  if(!FL) {  // flaps retracted
    Serial.println("Flaps are already retracted");
  } else {
    Serial.println("Retracting flaps");
    // add logic here
    FL = 0;
  }
}
