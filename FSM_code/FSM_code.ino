/*
  ME 588 Final Project
*/

// Defining pin numbers
const int CS_pin = 2;
const int HS_pin = 3;
const int SG_pin = 4;
const int SP_pin = 5;
const int HP_pin = 6;
const int CD_pin = 7;

// Defining FSM input variables
int CS = 0; // Cube sensor
int HS = 0; // Home Square
int SG = 0; // Start Game

// Defining FSM output variables
int SP_state = 0; // Search pattern
int HP_state = 0; // Home pattern
int CD_state = 0; // Cube delivery

// Defining states for FSM
const int STATE_0 = 0; // START
const int STATE_1 = 1; // SEARCH
const int STATE_2 = 2; // DELIVER
const int STATE_3 = 3; // GO HOME
const int STATE_4 = 4; // STOP

// Initialiing the first state of the robot
int FSM_state = STATE_0;

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

  switch(FSM_state) {
    case  STATE_0:
      // perform action
      StayStationary();
      HoldCubes();
      // transistion logic
      if((CS == 1) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_3;
      } else if((CS == 0) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_1;        
      } else if(SG == 0) {
        FSM_state = STATE_4;        
      } else if((HS == 1) && (SG == 1)) {
        FSM_state = STATE_2;        
      }
    break;

    case  STATE_1:
      // perform action
      ActivateSearchPattern();
      HoldCubes();
      // transistion logic
      if((CS == 0) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_1;        
      } else if(SG == 0) {
        FSM_state = STATE_4;        
      } else if((CS == 1) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_3;        
      } else if((HS == 1) && (SG == 1)) {
        FSM_state = STATE_2;        
      }
    break;

    case  STATE_2:
      // perform action
      StayStationary();
      ReleaseCubes();
      // transistion logic
      if((CS == 0) && (SG == 1)) {
        FSM_state = STATE_1;        
      } else if(SG == 0) {
        FSM_state = STATE_4;        
      } else if((CS == 1) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_3;        
      } else if((CS == 1) && (HS == 1) && (SG == 1)) {
        FSM_state = STATE_2;        
      }
    break;

    case  STATE_3:
      // perform action
      ActivateHomePattern();
      HoldCubes();
      // transistion logic
      if(SG == 0) {
        FSM_state = STATE_4; 
      } else if((CS == 0) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_1;
      } else if((CS == 1) && (HS == 0) && (SG == 1)) {
        FSM_state = STATE_3;
      } else if((HS == 1) && (SG == 1)) {
        FSM_state = STATE_2;
      }
    break;

    case  STATE_4:
      // perform action
      StayStationary();
      HoldCubes();
      // transistion logic
      if(SG == 1) {
        FSM_state = STATE_1;
      } else if(SG == 0) {
        FSM_state = STATE_4;
      }
    break;    
  }

  PrintStatus();
  delay(1000);  
}


void PrintStatus() {
  // Simply print input data and FSM status
  Serial.println("----------------------------------");
  Serial.print("Current input (CS, HS, SG) : "); Serial.print(CS); Serial.print(HS); Serial.println(SG);
  Serial.print("FSM_state : "); Serial.println(FSM_state);
  Serial.println();
}


void GetInputData() {
  /*
    Read all 3 inputs at once
    Example : Enter 101 and press enter
    This will set CS = 1, HS = 0, SG = 1
    NOTE : enter a 3 digit num, with 1's and 0's only
  */
  if(Serial.available() > 0) {
    input_data = Serial.readString();
    input_data.trim();
    SG = input_data[2] == '1' ? 1 : 0;
    HS = input_data[1] == '1' ? 1 : 0;
    CS = input_data[0] == '1' ? 1 : 0;
    Serial.print("Received : "); Serial.println(input_data);
    // Serial.flush();
  }
}


void StayStationary() {
  /*
    Logic to keep the robot stationary
  */  
  SP_state = 0;
  HP_state = 0;
  // add logic here  
}


void HoldCubes() {
  /*
    Logic to hold the cubes
  */  
  CD_state = 0;
  // add logic here   
}


void ActivateSearchPattern() {
  /*
    Logic to start search pattern
  */  
  SP_state = 1;
  // add logic here   
}


void ActivateHomePattern() {
  /*
    Logic to start home drive
  */  
  HP_state = 1;
  // add logic here  
}


void ReleaseCubes() {
  /*
    Logic to release cubes
  */  
  CD_state = 1;
  // add logic here   
}
