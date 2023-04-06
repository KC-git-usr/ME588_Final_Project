#include <util/atomic.h>

// Number of motors
#define NMOTORS 2

// Assigning pin numbers
const int ENC_A_pin[] = {2,3};  // Encoder A
const int ENC_B_pin[] = {4,5};  // Encoder B
const int PWM_pin[] = {6,11};  // PWM pins
const int IN_1_pin[] = {8,9};  // In 1
const int IN_2_pin[] = {7,10};  // In 2

// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// Class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
    SimplePID() :
        kp(1),
        kd(0),
        ki(0),
        umax(255),
        eprev(0.0),
        eintegral(0.0) {}

    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
        kp = kpIn;
        kd = kdIn;
        ki = kiIn;
        umax = umaxIn;
    }

    // Function to compute the control signal
    void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
        // error
        int e = target - value;

        // derivative
        float dedt = (e-eprev)/(deltaT);
  
        // integral
        eintegral = eintegral + e*deltaT;
  
        // control signal
        float u = kp*e + kd*dedt + ki*eintegral;
  
        // motor power
        pwr = (int) fabs(u);
        if( pwr > umax ) {
          pwr = umax;
        }
  
        // motor direction
        dir = 1;
        if(u<0) {
          dir = -1;
        }
  
        // store previous error
        eprev = e;
        // Serial.print("Dir: ");
        // Serial.println(dir);
        // Serial.print("error: ");
        // Serial.print(e);
        // Serial.print(" ");
    }

};

// PID class object
SimplePID pid[NMOTORS];


void setup() {
  Serial.begin(9600);

  // Setup motor pins
  for(int k = 0; k < NMOTORS; k++){
    pinMode(ENC_A_pin[k], INPUT);
    pinMode(ENC_B_pin[k], INPUT);
    pinMode(PWM_pin[k], OUTPUT);
    pinMode(IN_1_pin[k], OUTPUT);
    pinMode(IN_2_pin[k], OUTPUT);
    // Set PID constants for motors
    pid[k].setParams(1.5, 0.15, 0.01, 200);
  }
  
  attachInterrupt(digitalPinToInterrupt(ENC_A_pin[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_pin[1]), readEncoder<1>, RISING);
  
  Serial.println("target pos");
}


void loop() {
  // Set target position
  int target[NMOTORS];
  // Straight motion, sinusoidal speed
  target[0] = 750*sin(prevT/1e6);
  target[1] = 750*sin(prevT/1e6);
  // Straight motion, ramp speedA
  target[0] = 500;
  target[1] = 500;
  // Right turn motion, ramp speed
  target[0] = 500;
  target[1] = -500;
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  
  // Loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // Evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // Signal the motor
    setMotor(dir,pwr,PWM_pin[k],IN_1_pin[k],IN_2_pin[k]);
  }

  for(int k = 0; k < NMOTORS; k++){
    // Serial.print("Target: ");
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
    Serial.print(-1000); // To freeze the lower limit
    Serial.print(" ");
    Serial.print(1000); // To freeze the upper limit
    Serial.print(" ");
    //target pos target pos 
  }
  Serial.println();
}


// Signal the motor
void setMotor(const int& dir,const int& PWM_val,const int& PWM_pin,const int& IN_1_pin,const int& IN_2_pin){
  analogWrite(PWM_pin, PWM_val);
  if(dir == 1) {
    digitalWrite(IN_1_pin, HIGH);
    digitalWrite(IN_2_pin, LOW);
  } else if(dir == -1) {
    digitalWrite(IN_1_pin, LOW);
    digitalWrite(IN_2_pin, HIGH);
  } else {
    digitalWrite(IN_1_pin, LOW);
    digitalWrite(IN_2_pin, LOW);
  }  
}


template <int j>
void readEncoder() {
  int b = digitalRead(ENC_B_pin[j]);
  if(b > 0) {
    posi[j]++;
  } else {
    posi[j]--;
  }
}
