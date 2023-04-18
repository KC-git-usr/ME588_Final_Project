/* Simple button debounce for 4 buttons. Increments a count and sends the updated count to the serial monitor once per button press */
/* Tested on an Uno */
/* Connect simple push to make buttons between 0V and pin 2, 0V and pin 3, 0V and pin 4 and between 0V and pin 5 */

#define noOfButtons 3     //Exactly what it says; must be the same as the number of elements in buttonPins
#define bounceDelay 30    //Minimum delay before regarding a button as being pressed and debounced
#define minButtonPress 1  //Number of times the button has to be detected as pressed before the press is considered to be valid

const int buttonPins[] = {37, 39, 41};      // Input pins to use, connect buttons between these pins and 0V
uint32_t previousMillis[noOfButtons];       // Timers to time out bounce duration for each button
uint8_t pressCount[noOfButtons];            // Counts the number of times the button is detected as pressed, when this count reaches minButtonPress button is regared as debounced 
uint8_t testCount[noOfButtons];             //Test count, incremented once per button press

void setup() {
  uint8_t i;
  uint32_t baudrate = 115200;
  Serial.begin(baudrate);
  Serial.println("");
  Serial.print("Serial port connected: ");
  Serial.println(baudrate);
  for (i = 0; i < noOfButtons; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    Serial.print("Testcount ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(testCount[i]);
  }
}

void loop() {
  debounce();
  // delay(10);     //Your other code goes here instead of this delay. DO NOT leave this delay here, it's ONLY for demonstration.
}

void debounce() {
  uint8_t i;
  uint32_t currentMillis = millis();
  for (i = 0; i < noOfButtons; ++i) {
    if (digitalRead(buttonPins[i])) {             //Input is high, button not pressed or in the middle of bouncing and happens to be high
        previousMillis[i] = currentMillis;        //Set previousMillis to millis to reset timeout
        pressCount[i] = 0;                        //Set the number of times the button has been detected as pressed to 0
      } else {
      if (currentMillis - previousMillis[i] > bounceDelay) {
        previousMillis[i] = currentMillis;        //Set previousMillis to millis to reset timeout
        ++pressCount[i];
        if (pressCount[i] == minButtonPress) {
          doStuff(i);                             //Button has been debounced. Call function to do whatever you want done.
        }
      }
    }
  }
}

// Function to do whatever you want done once the button has been debounced.
// In this example it increments a counter and send the count to the serial monitor.
// Put your own functions here to do whatever you like.
void doStuff(uint8_t buttonNumber) {
  ++testCount[buttonNumber];
  Serial.print("Button ");
  Serial.print(buttonNumber);
  Serial.print(" testcount = ");
  Serial.println (testCount[buttonNumber]);
}