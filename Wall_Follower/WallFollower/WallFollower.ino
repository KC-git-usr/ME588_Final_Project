#include "WallFollower.h"

WallFollower wallFollower;

void setup() {
  Serial.begin(9600);
  wallFollower.setup();
}

void loop() {
  wallFollower.loop();
}
