#include "LF_Sensor.h"


LF_Sensor::LF_Sensor(int l_threshold, int r_threshold) :
    l_threshold_(l_threshold),
    r_threshold_(r_threshold) {
    counter = 0;
    prev_lf_rdg_1 = 0;
    prev_lf_rdg_2 = 0;
};


void LF_Sensor::setup() {
    // nothing to add here right now
};

void LF_Sensor::loop() {

    int curr_lf_rdg_1 = analogRead(LF1);
    int curr_lf_rdg_2 = analogRead(LF2);

    // for debugging, remove later
    Serial.print("LF 1 value:  ");
    Serial.println(lineCross1);
    Serial.print("LF 2 value:  ");
    Serial.println(lineCross2);

    // CHANGE THESE THRESHOLD VALUES
    if (((prev_lf_rdg_1 > l_threshold_) && (prev_lf_rdg_2 > r_threshold_)) &&
    ((curr_lf_rdg_1 < l_threshold_) && (curr_lf_rdg_2 < r_threshold_)))  //tune threshold for LF
        counter++;

    if((counter % 2) == 0)
        Serial.println("line crossed");

    prev_lf_rdg_1 = curr_lf_rdg_1;
    prev_lf_rdg_2 = curr_lf_rdg_2;
};
