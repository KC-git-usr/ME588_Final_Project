//
// Created by Kumar Chakravarthy on 25-Apr-23.
//

#ifndef GIT_LF_SENSOR_H
#define GIT_LF_SENSOR_H

class LF_Sensor {

private:

    int l_threshold_;
    int r_threshold_;
    static int counter;
    static int prev_lf_rdg_1;
    static int prev_lf_rdg_2;

    // LF_Sensor pins
    const int LF1 = A0;
    const int LF2 = A1;

public:

    LF_Sensor(int l_threshold, int r_threshold);
    void setup();
    void loop();

};

#endif //GIT_LF_SENSOR_H
