// Declaration of the PID Class

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
    float kp, ki, kd, T, a;   // main params
    float k1, k2, k3, k4; // discrete params
    float yp, dp; // previous values
  public:
    float ip, ep;
    bool on;
    void init(float kp, float ki, float kd, float T, float a);
    float calc(float ref, float y, bool saturate);
};

#endif //PID_H
