// Distributed control functions

#ifndef DIST_CONTROL_H
#define DIST_CONTROL_H

// System imports
#include <Arduino.h>


/*-------Variable declaration-------*/
// Input/Output pins
constexpr byte ledPin = 3;
constexpr byte ldrPin = 10;

/* Circuit parameters */
constexpr int Vcc = 5000;  // [mV]
constexpr int R1 = 10;     // [KOhm]

// LDR calibration
extern const float m[3];
extern const float b[3];
extern float k[3];

/*--------Function propotypes--------*/
float getLux(float measurement);


#endif // DIST_CONTROL_H
