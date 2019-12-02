// Distributed control functions

#ifndef DIST_CONTROL_H
#define DIST_CONTROL_H

// System imports
#include <Arduino.h>

// Custom imports
#include "hub.h"

/*-------Constants declaration-------*/
// Input/Output pins
constexpr byte ledPin = 3;
constexpr byte ldrPin = 10;

// Maximum illuminance value
constexpr float max_lux = 100;

// Circuit parameters
constexpr int Vcc = 5000;  // [mV]
constexpr int R1 = 10;     // [KOhm]

// Optimization
extern const float infinity = 1.0 / 0.0;

/*-------Variable declaration-------*/
// LDR calibration
extern const float m[3];
extern const float b[3];
extern float k[3];

// Optimization
extern float rho;

/*--------Function propotypes--------*/
float getLux(float measurement);
void costCalc();
float f_iCalc(float d[3]);
float d_localavgCalc(float d[3]);
float lanmultiplierCalc(float d[3]);
float distanceCalc(float d[3]);
float lagrangeCalc(float f_i, float d[3]);
void ziCalc(float* zi);
float dotProd(float x[3], float y[3]);
bool findminima();


#endif // DIST_CONTROL_H
