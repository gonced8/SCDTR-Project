// Distributed control functions

#ifndef DIST_CONTROL_H
#define DIST_CONTROL_H

#define maxNodes 5

// System imports
#include <Arduino.h>

// Custom imports
#include "can_comms.h"

/*-------Constants declaration-------*/
// Input/Output pins
constexpr byte ledPin = 3;
constexpr byte ldrPin = A0;

// Circuit parameters
constexpr int Vcc = 5000;  // [mV]
constexpr byte R1 = 10;     // [KOhm]

/*-------Variable declaration-------*/
// LDR calibration
extern const float m[maxNodes];
extern const float b[maxNodes];
extern float k[maxNodes];

// Control related variables
extern float Li;

// Optimization

// Actuation
extern float measuredLux;
extern float dutyCycle;

/*---------Type definition----------*/
class LedConsensus {
    byte nodeId;
    byte nNodes;
    float d_diagonal[maxNodes];
    float d_column[maxNodes];
    float d[maxNodes];
    float d_av[maxNodes];
    float y[maxNodes];
    float n;
    float m;
    float ci;
    float o;
    float L;
    float tol = 0.001;
    float rho = 0.1;
    byte maxiter = 50;
    byte counter = 0;

  private:
    float dotProd(float x[maxNodes], float y[maxNodes]);
    bool check_feasibility(float* d);
    float evaluate_cost(float *local_d, float rho);

  public:
    void init(byte _nodeId, byte _nNodes, float _ci, float _oi, float _Li);
    void consensus_run();
    void consensus_iterate(float *d, float &cost, float rho);
    float measure_o();
};

/*--------Function propotypes--------*/
float getLux(int measurement);
void calcDisturbance(LedConsensus &ledConsensus, float measured);


#endif // DIST_CONTROL_H
