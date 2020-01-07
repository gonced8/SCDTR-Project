// Distributed control functions

#ifndef DIST_CONTROL_H
#define DIST_CONTROL_H

#define maxNodes 3

// System imports
#include <Arduino.h>
#include <math.h>

// Custom imports
#include "can_comms.h"
#include "PID.h"

/*-------Constants declaration-------*/
// Input/Output pins
#define ledPin 3
#define ldrPin A0

// Circuit parameters
#define Vcc 5000      // [mV]
#define R1 10         // [KOhm]

#define maxIters 20
#define threshold 5

/*-------Variable declaration-------*/
// LDR calibration
extern const float m[maxNodes];
extern float b[maxNodes];
extern float k[maxNodes];

// Control related variables
extern float luxRefUnocc;
extern float luxRefOcc;
extern bool deskOccupancy;

// Optimization
extern const float infinity;

// Actuation
extern float measuredLux;

/*---------Type definition----------*/
class LedConsensus {
    byte nodeId;
    byte nNodes;
    float dNode[maxNodes];
    float dAvg[maxNodes];
    float rho;
    float y[maxNodes];
    unsigned long last_time;
    const unsigned int timeout = 20;
    byte remainingIters;
    bool first;
    bool handshakes[maxNodes];
    byte nHand;
    float tol = 0.001;
    byte state;
    float dColumn[maxNodes];
    bool boolArray[maxNodes];
    byte nBool;
    bool changedLuxRef = false;
    bool changedCost = false;

  public:
    float c_i;
    float c[maxNodes];
    float dNodeOverall[maxNodes];
    float o_i = 0;
    float L_i = 0;


  private:
    void ziCalc(float* zi);
    float dotProd(float x[maxNodes], float y[maxNodes]);
    bool f_iCalc(float* d);
    float evaluateCost(float* d);
    void findMinima();
    void calcMeanVector();
    void calcLagrangeMult();
    void resetBool();

  public:
    void init(byte nodeId, byte nNodes, float rho, byte c_i);
    byte getState();
    bool detectChanges();
    void setLocalC(float c_i);
    void setLocalO(float o_i);
    void setLocalL(float L_i);
    float calcExpectedLux();
    void tellOthers();
    void tellStart();
    void rcvAns(byte senderId);
    void rcvStart(byte senderId);
    void calcOverallDC();
    void resetConsensus();

    void run();

    void ask();
    void ans(byte senderId, char code, float value);
    void rcv(byte senderId, char code, float value);
};

/*--------Function propotypes--------*/
float getLux(int measurement);


#endif // DIST_CONTROL_H
