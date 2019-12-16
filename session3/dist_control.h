// Distributed control functions

#ifndef DIST_CONTROL_H
#define DIST_CONTROL_H

#define maxNodes 3

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

constexpr byte maxIters = 10;
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
extern float dutyCycle;

/*---------Type definition----------*/
class LedConsensus {
    byte nodeId;
    byte nNodes;
    float c[maxNodes];
    float c_i;
    float dNode[maxNodes];
    float dNodeOverall[maxNodes];
    float dAvg[maxNodes];
    float rho;
    float y[maxNodes];
    float o_i = 0;
    float L_i = 0;
    float f_i = 0;
    float cost;
    bool firstPart;
    unsigned long last_time;
    const unsigned int timeout = 250;
    byte remainingIters;
    bool waiting;
    bool first;
    bool handshakes[maxNodes];
    byte nHand;
    float tol = 0.01;
    float maxActuation;

    byte state;
    float dColumn[maxNodes];
    bool boolArray[maxNodes];
    byte nBool;

  private:
    void ziCalc(float* zi);
    float dotProd(float x[maxNodes], float y[maxNodes]);
    bool f_iCalc(float* d);
    float evaluateCost(float* d);
    bool findMinima();
    void calcMeanVector();
    void calcLagrangeMult();
    void resetBool();

  public:
    void init(byte nodeId, byte nNodes, float rho, byte c_i, float* new_y);
    void setLocalC(float c_i);
    float getLocalC();
    void setLocalO(float o_i);
    float getLocalO();
    void setLocalL(float L_i);
    float getLocalL();
    void getLocalDMean(float* dAvg);
    float getLocalD();
    float getMeasuredLux();
    void calcNewO();
    float calcExpectedLux();
    void startCounter();
    void tellOthers();
    void tellStart();
    void rcvAns(byte senderId);
    void rcvStart(byte senderId);
    void calcOverallDC();
    void checkConsensusError();
    void setMaxActuation(float calibration_input);

    bool finished();
    
    void run();

    void ask();
    void ans(byte senderId, char code);
    void rcv(byte senderId, char code, float value);
};

/*--------Function propotypes--------*/
float getLux(int measurement);
void calcDisturbance(LedConsensus &ledConsensus, float measured);


#endif // DIST_CONTROL_H
