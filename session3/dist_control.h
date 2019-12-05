// Distributed control functions

#ifndef DIST_CONTROL_H
#define DIST_CONTROL_H

#define maxNodes 5

// System imports
#include <Arduino.h>

// Custom imports
#include "hub.h"
#include "can_comms.h"

/*-------Constants declaration-------*/
// Input/Output pins
constexpr byte ledPin = 3;
constexpr byte ldrPin = A0;

// Circuit parameters
constexpr int Vcc = 5000;  // [mV]
constexpr int R1 = 10;     // [KOhm]

// Controller
constexpr byte idShift = 2;
constexpr uint32_t maskId = 0b11;

/*-------Variable declaration-------*/
// LDR calibration
extern const float m[maxNodes];
extern const float b[maxNodes];
extern float k[maxNodes];

// Control related variables
extern float luxRefUnocc;
extern float luxRefOcc;
extern bool deskOccupancy;

// Optimization
extern const float infinity;

/*---------Type definition----------*/
class LedConsensus {
  
    byte nodeId;
    byte nNodes;
    float c[maxNodes] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float c_i;
    float dNode[maxNodes] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float dNodep[maxNodes] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float dMat[maxNodes][maxNodes];
    float dAvg[maxNodes] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float rho;
    float y[maxNodes];
    float o_i = 0;
    float L_i = 0;
    float f_i = 0;
    bool firstPart;
    byte received;
    unsigned long last_time;
    const unsigned int timeout = 250;
    
  private:
    void ziCalc(float* zi);
    float dotProd(float x[maxNodes], float y[maxNodes]);
    float f_iCalc(float* d);
    
  public:
    void init(byte nodeId, byte nNodes, float rho, byte c_i, float* new_y);
    void setLocalC(float c_i);
    float getLocalC();
    void setLocalO(float o_i);
    float getLocalO();
    void setLocalL(float L_i);
    float getLocalL();
    void getLocalDMean(float* dAvg);
    float getLocalCost(float* d);
    void getLocalD(float* d);
    void calcNewO();
    bool findMinima();
    void send_duty_cycle();
    void receive_duty_cycle(can_frame frame);
    void calcMeanVector();
    void calcLagrangeMult();
    void run();
};

/*--------Function propotypes--------*/
float getLux(int measurement);

#endif // DIST_CONTROL_H
