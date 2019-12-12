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
extern const float b[maxNodes];
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
    float dMat[maxNodes][maxNodes];
    float dAvg[maxNodes];
    float rho;
    float y[maxNodes];
    float o_i = 0;
    float L_i = 0;
    float f_i = 0;
    bool firstPart;
    unsigned long last_time;
    const unsigned int timeout = 250;
    byte remainingIters;
    bool waiting;
    bool first;
    bool handshakes[maxNodes];
    byte nHand;
    bool boolMat[maxNodes][maxNodes];
    byte received;
    //byte allReceived;
    //bool nodesReceived[maxNodes];
    //bool nodesAck[maxNodes];
    //byte allAck;

  private:
    void ziCalc(float* zi);
    float dotProd(float x[maxNodes], float y[maxNodes]);
    float f_iCalc(float* d);
    bool findMinima();
    void calcMeanVector();
    void calcLagrangeMult();

  public:
    void init(byte nodeId, byte nNodes, float rho, byte c_i, float* new_y);
    void setLocalC(float c_i);
    float getLocalC();
    void setLocalO(float o_i);
    float getLocalO();
    void setLocalL(float L_i);
    float getLocalL();
    void getLocalDMean(float* dAvg);
    void getLocalD(float* d);
    void calcNewO();
    float calcExpectedLux();
    void receive_ack(byte senderId);
    void receive_duty_cycle(byte senderId, char code, float value);
    void send_duty_cycle();
    void startCounter();
    bool finished();
    void tellOthers();
    void tellStart();
    void rcvAns(byte senderId);
    void rcvStart(byte senderId);
    void calcOverallDC();
    void run();
};

/*--------Function propotypes--------*/
float getLux(int measurement);
void calcDisturbance(LedConsensus &ledConsensus, float measured);


#endif // DIST_CONTROL_H
