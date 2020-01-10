// System gains calibration related functions

#ifndef CALIBR_H
#define CALIBR_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>
#include <stdlib.h>

// Custom imports
#include "can_comms.h"
#include "dist_control.h"

#define maxNodes 3
#define max_lux 100

/*-------Variable declaration-------*/


/*---------Type definition----------*/
class Calibration {
    byte nNodes, nodeId, nodeCounter;
    int measurements[maxNodes + 1];
    bool on = false;
    byte state;
    bool first;
    bool boolArray[maxNodes];
    byte nBool;
    unsigned long last_time;
    const unsigned int timeout = 500;
  public:
    bool isOn();
    float getB(int measurement);
    void init(byte id, byte n);
    void run(LedConsensus &ledConsensus);
    void ask();
    void ans(byte senderId, char code);
    void rcv(byte senderId, char code);
    void resetBool();
};
/*--------Function propotypes--------*/


#endif // CALIBR_H
