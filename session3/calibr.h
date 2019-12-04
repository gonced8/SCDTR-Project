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

/*-------Variable declaration-------*/


/*---------Type definition----------*/
class Calibration {
    byte nNodes, nodeId, nodeCounter;
    float *measurements = NULL;
    bool on = false;
    enum Action {turn = 't', measure = 'm'};
    Action action;
    bool waiting;
    byte nAck;
  public:
    bool isOn();
    void init(byte id, byte n);
    void run();
    void receiveAck();
};
/*--------Function propotypes--------*/


#endif // CALIBR_H
