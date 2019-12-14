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

/*-------Variable declaration-------*/


/*---------Type definition----------*/
class Calibration {
    byte nNodes, nodeId, nodeCounter;
    int measurements[maxNodes + 1];
    bool on = false;
    enum Action {turn = 't', measure = 'm'};
    Action action;
    bool waiting;
    bool handshakes[maxNodes];
    byte nHand;
    unsigned long last_time;
    const unsigned int timeout = 500;
    const float max_lux = 100;
  public:
    bool isOn();
    float getB(int measurement);
    void init(byte id, byte n);
    void run(LedConsensus &ledConsensus);
    void receive_answer(byte senderId, float value);
    void send_answer(byte senderId, float value);
};
/*--------Function propotypes--------*/


#endif // CALIBR_H
