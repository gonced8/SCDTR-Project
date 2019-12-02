#ifndef CONTROLLER_H
#define CONTROLLER_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>
#include <stdlib.h>

// Custom imports
#include "can_comms.h"

#define maxNodes 5

/*---------Constants----------*/
constexpr byte idShift = 2;
constexpr uint32_t maskId = 0b11;

/*---------Type definition----------*/
class Controller {
    bool on;
    byte nodeId;
    byte nNodes;
    byte current;
    byte currentIndex;
    byte currentNode;
    bool wait;
    unsigned long last_time;
    const int timeout = 500;
    float dutyCycles[maxNodes];
  public:
    void init(byte nodeId, byte nNodes);
    bool isOn();
    void ask_duty_cycle(can_frame frame);
    void receive_duty_cycle(can_frame frame);
    void answer_duty_cycle(can_frame frame);
    void incrementIndices();
};
/*--------Function propotypes--------*/


#endif // CONTROLLER_H
