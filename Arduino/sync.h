#ifndef SYNC_H
#define SYNC_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>
#include <stdlib.h>

// Custom imports
#include "can_comms.h"

/*---------Constants----------*/
#define maxNodes 3

/*---------Type definition----------*/
class Sync {
    bool on;
    byte nodeId;
    byte nNodes;
    bool first;
    bool handshakes[maxNodes];
    byte nHand;
    unsigned long last_time;
    const unsigned int timeout = 500;
  public:
    void init(byte nodeId, byte nNodes);
    bool isOn();
    void ask_node();
    void receive_answer(byte senderId);    
    void answer_node(byte senderId);
};
/*--------Function propotypes--------*/


#endif // SYNC_H
