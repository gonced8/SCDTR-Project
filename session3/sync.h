#ifndef SYNC_H
#define SYNC_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>
#include <stdlib.h>

// Custom imports
#include "can_comms.h"

/*---------Constants----------*/

/*---------Type definition----------*/
class Sync {
    bool on;
    byte nodeId;
    byte nNodes;
    byte current;
    bool handshake;
    unsigned long last_time;
    const unsigned int timeout = 500;
  public:
    void init(byte nodeId, byte nNodes);
    bool isOn();
    void ask_node();
    void receive_answer(can_frame frame);    
    void answer_node(byte senderId);
};
/*--------Function propotypes--------*/


#endif // SYNC_H
