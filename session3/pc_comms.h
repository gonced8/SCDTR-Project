// Hub identification and related functions

#ifndef HUB_H
#define HUB_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>

// Custom imports
#include "can_comms.h"

/*-------Constant declaration-------*/
constexpr byte ID0 = 7;
constexpr byte ID1 = 8;

/*-------Variable declaration-------*/
// Node identification
extern byte nodeId;
extern byte nNodes;
extern bool hub;

/*--------Function propotypes--------*/
void getNodeId();
void hubFinder();


#endif // HUB_H
