// Hub identification and related functions

#ifndef HUB_H
#define HUB_H

// System imports
#include "Arduino.h"


/*-------Variable declaration-------*/
// Node identification
extern byte nodeId;
extern byte nNodes;
constexpr byte ID0 = 7;
constexpr byte ID1 = 8;
extern bool hub;

/*--------Function propotypes--------*/
void getNodeId();
void hubFinder();


#endif // HUB_H
