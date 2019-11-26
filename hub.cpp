// Implementation of hub related functions


// System imports
#include <mcp2515.h>

// Custom imports
#include "hub.h"
#include "can_comms.h"


/*-------Variable definition--------*/
extern byte nodeId = 0; // initialize the variable to make it global
extern byte nNodes = 3; // TODO: should be automatically computed
extern bool hub = false;

/*--------Function definition--------*/
void getNodeId(){
	nodeId = digitalRead(ID0);
	nodeId |= digitalRead(ID1)<<1;
}

void hubFinder(){
	can_frame frame;
	byte counter = 0;
	Serial.print("Am I the hub?\n?");
	while(!Serial.available()){
		if(mcp2515.readMessage(&frame) == MCP2515::ERROR_OK || counter > 3)
		// We will wait 3 seconds to find a hub, else we will go default
			return;
    	counter ++;
    	delay(1000);
	}
	hub = true;
    write(3, -1); // Message to tell other nodes they are not the hub
}
