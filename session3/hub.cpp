// Implementation of hub related functions

#include "hub.h"

/*-------Variable definition--------*/
byte nodeId = 0; // initialize the variable to make it global
byte nNodes = 3; // TODO: should be automatically computed
bool hub = false;

/*--------Function definition--------*/
void getNodeId(){
	nodeId = digitalRead(ID0);
	nodeId |= digitalRead(ID1)<<1;
}

void hubFinder(){
	can_frame frame;
	byte counter = 0;
	Serial.print("Am I the hub?");
	while(!Serial.available()){
		if(mcp2515.readMessage(&frame) == MCP2515::ERROR_OK || counter > 3)
		// We will wait 3 seconds to find a hub, else we will go default
			return;
  	counter ++;
  	delay(1000);
	}
	hub = true;
  Serial.print("Hello!");
  write(0, 0, 0); // Message to tell other nodes they are not the hub
}