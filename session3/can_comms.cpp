// Implementation of CAN bus related functions

#include "can_comms.h"

/*-------Variable definition--------*/
// CAN Bus
MCP2515 mcp2515(10); //SS pin 10
byte id_counter = 0;

/*--------Function definition--------*/
MCP2515::ERROR write(byte to, byte priority,  uint32_t val) {
  	uint32_t id = 0;
    id_counter++;
    if(id_counter >= id_counter_max)
      id_counter = 0;
    id |= (to & 0x03);                // 2 bits for to id
    id |= (nodeId & 0x03) << 2;       // 2 bits for from id
    id |= (id_counter & 0x1F) << 4;   // 5 bits for counter
    id |= (priority & 0x07) << 9;     // 2 bits for priority
    
  	can_frame frame;
  	frame.can_id = id;
  	frame.can_dlc = data_bytes;
  	my_can_msg msg;
  	msg.value = val;
  	for(int i = 0; i < data_bytes; i++)
    	frame.data[i] = msg.bytes[i];
  	return mcp2515.sendMessage(&frame);
}

MCP2515::ERROR read(unsigned long &c) {
    can_frame frame;
    my_can_msg msg;
    MCP2515::ERROR err = mcp2515.readMessage(&frame);
    if(err == MCP2515::ERROR_OK) {
    	for(int i = 0; i < data_bytes; i++)
        	msg.bytes[i] = frame.data[i];
     	c = msg.value;
  	}
  	return err;
}

void setMasksFilters() {
  uint32_t filt0 = nodeId;    // accepts messages with own ID
  uint32_t filt1 = 0x00;      // accepts messages from everyone

  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  //filters related to RXB0
  mcp2515.setFilter(MCP2515::RXF0, 0, filt0);
  mcp2515.setFilter(MCP2515::RXF1, 0, filt1);
  //filters related to RXB1
  mcp2515.setFilter(MCP2515::RXF2, 0, filt0);
  mcp2515.setFilter(MCP2515::RXF3, 0, filt1);
}

void decodeMessage(uint32_t message, byte &code, uint32_t &data) {
  code = code_mask & message;
  data = (code_mask & message) << masksize;
}

void barrier() {
  // Synchronization code, wait for other nodes
  unsigned long c;
  
  for(byte i=1; i<=nNodes; i++) {
    if (i == nodeId) {
      // write message to bus
      write(0, 0, 0);
    }
    else{
      // read message from bus
      while(read(c) != MCP2515::ERROR_OK) {
        delay(1);
      }
    }
  }
}
