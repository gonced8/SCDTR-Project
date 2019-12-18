// CAN Bus communication protocol structures
// and func definitions

#ifndef CAN_COMMS_H
#define CAN_COMMS_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>

// Custom imports
#include "can_frame_stream.h"
#include "comm_codes.h"

/*-------Constants declaration-------*/
#define data_bytes 4
#define mask 0b11               // To check the ID
#define code_mask 0b1111111     // To mask the code
#define shiftCode 4
#define shiftToId 2
#define shiftFromId 0

/*-------Variable declaration-------*/
// CAN Bus
extern MCP2515 mcp2515;
extern byte nodeId;

/*---------Type definition----------*/
extern volatile bool interrupt; //notification flag for ISR and loop()
extern volatile bool mcp2515_overflow;
extern volatile bool arduino_overflow;
extern volatile can_frame_stream cf_stream; //the object to use

union my_can_msg {
  float value;
  uint8_t bytes[data_bytes];
};

/*--------Function propotypes--------*/
MCP2515::ERROR write(byte to, byte code, float value);
MCP2515::ERROR read(uint8_t msg[data_bytes]);
void setMasksFilters();
void decodeMessage(can_frame frame, byte &senderId, char &code, float &value);
void irqHandler();

#endif // CAN_COMMS_H
