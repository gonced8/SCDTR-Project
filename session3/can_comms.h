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
#include "hub.h"

/*-------Constants declaration-------*/
constexpr byte data_bytes = 4;
constexpr uint32_t mask = 0b11;       // To check the ID
constexpr uint32_t code_mask = 0b1111111;  // To mask the code
constexpr byte shiftCode = 4;
constexpr byte shiftToId = 2;
constexpr byte shiftFromId = 0;

/*-------Variable declaration-------*/
// CAN Bus
extern MCP2515 mcp2515;

/*---------Type definition----------*/
extern volatile bool interrupt; //notification flag for ISR and loop()
extern volatile bool mcp2515_overflow;
extern volatile bool arduino_overflow;
extern volatile can_frame_stream cf_stream; //the object to use

union my_can_msg {
  uint32_t value;
  uint8_t bytes[data_bytes];
};

/*--------Function propotypes--------*/
MCP2515::ERROR write(byte to, byte code, float value);
MCP2515::ERROR read(uint8_t msg[data_bytes]);
void setMasksFilters();
void decodeMessage(can_frame frame, byte &senderId, char &code, float &value);
void irqHandler();

#endif // CAN_COMMS_H
