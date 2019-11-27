// CAN Bus communication protocol structures
// and func definitions

#ifndef CAN_COMMS_H
#define CAN_COMMS_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>

// Custom imports
#include "comm_codes.h"
#include "hub.h"


/*-------Variable declaration-------*/

// CAN Bus
extern MCP2515 mcp2515;
constexpr uint32_t mask = 0x00000003;       // To check the ID
extern uint32_t filt0;
extern uint32_t filt1;
constexpr uint32_t code_mask = 0x0000003F;  // To check the mask
constexpr byte masksize = 6;
byte id_counter = 0;
constexpr id_counter_max = 255;

/*---------Type definition----------*/
union my_can_msg {
  uint32_t value;
  uint8_t bytes[4];
};

/*--------Function propotypes--------*/
MCP2515::ERROR write(uint32_t id, uint32_t val);
MCP2515::ERROR read(unsigned long &c);
void setMasksFilters();
void decodeMessage(uint32_t, byte*, uint32_t);


#endif // CAN_COMMS_H
