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
constexpr byte data_bytes = 6;
constexpr uint32_t mask = 0x00000003;       // To check the ID
constexpr uint32_t code_mask = 0x0000003F;  // To check the mask
constexpr byte masksize = 6;
constexpr byte id_counter_max = 32;
constexpr byte shiftId = 2;

/*-------Variable declaration-------*/
// CAN Bus
extern MCP2515 mcp2515;
extern byte id_counter;

/*---------Type definition----------*/
extern volatile bool interrupt; //notification flag for ISR and loop()
extern volatile bool mcp2515_overflow;
extern volatile bool arduino_overflow;
extern volatile can_frame_stream cf_stream; //the object to use

/*--------Function propotypes--------*/
MCP2515::ERROR write(byte to, byte priority,  uint8_t msg[data_bytes]);
MCP2515::ERROR read(uint8_t msg[data_bytes]);
void setMasksFilters();
void decodeMessage(can_frame frame, byte &senderId, char *code, float &value);
void encodeMessage(uint8_t msg[data_bytes], char code0, char code1, float value);
void barrier();
void irqHandler();

#endif // CAN_COMMS_H
