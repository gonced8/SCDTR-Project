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

/*-------Constants declaration-------*/
constexpr byte data_bytes = 4;
constexpr uint32_t mask = 0x00000003;       // To check the ID
constexpr uint32_t code_mask = 0x0000003F;  // To check the mask
constexpr byte masksize = 6;
constexpr byte id_counter_max = 32;

/*-------Variable declaration-------*/
// CAN Bus
extern MCP2515 mcp2515;
extern byte id_counter;

/*---------Type definition----------*/
union my_can_msg {
  uint32_t value;
  uint8_t bytes[data_bytes];
};

class can_frame_stream {
  static constexpr int buffsize = 10; //space for 10 can_messages - increase if needed
  can_frame cf_buffer[buffsize];
  int read_index; //where to read next message
  int write_index; //where to write next message
  bool write_lock; //buffer full
public:
  can_frame_stream() : read_index{0}, write_index{0}, write_lock{false} {};
  int put(can_frame &frame) {
    if(write_lock) return 0; //buffer full
    cf_buffer[write_index] = frame;
    write_index=(++write_index)%buffsize;
    if(write_index == read_index) write_lock = true; //cannot write more
    return 1;
  }
  int get(can_frame &frame) {
    if(!write_lock && (read_index==write_index) ) return 0; //empty buffer
    if(write_lock && (read_index==write_index) ) write_lock = false; //release lock
    frame = cf_buffer[read_index];
    read_index = (++read_index)%buffsize;
    return 1;
  }
} extern volatile cf_stream; //the object to use
extern volatile bool interrupt; //notification flag for ISR and loop()
extern volatile bool mcp2515_overflow;
extern volatile bool arduino_overflow;

/*--------Function propotypes--------*/
MCP2515::ERROR write(byte to, byte priority,  uint32_t val);
MCP2515::ERROR read(unsigned long &c);
void setMasksFilters();
void decodeMessage(uint32_t, byte*, uint32_t);
void barrier();
void irqHandler();


#endif // CAN_COMMS_H
