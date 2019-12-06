// Implementation of CAN bus related functions

#include "can_comms.h"

/*-------Variable definition--------*/
// CAN Bus
MCP2515 mcp2515(10); //SS pin 10
byte id_counter = 0;

volatile bool interrupt = false; //notification flag for ISR and loop()
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
volatile can_frame_stream cf_stream = can_frame_stream();

/*--------Function definition--------*/
MCP2515::ERROR write(byte to, byte priority,  uint8_t msg[data_bytes]) {
  uint32_t id = 0;
  id_counter = (++id_counter) % id_counter_max;
  id |= (to & 0x03);                // 2 bits for to id
  id |= (nodeId & 0x03) << 2;       // 2 bits for from id
  id |= (id_counter & 0x1F) << 4;   // 5 bits for counter
  id |= (priority & 0x07) << 9;     // 2 bits for priority

  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = data_bytes;

  for (int i = 0; i < data_bytes; i++)
    frame.data[i] = msg[i];

  MCP2515::ERROR err = mcp2515.sendMessage(&frame);

  if (err != MCP2515::ERROR_OK)
    Serial.println("\t\tError: MCP2515 TX Buffers Full");

  return err;
}

MCP2515::ERROR read(uint8_t msg[data_bytes]) {
  can_frame frame;
  MCP2515::ERROR err = mcp2515.readMessage(&frame);
  if (err == MCP2515::ERROR_OK) {
    for (int i = 0; i < data_bytes; i++)
      msg[i] = frame.data[i];
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

void decodeMessage(can_frame frame, byte &senderId, char *code, float *value) {
  senderId = (frame.can_id >> shiftId) & mask;
  code[0] = frame.data[0];
  code[1] = frame.data[1];
  memcpy(value, &(frame.data[2]), sizeof(float));
}

void encodeMessage(uint8_t msg[data_bytes], char code0, char code1, float value) {
  msg[0] = code0;
  msg[1] = code1;
  memcpy(&(msg[2]), &value, sizeof(float));
}

void barrier() {
  // Synchronization code, wait for other nodes
  can_frame frame;

  for (byte i = 1; i <= nNodes; i++) {
    if (i == nodeId) {
      // write message to bus
      write(0, 0, 0);
    }
    else {
      // read message from bus
      while (cf_stream.get(frame) != 1) {
        delay(1);
      }
    }
  }
}

void irqHandler() {
  can_frame frame;
  uint8_t irq = mcp2515.getInterrupts(); //read CANINTF
  if (irq & MCP2515::CANINTF_RX0IF) { //msg in receive buffer 0
    mcp2515.readMessage(MCP2515::RXB0, &frame); //also clears RX0IF
    if (!cf_stream.put(frame))
      arduino_overflow = true;
  }
  if (irq & MCP2515::CANINTF_RX1IF) { //msg in receive buffer 1
    mcp2515.readMessage(MCP2515::RXB1, &frame); //also clears RX1IF
    if (!cf_stream.put(frame))
      arduino_overflow = true;
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if ( (irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR) ) {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();
  interrupt = true; //notify loop()
}
