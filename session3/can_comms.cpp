// Implementation of CAN bus related functions

#include "can_comms.h"

/*-------Variable definition--------*/
// CAN Bus
MCP2515 mcp2515(10); //SS pin 10

volatile bool interrupt = false; //notification flag for ISR and loop()
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
volatile can_frame_stream cf_stream = can_frame_stream();

/*--------Function definition--------*/
MCP2515::ERROR write(byte to, byte code, float value) {
  uint32_t id = 0;
  Serial.print("Value inside write is "); Serial.println(value);
  id |= (code & code_mask) << shiftCode;    // 7 bits for counter
  id |= (to & mask) << shiftToId;           // 2 bits for to id
  id |= (nodeId & mask) << shiftFromId;     // 2 bits for from id

  union my_can_msg msg;
  msg.value = value;

  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = data_bytes;

  for (int i = 0; i < data_bytes; i++)
    frame.data[i] = msg.bytes[i];

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
  uint32_t filt0 = nodeId << shiftToId; // accepts messages with own ID
  uint32_t filt1 = 0;                   // accepts messages from everyone

  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask << shiftToId);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask << shiftToId);
  //filters related to RXB0
  mcp2515.setFilter(MCP2515::RXF0, 0, filt0);
  mcp2515.setFilter(MCP2515::RXF1, 0, filt1);
  //filters related to RXB1
  mcp2515.setFilter(MCP2515::RXF2, 0, filt0);
  mcp2515.setFilter(MCP2515::RXF3, 0, filt1);
}

void decodeMessage(can_frame frame, byte &senderId, char &code, float &value) {
  union my_can_msg msg;

  senderId = (frame.can_id >> shiftFromId) & mask;
  code = (frame.can_id >> shiftCode) & code_mask;

  for (int i = 0; i < data_bytes; i++)
    msg.bytes[i] = frame.data[i];

  value = msg.value;
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
