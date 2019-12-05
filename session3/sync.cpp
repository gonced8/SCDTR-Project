#include "sync.h"

void Sync::init(byte _nodeId, byte _nNodes) {
  on = true;
  nodeId = _nodeId;
  nNodes = _nNodes;
  current = 1;
  if (current == nodeId)
    current++;
  handshake = true;
  last_time = millis();
}

bool Sync::isOn() {
  return on;
}

void Sync::ask_node() {
  unsigned long curr_time = millis();

  if (!handshake & (curr_time - last_time < timeout)) {
    return;
  }

  uint8_t msg[data_bytes];
  encodeMessage(msg, sync_ask[0], sync_ask[1], 0);
  handshake = false;
  write(current, 0, msg);
  last_time = millis();

  Serial.print("Sync: Asked node "); Serial.println(current);
}

void Sync::receive_answer(can_frame frame) {
  byte senderId = (frame.can_id >> shiftId) & idMask;

  Serial.print("Sync: Current "); Serial.print(current);
  Serial.print(". Received from node "); Serial.println(senderId);

  if (senderId == current) {
    handshake = true;

    current++;
    // Doesn't count with itself
    if (current == nodeId)
      current++;

    // Check if sync with all
    if (current > nNodes) {
      on = false;
      Serial.println("Synchronization complete.");
      return;
    }
  }
}

void Sync::answer_node(can_frame frame) {
  byte senderId = (frame.can_id >> shiftId) & idMask;
  uint8_t msg[data_bytes];
  encodeMessage(msg, sync_ans[0], sync_ans[1], 0);
  write(senderId, 0, msg);

  Serial.print("Sync: Answered node "); Serial.println(senderId);
}
