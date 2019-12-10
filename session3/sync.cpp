#include "sync.h"

void Sync::init(byte _nodeId, byte _nNodes) {
  on = true;
  nodeId = _nodeId;
  nNodes = _nNodes;
  first = true;
  for (byte i = 0; i < nNodes; i++) {
    handshakes[i] = false;
  }
  nHand = 0;
  last_time = millis();
}

bool Sync::isOn() {
  return on;
}

void Sync::ask_node() {
  unsigned long curr_time = millis();

  if (first || (curr_time - last_time >= timeout)) {
    first = false;
    write(0, sync_ask, 0);
    last_time = curr_time;
  }
}

void Sync::receive_answer(byte senderId) {
  if (!handshakes[senderId - 1]) {
    nHand++;
    handshakes[senderId - 1] = true;
    Serial.print("Handshaked with node "); Serial.println(senderId);
  }

  // Check if sync with all
  if (nHand >= nNodes - 1) {
    on = false;
    Serial.println("Synchronization complete.");
  }
}

void Sync::answer_node(byte senderId) {
  write(senderId, sync_ans, 0);

  Serial.print("Sync: Answered node "); Serial.println(senderId);
}
