#include "controller.h"

void Controller::init(byte _nodeId, byte _nNodes) {
  on = true;
  nodeId = _nodeId;
  nNodes = _nNodes;
  currentIndex = -1;
  currentNode = 0;
  wait = false;
  last_time = millis();
  for (int i = 0; i < nNodes; i++) {
    dutyCycles[i] = 0;
  }
}

bool Controller::isOn() {
  return on;
}

void Controller::ask_duty_cycle() {
  unsigned long curr_time = millis();

  if (!wait) {
    incrementIndices();

    // Check if sync with all
    if (current > nNodes) {
      on = false;
      Serial.println("Transmission complete.");
      return;
    }
  }
  else if (curr_time - last_time < timeout) {
    return;
  }

  uint8_t msg[data_bytes];
  encodeMessage(msg, ctrl_send[0], (char) currentIndex, 0);
  write(currentNode, 0, msg);
  wait = true;
  last_time = curr_time();
}

void Controller::receive_duty_cycle(can_frame frame) {
  byte senderId = (frame.can_id >> shiftId) & idMask;

  if (senderId == current) {
    handshake = true;
    Serial.print("Receveid answer from node "); Serial.println(senderId);
  }
}

void Controller::answer_duty_cycle(can_frame frame) {
  byte senderId = (frame.can_id >> shiftId) & idMask;
  uint8_t msg[data_bytes];
  encodeMessage(msg, sync_ans[0], sync_ans[1], 0);
  write(current, 0, msg);
}

void Controller::incrementIndices() {
  currentIndex++;
  // Doesn't count with itself
  if (currentIndex == nodeId)
    currentIndex++;

  if(currentIndex > nNodes){
    currentIndex = 0;
    
    currentNode++;
    if (currentNode == nodeId)
      currentNode++;    
  }
}
