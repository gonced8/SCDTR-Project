// Implementation of system calibration class

#include "calibr.h"

/*-------Function definition--------*/

bool Calibration::isOn() {
  return on;
}

void Calibration::init(byte id, byte n) {
  nodeId = id;
  nNodes = n;
  on = true;
  waiting = false;
  action = turn;
  nodeCounter = 0;
  nAck = 0;
}

void Calibration::run(LedConsensus &ledConsensus) {
  uint8_t msg[data_bytes];

  if (waiting) {
    if (nAck >= (nNodes - 1)) {
      nAck -= (nNodes - 1);
      waiting = false;
    }
  }
  else {
    switch (action) {
      case turn:
        if (nodeCounter == nodeId)
          analogWrite(ledPin, 255);
        else
          analogWrite(ledPin, 0);

        // Check if end calibration
        if (nodeCounter > nNodes) {
          on = false;

          float o_temp = getLux(measurements[0]);
          ledConsensus.setLocalO(o_temp);

          for (byte i = 1; i <= nNodes; i++) {
            k[i - 1] = getLux(measurements[i]) / 100;
            Serial.println(k[i - 1]);
          }

          Serial.println("Calibration complete");
          return;
        }

        encodeMessage(msg, calibr_wait[0], calibr_wait[1], 0);
        write(0, 0, msg);
        Serial.println("Wrote wait message");

        action = measure;
        waiting = true;
        break;

      case measure:
        measurements[nodeCounter] = analogRead(ldrPin);

        encodeMessage(msg, calibr_wait[0], calibr_wait[1], 0);
        write(0, 0, msg);
        Serial.println("Wrote wait message");

        nodeCounter++;
        action = turn;
        waiting = true;
        break;
    }
  }
  return;
}

void Calibration::receiveAck() {
  Serial.print("Ack "); Serial.println(nAck);
  nAck++;
}
