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
  for (byte i = 0; i < nNodes; i++) {
    handshakes[i] = false;
  }
  nHand = 0;
}

void Calibration::run(LedConsensus &ledConsensus) {
  uint8_t msg[data_bytes];

  if (waiting) {
    // Successful handshakes
    if (nHand == nNodes - 1) {
      waiting = false;

      // Reset handshakes array
      for (byte i = 0; i < nNodes; i++) {
        handshakes[i] = false;
      }
      nHand = 0;
    }
    // Missing handshakes
    else {
      unsigned long curr_time = millis();
      // Timed out, send again
      if (curr_time - last_time >= timeout) {
        write(0, calibr_wait, 0);
        last_time = curr_time;
        Serial.println("Wrote wait message");
      }
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

        action = measure;
        break;

      case measure:
        delay(500);
        measurements[nodeCounter] = analogRead(ldrPin);

        nodeCounter++;
        action = turn;
        break;
    }

    write(0, calibr_wait, 0);
    waiting = true;
    last_time = millis();
    Serial.println("Wrote wait message");
  }
  return;
}

void Calibration::receive_answer(byte senderId) {
  if (!handshakes[senderId - 1]) {
    nHand++;
    handshakes[senderId - 1] = true;
  }
}

void Calibration::send_answer(byte senderId) {
  uint8_t msg[data_bytes];
  write(senderId, calibr_answer, 0);

  Serial.print("Sync: Answered node "); Serial.println(senderId);
}
