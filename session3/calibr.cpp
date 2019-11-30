// Implementation of system calibration class

#include "calibr.h"

/*-------Function definition--------*/

bool Calibration::isOn() {
  return on;
}

void Calibration::start(byte id, byte n) {
  nodeId = id;
  nNodes = n;
  on = true;
  waiting = false;
  action = turn;
  nodeCounter = 0;
  nAck = 0;

  if (measurements != NULL)
    free(measurements);

  measurements = (float *) malloc((nNodes + 1) * sizeof(float));
}

void Calibration::run() {
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

        if (nodeCounter == (nNodes + 1)) {
          on = false;
          return;
        }

        encodeMessage(msg, calibr_wait[0], calibr_wait[1], 0);
        write(0, 0, msg);

        action = measure;
        waiting = true;
        break;

      case measure:
        measurements[nodeCounter] = analogRead(ldrPin);

        encodeMessage(msg, calibr_wait[0], calibr_wait[1], 0);
        write(0, 0, msg);

        nodeCounter++;
        action = turn;
        waiting = true;
        break;
    }
  }
  return;
}

void Calibration::receiveAck() {
  nAck++;
}
