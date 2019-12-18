// Implementation of system calibration class

#include "calibr.h"

/*-------Function definition--------*/

bool Calibration::isOn() {
  return on;
}

float Calibration::getB(int measurement) {
  float Vldr, Rldr;
  measurement = map(measurement, 0, 1023, 0, 5000);
  Vldr = Vcc - measurement;
  Rldr = (float) Vldr * R1 / (Vcc - Vldr);
  return log10(Rldr) - m[nodeId - 1] * log10(max_lux);
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
    if (nHand >= (nNodes - 1)) {
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
        write(0, calibr_wait, nodeCounter);
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

        /*// Check if end calibration
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
          }*/

        action = measure;
        break;

      case measure:
        if (nodeCounter > nNodes) {
          on = false;

          b[nodeId - 1] = getB(measurements[nodeId]);

          float o_temp = getLux(measurements[0]);
          ledConsensus.setLocalO(o_temp);

          for (byte i = 1; i <= nNodes; i++) {
            k[i - 1] = getLux(measurements[i]) / 100;
            Serial.println(k[i - 1]);
          }

          //
          float aux = 0;
          for (byte i = 0; i < nNodes; i++)
            aux += 100 * k[i];
          Serial.print("Max actuation is "); Serial.println(aux);
          //

          Serial.println("Calibration complete");
          return;
        }
        measurements[nodeCounter] = analogRead(ldrPin);

        nodeCounter++;
        action = turn;
        break;
    }

    delay(500);
    write(0, calibr_wait, nodeCounter);
    waiting = true;
    last_time = millis();
    Serial.println("Wrote wait message");
  }
  return;
}

void Calibration::receive_answer(byte senderId, float value) {
  Serial.print("Current "); Serial.println(nodeCounter);
  if (!handshakes[senderId - 1] && ((int) value) == nodeCounter) {
    nHand++;
    handshakes[senderId - 1] = true;
  }
}

void Calibration::send_answer(byte senderId, float value) {
  if (value <= nodeCounter)
    write(senderId, calibr_answer, value);

  Serial.print("Calibr: Answered node "); Serial.println(senderId);
}
