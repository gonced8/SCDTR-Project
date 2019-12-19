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
  state = 0;
  nodeCounter = 0;
  for (byte i = 0; i < nNodes; i++) {
    boolArray[i] = false;
  }
  nBool = 0;
}

void Calibration::run(LedConsensus &ledConsensus) {
  uint8_t msg[data_bytes];

  switch (state) {
    // Start
    case 0:
      ask();
      break;

    // Set led
    case 1:
      Serial.print("nodeCounter = "); Serial.println(nodeCounter);
      if (nodeCounter == nodeId)
        analogWrite(ledPin, 255);
      else
        analogWrite(ledPin, 0);
      state++;
      break;

    // Sync
    case 2:
      ask();
      break;

    // Measure
    case 3:
      delay(500);
      measurements[nodeCounter] = analogRead(ldrPin);
      nodeCounter++;
      state++;
      break;

    case 4:
      ask();
      break;
  }
}

void Calibration::ask() {
  unsigned long current_time = millis();
  char code;

  switch (state) {
    case 0:
      code = calibr_start_ask;
      break;
    case 2:
      code = calibr_set_ask;
      break;
    case 4:
      code = calibr_measure_ask;
      break;
  }

  if (first) {
    write(0, code, 0);
    last_time = current_time;
    first = false;
  }
  else if (current_time - last_time >= timeout) {
    for (byte i = 1; i <= nNodes; i++) {
      if (i != nodeId && !boolArray[i - 1]) {
        write(i, code, 0);
      }
    }
    last_time = current_time;
  }
}

void Calibration::ans(byte senderId, char code) {
  bool valid = false;
  char ans_code;
  float ans_value;

  switch (code) {
    case calibr_start_ask:
      valid = (state <= 2);
      ans_code = calibr_start_ans;
      break;
    case calibr_set_ask:
      valid = (state >= 2 && state <= 4);
      ans_code = calibr_set_ans;
      break;
    case calibr_measure_ask:
      valid = (state >= 4 || (state >= 1 && state <= 2));
      ans_code = calibr_measure_ans;
      break;
  }

  if (valid) {
    write(senderId, ans_code, 0);
    rcv(senderId, ans_code);
  }
}

void Calibration::rcv(byte senderId, char code) {
  if (!boolArray[senderId - 1]) {
    bool valid = false;

    switch (code) {
      case calibr_start_ans:
        valid = (state == 0);
        break;
      case calibr_set_ans:
        valid = (state == 2);
        break;
      case calibr_measure_ans:
        valid = (state == 4);
        break;
    }

    if (valid) {
      boolArray[senderId - 1] = true;
      nBool++;

      if (nBool == nNodes - 1) {
        // Final state
        if (state == 4) {
          // Next config
          if (nodeCounter <= nNodes)
            state = 1;
          // End calibration
          else {
            state = 5;  // state doesn't exist. standby state
            on = false;

            b[nodeId - 1] = getB(measurements[nodeId]);

            for (byte i = 1; i <= nNodes; i++) {
              k[i - 1] = getLux(measurements[i]) / 100;
              Serial.println(k[i - 1]);
            }

            Serial.println("Calibration complete");
          }
        }
        else
          state++;

        resetBool();
      }
    }
  }
}

void Calibration::resetBool() {
  for (byte i = 0; i < nNodes; i++) {
    boolArray[i] = false;
  }
  nBool = 0;
  first = true;
}
