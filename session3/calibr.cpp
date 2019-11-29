// Implementation of system calibration class

#include "calibr.h"
/*-------Variable definition--------*/


void Calibration::init(byte id, byte n, float *&measurements) {
  nodeId = id;
  nNodes = n;
  measurements = (float *) malloc((n + 1) * sizeof(float));
  turn_led(0);
  write(0, 0, 'c');
}

void Calibration::run(float *measurements) {
  if (nodeCounter == (nNodes - 1)) {
    nodeCounter = 0;
    if (measure_flag) {
      measurements[current] = analogRead(ldrPin);
      measure_flag = false;
      write(0, 0, 'c');
    }
    else {
      measure_flag = true;
      current++;
    }
    
    if (current == nodeId)
      turn_led(255);
    else
      turn_led(0);

    if (current == (nNodes + 1))
      on = false;
    else
      write(0, 0, 'c');
  }
  else
    nodeCounter++;
}

void Calibration::turn_led(byte value) {
  analogWrite(ledPin, value);
}

bool Calibration::isOn() {
  return on;
}
