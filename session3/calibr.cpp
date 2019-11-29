// Implementation of system calibration class

#include "calibr.h"
/*-------Variable definition--------*/


void CALIBRATION::init(byte id, byte n, float *&measurements) {
	nodeId = id;
	nNodes = n;
  measurements = (float *) malloc((n+1)*sizeof(float));
}

void CALIBRATION::run(float *measurements) {
	for(byte nodeCounter=0; nodeCounter <= nNodes; nodeCounter++) {
		if (nodeCounter == nodeId)
			analogWrite(ledPin, 255);
		else
			analogWrite(ledPin, 0);

    Serial.println(nodeCounter);

		barrier();

    Serial.println("Past first barrier");
		// Measure ldr voltage, compute gain
    delay(500);
		measurements[nodeCounter] = analogRead(ldrPin);

    barrier();

    Serial.println("Past second barrier");
	}
}
