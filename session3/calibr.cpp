// Implementation of system calibration class

#include "calibr.h"

/*-------Variable definition--------*/


void CALIBRATION::init(byte id, byte n) {
	nodeId = id;
	nNodes = n;
}

void CALIBRATION::run(float *measurements) {
	unsigned long c;

	for(byte nodeCounter=0; nodeCounter <= nNodes; nodeCounter++) {
		if (nodeCounter == nodeId)
			analogWrite(ledPin, 255);
		else
			analogWrite(ledPin, 0);

		barrier();

		// Measure ldr voltage, compute gain
		measurements[nodeCounter] = analogRead(ldrPin);

    barrier();
	}
}
