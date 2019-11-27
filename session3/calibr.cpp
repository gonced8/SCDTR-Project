// Implementation of system calibration class

#include "calibr.h"

/*-------Variable definition--------*/


void CALIBRATION::init(byte id, byte n) {
	nodeId = id;
	nNodes = n;
}

void CALIBRATION::run(float* measurements) {

	unsigned long c;
	byte nodeCounter = -1;
	measurements = (float*) malloc(nNodes + 1);

	while(nodeCounter < nNodes) {
		if (nodeCounter == nodeId)
			analogWrite(ledPin, 255);
		else
			analogWrite(ledPin, 0);

		// Synchronization code, wait for other nodes
		byte ii = 0;
		while(ii+1 != nNodes) {
			if (ii == nodeId) {
				// write message to bus
				write(0, 0, 0, (byte) nodeId);
			}
			// read message from bus
			// if other node sends message, increase counter
			while(read(c) == MCP2515::ERROR_OK) {
				ii++;
			}
		}

		// Measure ldr voltage, compute gain
		measurements[nodeCounter + 1] = analogRead(ldrPin);
		nodeCounter++;
	}
}
