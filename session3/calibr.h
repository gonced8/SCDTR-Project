// System gains calibration related functions

#ifndef CALIBR_H
#define CALIBR_H

// System imports
#include <Arduino.h>
#include <mcp2515.h>
#include <stdlib.h>

// Custom imports
#include "can_comms.h"
#include "dist_control.h"

/*-------Variable declaration-------*/


/*---------Type definition----------*/
class Calibration {

	byte nNodes, nodeId, nodeCounter = -1;

	public:
    void init(byte id, byte n, float *&measurements);
		void run(float *measurements);

};
/*--------Function propotypes--------*/


#endif // CALIBR_H
