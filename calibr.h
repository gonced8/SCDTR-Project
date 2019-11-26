// System gains calibration related functions

#ifndef CALIBR_H
#define CALIBR_H

// System imports
#include <Arduino.h>


/*-------Variable declaration-------*/


/*---------Type definition----------*/
class CALIBRATION {

	byte nNodes, nodeId, nodeCounter = -1;

	public:
		void init(byte nodeId, byte nNodes);
		void run(float* measurements);

};
/*--------Function propotypes--------*/


#endif // CALIBR_H
