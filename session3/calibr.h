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
    byte nNodes, nodeId = -1;
    byte nodeCounter = 0;
    bool on = true;
    byte current = 0;
    bool measure_flag = false;
    char *measurements;

  public:
    void init(byte id, byte n, float *&measurements);
    void run(float *measurements);
    void turn_led(byte value);
    bool isOn();
};
/*--------Function propotypes--------*/


#endif // CALIBR_H
