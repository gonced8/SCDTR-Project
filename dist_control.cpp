// Implementation of distributed control related functions


// System imports
#include <Arduino.h>

// Custom imports
#include "hub.h"
#include "dist_control.h"

/*-------Variable definition--------*/
extern const float m[3] = {1, 1, 1};
extern const float b[3] = {1, 1, 1};
extern float k[3] = {0.0, 0.0, 0.0};

/*--------Function definition--------*/
float getLux(float measurement) {

	float Vldr, Rldr, lux;

	measurement = map(measurement, 0, 1023, 0, 5000);
	Vldr = Vcc - measurement;
	Rldr = (float) Vldr * R1 / (Vcc - Vldr);
	lux = pow(10, (float) (log10(Rldr) - b[nodeId]) / m[nodeId]);

	return lux;
}
