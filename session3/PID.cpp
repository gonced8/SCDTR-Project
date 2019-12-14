// Implementation of PID class

// Custom imports
#include "PID.h"

void PID::init(float p, float i, float d, float _T, float _a) {

	kp = p; ki = i; kd = d; T = _T; a = _a;
	ep = yp = ip = dp = 0.0f;

	k1 = kp;
	k2 = kp * ki * T / 2;

	float den = kd + a * T;
	if (den < 0.000001f)
		k3 = k4 = 0;
	else {
		k3 = kd / den;
		k4 = kp * kd * a / den;
	}
}

float PID::calc(float ref, float y, bool saturate) {

	float e = ref - y;

	// Error deadzone
	if (e >= 0.5)
		e = e - 0.5;
	else if (e <= -0.5)
		e = e + 0.5;
	else
		e = 0;

	float p = k1 * e;
	float i;

   	if (saturate)
		i = ip;
	else
		i = ip + k2 * (e + ep);
	float d = k3 * dp - k4 * (y - yp);
	yp = y;
	ep = e;
	ip = i;
	dp = d;

	return p + i + d;
}
