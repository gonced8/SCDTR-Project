// Implementation of distributed control related functions

#include <Arduino.h>
#include "dist_control.h"

/*-------Variable definition--------*/
const float m[3] = {1, 1, 1}; // LDR calibration
const float b[3] = {1, 1, 1}; // LDR calibration
float k[3] = {0.0, 0.0, 0.0}; // Gains

float c[3] = {0.0, 0.0, 0.0}; // Local cost vector TODO: initialize c_i and c[] in setup after getNodeId() calling costCalc!!!
float c_i = 1; // Cost weight for local node (c[] will have c_i)
float node_d[3] = {0.0, 0.0, 0.0}; // Duty cycle
float y[3] = {1.0, 1.0, 1.0}; // Lagrange multipliers
float d_avg[3] = {0.0, 0.0, 0.0}; // Average actuation of all local d_is
float d_localavg = 0; // Local average, part of the vector above
float ro = 1; // Quadratic term multiplier
float o_i = 0; // External luminance
float L_i = 0; // Local luminance lower bound
float f_i = 0; // Local cost

//TODO: check which variables are not needed
/*--------Function definition--------*/
float getLux(float measurement) {

  float Vldr, Rldr, lux;

  measurement = map(measurement, 0, 1023, 0, 5000);
  Vldr = Vcc - measurement;
  Rldr = (float) Vldr * R1 / (Vcc - Vldr);
  lux = pow(10, (float) (log10(Rldr) - b[nodeId]) / m[nodeId]);

  return lux;
}

void costCalc() {
  for (byte i = 1; i <= 3; i++) {
    if (i == nodeId)
      c[i - 1] = c_i;
    else
      c[i - 1] = 0.0;
  }
}

float f_iCalc(float d[3]) {
  if (d[id - 1] <= 100 && d[id - 1] >= 0 && k[0]*d[0] + k[1]*d[1] + k[2]*d[2] >= L_i - o_i)
    return c_i * k[id - 1];
  else
    return infinity;
}

float d_localavgCalc(float d[3]) { // Calculates the average of the node
  return (d[0] + d[1] + d[2]) / 3;
}

float lanmultiplierCalc(float d[3]) {
  return (y[0] * (d[0] - d_avg[0]) + y[1] * (d[1] - d_avg[1]) + y[2] * (d[2] - d_avg[2]));
}

float distanceCalc(float d[3]) {
  return ( (d[0] - d_avg[0]) * (d[0] - d_avg[0]) + (d[1] - d_avg[1]) * (d[1] - d_avg[1]) + (d[2] - d_avg[2]) * (d[2] - d_avg[2]) );
}
// TODO: see what functions are not necessary
float lagrangeCalc(float f_i, float d[3]) {
  return (f_i + lanmultiplierCalc(d, d_avg, y) + ro * distanceCalc(d, d_avg) / 2);
}

void ziCalc(float* zi) {
  zi[0] = ro * d_avg[0] - c[0] - y[0];
  zi[1] = ro * d_avg[1] - c[1] - y[1];
  zi[2] = ro * d_avg[2] - c[2] - y[2];
}

float dotProd(float x[3], float y[3]) {
  // Calculates the dot product of two vectors
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

bool findminima() {
  // OUTPUTS:
  // node_d: array with local optimal led duty cycles (note: only need the own node duty cycle)
  // bool: true if feasible, false if unfeasible
  float zi[3];
  float knorm = k[0] * k[0] + k[1] * k[1] + k[2] * k[2]; // TODO: put this constant global and calculated after each calibration

  // First we will try to find the solution in the interior
  for (byte i = 1; i <= 3; i++) {
    if (i != nodeId) //Not our node
      newd[i - 1] = d_avg[i - 1] - y[i - 1] / ro;
    else
      newd[i - 1] = d_avg[i - 1] - y[i - 1] / ro - c_i / ro;
  }
  // Now we check if this first solution is feasible
  if (f_iCalc(newd) != infinity) // Solution is feasible
    node_d = newd;
  return true;

  // Else continue looking for solutions on the borders
  float newd[3];
  float newd2[3];
  float newd3[3];
  float newd4[3];
  float newd5[3];
  float* dvec_pointer[] = {newd, newd2, newd3, newd4, newd5};
  bool feasible[5] = {false, false, false, false, false};
  //TODO: optimize the solutions generation
  //      there are factors in common among different solutions, make only one loop

  // Solution 1
  ziCalc(zi);
  float aux_cte = (1 / knorm) * (o_i - L_i + (1 / ro) * dotProd(k, zi));
  for (byte i = 1; i <= 3; i++)
    newd[i - 1] = (1 / ro) * zi[i - 1] - k[i - 1] * aux_cte;
  if (f_iCalc(newd) != infinity) // Solution is feasible
    feasible[0] = true;

  // Solution 2
  for (byte i = 1; i <= 3; i++) {
    if (i != nodeId)
      newd2[i - 1] = zi[i - 1] / ro;
    else
      newd2[i - 1] = 0;
  }
  if (f_iCalc(newd2) != infinity) // Solution is feasible
    feasible[1] = true;

  // Solution 3
  for (byte i = 1; i <= 3; i++) {
    if (i != nodeId)
      newd3[i - 1] = zi[i - 1] / ro;
    else
      newd3[i - 1] = 100;
  }
  if (f_iCalc(newd3) != infinity) // Solution is feasible
    feasible[2] = true;

  //Solution 4
  for (byte i = 1; i <= 3; i++) {
    if (i != nodeId)
      newd4[i - 1] = zi[i - 1] / ro - (k[i - 1] / (knorm - k[nodeId - 1] * k[nodeId - 1])) * (o_i - L_i) + (1 / ro) * (k[i - 1] / (knorm - k[nodeId - 1] * k[nodeId - 1])) * (-dotProd(k, zi) + k[nodeId - 1] * zi[nodeId - 1]);
    else
      newd4[i - 1] = 0;
  }
  if (f_iCalc(newd4) != infinity) // Solution is feasible
    feasible[3] = true;

  //Solution 5
  for (byte i = 1; i <= 3; i++) {
    if (i != nodeId)
      //newd5[i-1] = zi[i-1]/ro - (k[i-1]*(o_i - L_i) + 100*k[i-1]*k[nodeId-1])/(knorm - k[nodeId -1]*k[nodeId -1]) - (1/ro)*(k[i-1]/(knorm - k[nodeId -1]*k[nodeId -1]))*(-dotProd(k, zi) + k[nodeId-1]*zi[nodeId-1]);
      newd5[i - 1] = newd4[i - 1] - (100 * k[i - 1] * k[nodeId - 1]) / (knorm - k[nodeId - 1] * k[nodeId - 1]);
    else
      newd5[i - 1] = 0;
  }
  if (f_iCalc(newd5) != infinity) // Solution is feasible
    feasible[4] = true;

  // See the minimum cost between the feasible possibilities
  float lagrangean_aux = 0;
  float ft = 0;
  bool first = true;
  for (byte i = 1; i <= 5; i++) {
    if (feasible[i - 1]) {
      lagrangean_aux = (rho / 2) * dotProd(dvec_pointer[i - 1], dvec_pointer[i - 1]) - dotProd(dvec_pointer[i - 1], zi);
      if (first || lagrangean_aux < ft) {
        ft = lagrangean_aux;
        node_d = dvec_pointer[i - 1];
        first = false;
      }
    }
  }

  if (first)
    return false;    // Unfeasible

  return true;
}
