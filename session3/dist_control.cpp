// Implementation of distributed control related functions

#include <Arduino.h>
#include <math.h>
#include "dist_control.h"

/*-------Variable definition--------*/
// LDR calibration
const float ldr_m[5] = { -0.67, -0.67, -0.67, 0, 0}; // LDR calibration
const float ldr_b[5] = {1.763, 1.763, 1.763, 0, 0}; // LDR calibration
float k[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Gains

// Control related variables
float Li = 10;

// Actuation
float measuredLux = 0;
float dutyCycle = 0;

/*--------Function definition--------*/
float getLux(int measurement) {

  float Vldr, Rldr, lux;

  measurement = map(measurement, 0, 1023, 0, 5000);
  Vldr = Vcc - measurement;
  Rldr = (float) Vldr * R1 / (Vcc - Vldr);
  lux = pow(10, (float) (log10(Rldr) - ldr_b[nodeId - 1]) / ldr_m[nodeId - 1]);

  return lux;
}

void LedConsensus::init(byte _nodeId, byte _nNodes, float _ci, float _oi, float _Li) {
  nodeId = _nodeId;
  nNodes = nNodes;
  n = dotProd(k, k);
  m = n - k[nodeId] * k[nodeId];
  ci = _ci;
  o = _oi;
  L = _Li;
  for (byte i = 0; i < nNodes; i++) {
    d_diagonal[i] = 0;
    d_column[i] = 0;
    d[i] = 0;
    d_av[i] = 0;
    y[i] = 0;
  }
}

float LedConsensus::measure_o() {
  float o;
  for (byte i = 0; i < nNodes; i++) {
    o = 0;
  }
}

float LedConsensus::dotProd(float x[5], float y[5]) {
  // Calculates the dot product of two vectors
  float aux_sum = 0;
  for (byte i = 0; i < nNodes; i++)
    aux_sum = aux_sum + x[i] * y[i];
  return aux_sum;
}

bool LedConsensus::check_feasibility(float* d) {
  return (d[nodeId - 1] <= 100 + tol && d[nodeId - 1] >= 0 - tol && dotProd(k, d) >= L - o - tol);
}

float LedConsensus::evaluate_cost(float *local_d, float rho) {
  float local_cost = 0;
  for (byte i = 0; i < nNodes; i++) {
    if (i == nodeId - 1)
      local_cost += ci * local_d[i];
    else
      local_cost += y[i] * (local_d[i] - d_av[i]) + rho / 2 * (local_d[i] - d_av[i]) * (local_d[i] - d_av[i]);
  }
  return local_cost;
}

//
// CALCULATE NEW O
//

void LedConsensus::consensus_iterate(float *d, float &cost, float rho) {
  // OUTPUTS:
  // dNode: array with local optimal led duty cycles (note: only need the own node duty cycle)
  // bool: true if feasible, false if unfeasible
  float d_best[5];
  float cost_best = 1 / 0;
  float z[5];

  for (byte i = 0; i < nNodes; i++)
    z[i] = rho * d_av[i] - y[i];
  z[nodeId - 1] -= ci;

  float d_temp[5];
  float cost_temp;

  // unconstrained minimum
  for (byte i = 0; i < nNodes; i++) {
    d_temp[i] = z[i] / rho;
  }

  // Now we check if this first solution is feasible
  if (check_feasibility(d_temp)) { // Solution is feasible
    Serial.println("Interior is feasible");
    memcpy(d_best, d_temp, nNodes * sizeof(float));
  }
  else {
    float aux;

    // Solution 1
    aux = (1 / n) * (o - L + (1 / rho) * dotProd(z, k));
    for (byte i = 0; i < nNodes; i++)
      d_temp[i] = (1 / rho) * z[i] - k[i] * aux;

    if (check_feasibility(d_temp)) { // Solution is feasible
      cost_temp = evaluate_cost(d_temp, rho);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    // Solution 2
    for (byte i = 0; i < nNodes; i++) {
      if (i != nodeId - 1)
        d_temp[i] = z[i] / rho;
      else
        d_temp[i] = 0;
    }

    if (check_feasibility(d_temp)) { // Solution is feasible
      cost_temp = evaluate_cost(d_temp, rho);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    // Solution 3
    // no need for the loop because the other elements are already initialized from solution 2
    d_temp[nodeId - 1] = 100;

    if (check_feasibility(d_temp)) { // Solution is feasible
      cost_temp = evaluate_cost(d_temp, rho);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    //Solution 4
    // simplified because values are partially computed from sol 2
    aux = dotProd(z, k);
    for (byte i = 0; i < nNodes; i++) {
      if (i != nodeId - 1)
        d_temp[i] += (-1 / m) * k[i] * (o - L) +
                     (1 / rho / m) * k[i] * (k[nodeId - 1] * z[nodeId - 1] - aux);
      else
        d_temp[i] = 0;
    }

    if (check_feasibility(d_temp)) { // Solution is feasible
      cost_temp = evaluate_cost(d_temp, rho);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    //Solution 5
    // simplified due to repeated calcs in 2, 3, 4
    for (byte i = 0; i < nNodes; i++) {
      if (i != nodeId - 1)
        d_temp[i] -= (-1 / m) * k[i] * 100 * k[nodeId - 1];
      else
        d_temp[nodeId - 1] = 100;
    }

    if (check_feasibility(d_temp)) { // Solution is feasible
      cost_temp = evaluate_cost(d_temp, rho);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }
  }

  memcpy(d, d_best, nNodes * sizeof(float));
  cost = cost_best;
}

void LedConsensus::consensus_run() {
  float di[nNodes];
  float costi;

  if (counter == 0)
    o = measure_o();

  consensus_iterate(di, costi, rho);
  memcpy(d, di, nNodes * sizeof(float));

  d_av[nodeId - 1] = 0;
  for (byte i = 0; i < nNodes; i++) {
    d_av[nodeId - 1] += d_column[i];
  }
  d_av[nodeId - 1] /= 3;

  for (byte i = 0; i < nNodes; i++) {
    y[i] += rho * (d[i] - d_av[i]);
  }

  counter++;
}
