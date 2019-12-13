// Implementation of distributed control related functions

#include <Arduino.h>
#include <math.h>
#include "dist_control.h"

/*-------Variable definition--------*/
// LDR calibration
const float m[3] = { -0.67, -0.67, -0.67}; // LDR calibration
const float b[3] = {1.763, 1.763, 1.763}; // LDR calibration
float k[3] = {0.0, 0.0, 0.0}; // Gains

// Control related variables
float luxRefUnocc = 20;
float luxRefOcc = 40;
bool deskOccupancy = false;

// Optimization
const float infinity = 1.0 / 0.0;

// Actuation
float measuredLux = 0;
float dutyCycle = 0;

/*--------Function definition--------*/
float getLux(int measurement) {

  float Vldr, Rldr, lux;

  measurement = map(measurement, 0, 1023, 0, 5000);
  Vldr = Vcc - measurement;
  Rldr = (float) Vldr * R1 / (Vcc - Vldr);
  lux = pow(10, (float) (log10(Rldr) - b[nodeId - 1]) / m[nodeId - 1]);

  return lux;
}

void calcDisturbance(LedConsensus &ledConsensus, float measured) {
  float new_o = measured - ledConsensus.calcExpectedLux();
  Serial.print("New o is "); Serial.println(new_o);
  ledConsensus.setLocalO(new_o);
  ledConsensus.startCounter();
  /*
    if (abs(new_o - ledConsensus.getLocalO()) > 2) {
    ledConsensus.setLocalO(new_o);
    ledConsensus.startCounter();
    ledConsensus.tellOthers();
    Serial.println("Will enter consensus again");
    }
  */
}

void LedConsensus::tellOthers() {
  waiting = true;
}

void LedConsensus::tellStart() {
  unsigned long curr_time = millis();

  if (first || (curr_time - last_time >= timeout)) {
    first = false;
    write(0, consensus_tell, 0);
    last_time = curr_time;
  }
  Serial.print("Consensus: Told all nodes");
}

void LedConsensus::rcvAns(byte senderId) {
  if (!handshakes[senderId - 1]) {
    nHand++;
    handshakes[senderId - 1] = true;
    Serial.print("Handshaked with node "); Serial.println(senderId);
  }

  // Check if sync with all
  if (nHand >= nNodes - 1) {
    // Reset setup
    for (byte i = 0; i < nNodes; i++)
      handshakes[i] = false;
    nHand = 0;
    first = true;
    // Stop waiting
    waiting = false;
    Serial.println("Consensus telling complete.");
  }
}

/*
  void LedConsensus::rcvStart(byte senderId) {
  write(senderId, consensus_rcv, 0);
  if (finished())
    startCounter();
  Serial.print("Consensus: Answered node "); Serial.println(senderId);
  }
*/

void LedConsensus::init(byte _nodeId, byte _nNodes, float _rho, byte _c_i, float* new_y) {
  nodeId = _nodeId;
  nNodes = _nNodes;
  // Consensus setup
  rho = _rho;
  c_i = _c_i;
  memcpy(y, new_y, sizeof(y));
  startCounter();
  firstPart = true;
  for (int i = 0; i < nNodes; i++) {
    c[i] = 0;
    dNode[i] = 0;
    dNodeOverall[i] = 0;
    dAvg [i] = 0;
  }
  c[nodeId - 1] = c_i;

  // Ref setup
  if (deskOccupancy)
    setLocalL(luxRefOcc);
  else
    setLocalL(luxRefUnocc);
  // Comms setup
  waiting = false;
  last_time = millis();

  state = 0;
  first = true;
}

void LedConsensus::ziCalc(float* zi) {
  for (byte i = 0; i < nNodes; i++)
    zi[i] = rho * dAvg[i] - c[i] - y[i];
}

float LedConsensus::dotProd(float x[5], float y[5]) {
  // Calculates the dot product of two vectors
  float aux_sum = 0;
  for (byte i = 0; i < nNodes; i++)
    aux_sum = aux_sum + x[i] * y[i];

  return aux_sum;
}

bool LedConsensus::f_iCalc(float* d) {
  return (d[nodeId - 1] <= 100 && d[nodeId - 1] >= 0 && dotProd(k, d) >= L_i - o_i);
}

void LedConsensus::setLocalC(float _c_i) {
  c_i = _c_i;
  c[nodeId - 1] = c_i;
}

float LedConsensus::getLocalC() {
  return c_i;
}

void LedConsensus::setLocalO(float _o_i) {
  o_i = _o_i;
}

float LedConsensus::getLocalO() {
  return o_i;
}

void LedConsensus::setLocalL(float _L_i) {
  L_i = _L_i;
}

float LedConsensus::getLocalL() {
  return L_i;
}

void LedConsensus::getLocalDMean(float* new_dAvg) {
  memcpy(new_dAvg, dAvg, nNodes * sizeof(float));
}

void LedConsensus::getLocalD(float* d) {
  memcpy(d, dNode, nNodes * sizeof(float));
}

float LedConsensus::calcExpectedLux() {
  return dotProd(k, dNodeOverall);
}

void LedConsensus::calcNewO() {
  float new_o = getLux(analogRead(ldrPin)) - calcExpectedLux();
  Serial.print("New o inside consensus is "); Serial.println(new_o);
  o_i = new_o;
}

void LedConsensus::startCounter() {
  remainingIters = maxIters;
}

float LedConsensus::evaluateCost(float* local_d) {
  float local_cost = 0;
  for (byte i = 0; i < nNodes; i++)
    local_cost += y[i] * (local_d[i] - dAvg[i]) + rho / 2 * (local_d[i] - dAvg[i]) * (local_d[i] - dAvg[i]);
  local_cost += c_i * local_d[nodeId - 1];
  return local_cost;
}

bool LedConsensus::findMinima() {
  // OUTPUTS:
  // dNode: array with local optimal led duty cycles (note: only need the own node duty cycle)
  // bool: true if feasible, false if unfeasible
  float d_temp[maxNodes];
  float d_best[maxNodes];
  float cost_best = infinity;
  float cost_temp;
  float zi[maxNodes];

  for (byte i = 0; i < nNodes; i++)
    zi[i] = rho * dAvg[i] - y[i];
  zi[nodeId - 1] -= c_i;

  // First we will try to find the solution in the interior
  for (byte i = 0; i < nNodes; i++)
    d_temp[i] = zi[i] / rho;

  // Now we check if this first solution is feasible
  if (f_iCalc(d_temp)) { // Solution is feasible
    memcpy(d_best, d_temp, nNodes * sizeof(float));
    cost_best = evaluateCost(d_temp);
    Serial.println("Interior is feasible");
  }
  else {
    // Else continue looking for solutions on the borders
    float aux;
    float aux2;
    // Solution 1
    aux = (1 / dotProd(k, k)) * (o_i - L_i + (1 / rho) * dotProd(k, zi));
    for (byte i = 0; i < nNodes; i++)
      d_temp[i] = (1 / rho) * zi[i] - k[i] * aux;

    if (f_iCalc(d_temp)) { // Solution is feasible
      cost_temp = evaluateCost(d_temp);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    // Solution 2
    for (byte i = 0; i < nNodes; i++)
      d_temp[i] = zi[i] / rho;
    d_temp[nodeId - 1] = 0;

    if (f_iCalc(d_temp)) { // Solution is feasible
      cost_temp = evaluateCost(d_temp);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    // Solution 3
    // Other terms are already calculated from solution2 loop
    d_temp[nodeId - 1] = 100;

    if (f_iCalc(d_temp)) { // Solution is feasible
      cost_temp = evaluateCost(d_temp);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    //Solution 4
    aux = dotProd(k, k) - k[nodeId - 1] * k[nodeId - 1];
    aux2 = -dotProd(k, zi) + k[nodeId - 1] * zi[nodeId - 1];
    for (byte i = 0; i < nNodes; i++)
      d_temp[i] = zi[i] / rho - (k[i] / (aux)) * (o_i - L_i) + (1 / rho) * (k[i] / (aux)) * (aux2);
    d_temp[nodeId - 1] = 0;

    if (f_iCalc(d_temp)) { // Solution is feasible
      cost_temp = evaluateCost(d_temp);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }

    //Solution 5
    for (byte i = 0; i < nNodes; i++)
      d_temp[i] = d_temp[i] - (100 * k[i] * k[nodeId - 1]) / aux;
    d_temp[nodeId - 1] = 100;

    if (f_iCalc(d_temp)) { // Solution is feasible
      cost_temp = evaluateCost(d_temp);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }
  }

  memcpy(dNode, d_best, nNodes * sizeof(float));
  dColumn[nodeId - 1] = d_best[nodeId - 1];
  cost = cost_best;
}


void LedConsensus::calcMeanVector() {
  dColumn[nodeId - 1] = dNode[nodeId - 1];
  dAvg[nodeId - 1] = 0;
  for (byte i = 0; i < nNodes; i++) {
    dAvg[nodeId - 1] += dColumn[i];
  }
  dAvg[nodeId - 1] /= nNodes;
}

void LedConsensus::calcLagrangeMult() {
  for (byte i = 0; i < nNodes; i++) {
    y[i] = y[i] + rho * (dNode[i] - dAvg[i]);
  }
}

void LedConsensus::run() {
  /*if (waiting) {
    tellStart();
    }
    else {*/
  //Serial.println(state);

  float aux;

  Serial.println(state);
  switch (state) {
    // Measure lux
    case 0:
      measuredLux = getLux(analogRead(ldrPin));
      Serial.print("Measured lux is "); Serial.println(measuredLux);
      aux = measuredLux - calcExpectedLux();
      Serial.print("New o is "); Serial.println(aux);
      setLocalO(aux);
      startCounter();
      state++;
      break;

    // Find minima
    case 1:
      findMinima();
      state++;
      break;

    // Receive duty cycles
    case 2:
      ask_duty_cycles();
      break;

    // Compute average
    case 3:
      calcMeanVector();
      state++;
      break;

    // Receive means
    case 4:
      ask_mean();
      break;

    // Calculate y
    case 5:
      calcLagrangeMult();
      remainingIters--;
      Serial.print("Remaining iterations: "); Serial.println(remainingIters);
      if (remainingIters > 0)
        state = 1;
      else
        state++;
      break;

    // Update led duty cycle
    case 6:
      dNodeOverall[nodeId - 1] = dNode[nodeId - 1];
      analogWrite(ledPin, dNodeOverall[nodeId - 1] * 255.0 / 100);
      state++;
      break;

    // Receive d real
    case 7:
      ask_real_d();
      break;
  }
}

void LedConsensus::ask_duty_cycles() {
  unsigned long current_time = millis();

  if (first || current_time - last_time >= timeout) {
    for (byte i = 1; i <= nNodes; i++) {
      if (i != nodeId) {
        write(i, duty_cycle_ask, dNode[i - 1]);
      }
    }
    last_time = current_time;
  }
}

void LedConsensus::ans_duty_cycles(byte senderId) {
  if (state >= 2 && state <= 4)
    write(senderId, duty_cycle_ans, dNode[senderId - 1]);
}

void LedConsensus::rcv_duty_cycles(byte senderId, float value) {
  if (state != 2)
    return;

  if (!boolArray[senderId - 1]) {
    boolArray[senderId - 1] = true;
    nBool++;
    dColumn[senderId - 1] = value;
  }

  if (nBool == nNodes - 1) {
    state++;
    resetBool();
  }
}

void LedConsensus::ask_mean() {
  unsigned long current_time = millis();

  if (first || current_time - last_time >= timeout) {
    for (byte i = 1; i <= nNodes; i++) {
      if (i != nodeId) {
        write(i, mean_ask, dAvg[nodeId - 1]);
      }
    }
    last_time = current_time;
  }
}

void LedConsensus::ans_mean(byte senderId) {
  if ((state >= 4 && state <= 7) || state <= 2)
    write(senderId, mean_ans, dAvg[nodeId - 1]);
  return;
}

void LedConsensus::rcv_mean(byte senderId, float value) {
  if (state != 4)
    return;

  if (!boolArray[senderId - 1]) {
    boolArray[senderId - 1] = true;
    nBool++;
    dAvg[senderId - 1] = value;
  }

  if (nBool == nNodes - 1) {
    state++;
    resetBool();
  }
}

void LedConsensus::ask_real_d() {
  unsigned long current_time = millis();

  if (first || current_time - last_time >= timeout) {
    for (byte i = 1; i <= nNodes; i++) {
      if (i != nodeId) {
        write(i, real_ask, dNodeOverall[nodeId - 1]);
      }
    }
    last_time = current_time;
  }
}

void LedConsensus::ans_real_d(byte senderId) {
  if (state >= 7 || state <= 2)
    write(senderId, real_ans, dNodeOverall[nodeId - 1]);
  return;
}

void LedConsensus::rcv_real_d(byte senderId, float value) {
  if (state != 7)
    return;

  if (!boolArray[senderId - 1]) {
    boolArray[senderId - 1] = true;
    nBool++;
    dNodeOverall[senderId - 1] = value;
  }

  if (nBool == nNodes - 1) {
    // FINISHED
    state = 0;
    resetBool();
  }
}

void LedConsensus::resetBool() {
  for (byte i = 0; i < nNodes; i++) {
    boolArray[i] = false;
  }
  nBool = 0;
  first = true;
}
