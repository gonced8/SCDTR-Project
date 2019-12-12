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

void LedConsensus::rcvStart(byte senderId) {
  write(senderId, consensus_rcv, 0);
  if (finished())
    startCounter();
  Serial.print("Consensus: Answered node "); Serial.println(senderId);
}

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

float LedConsensus::f_iCalc(float* d) {

  if (d[nodeId - 1] <= 100 && d[nodeId - 1] >= 0 && dotProd(k, d) >= L_i - o_i)
    return c_i * d[nodeId - 1];
  else
    return infinity;
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

bool LedConsensus::finished() {
  return remainingIters == 0;
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

bool LedConsensus::findMinima() {
  // OUTPUTS:
  // dNode: array with local optimal led duty cycles (note: only need the own node duty cycle)
  // bool: true if feasible, false if unfeasible
  float newd[5];
  float zi[5];
  float knorm = dotProd(k, k);

  // First we will try to find the solution in the interior
  for (byte i = 0; i < nNodes; i++) {
    if (i != nodeId - 1) //Not our node
      newd[i] = dAvg[i] - y[i] / rho;
    else
      newd[i] = dAvg[i] - y[i] / rho - c_i / rho;
  }
  // Now we check if this first solution is feasible
  if (f_iCalc(newd) != infinity) { // Solution is feasible
    memcpy(dNode, newd, nNodes * sizeof(float));
    dColumn[nodeId - 1] = newd[nodeId - 1];
    Serial.println("Interior is feasible");
    return true;
  }

  // Else continue looking for solutions on the borders
  float newd2[5];
  float newd3[5];
  float newd4[5];
  float newd5[5];
  float* dvec_pointer[] = {newd, newd2, newd3, newd4, newd5};
  bool feasible[5] = {false, false, false, false, false};
  //TODO: optimize the solutions generation
  //      there are factors in common among different solutions, make only one loop

  // Solution 1
  ziCalc(zi);
  float aux_cte = (1 / knorm) * (o_i - L_i + (1 / rho) * dotProd(k, zi));
  for (byte i = 0; i < nNodes; i++)
    newd[i] = (1 / rho) * zi[i] - k[i] * aux_cte;
  if (f_iCalc(newd) != infinity) // Solution is feasible
    feasible[0] = true;

  // Solution 2
  for (byte i = 0; i < nNodes; i++) {
    if (i != nodeId - 1)
      newd2[i] = zi[i] / rho;
    else
      newd2[i] = 0;
  }
  if (f_iCalc(newd2) != infinity) // Solution is feasible
    feasible[1] = true;

  // Solution 3
  for (byte i = 0; i < nNodes; i++) {
    if (i != nodeId - 1)
      newd3[i] = zi[i] / rho;
    else
      newd3[i] = 100;
  }
  if (f_iCalc(newd3) != infinity) // Solution is feasible
    feasible[2] = true;

  //Solution 4
  for (byte i = 0; i < nNodes; i++) {
    if (i != nodeId - 1)
      newd4[i] = zi[i] / rho - (k[i] / (knorm - k[nodeId - 1] * k[nodeId - 1])) * (o_i - L_i) + (1 / rho) * (k[i] / (knorm - k[nodeId - 1] * k[nodeId - 1])) * (-dotProd(k, zi) + k[nodeId - 1] * zi[nodeId - 1]);
    else
      newd4[i] = 0;
  }
  if (f_iCalc(newd4) != infinity) // Solution is feasible
    feasible[3] = true;

  //Solution 5
  for (byte i = 0; i < nNodes; i++) {
    if (i != nodeId - 1)
      newd5[i] = newd4[i] - (100 * k[i] * k[nodeId - 1]) / (knorm - k[nodeId - 1] * k[nodeId - 1]);
    else
      newd5[i] = 100;
  }
  if (f_iCalc(newd5) != infinity) // Solution is feasible
    feasible[4] = true;

  // See the minimum cost between the feasible possibilities
  float lagrangean_aux = 0;
  float ft = infinity;
  float aux_vec[maxNodes];
  for (byte i = 0; i < 5; i++) {  // this 5 is fixed
    Serial.print("Feasible "); Serial.print(i); Serial.print(" is "); Serial.println(feasible[i]);
    if (feasible[i]) {
      for (byte j = 0; j < nNodes; j++)
        aux_vec[j] = dvec_pointer[i][j] - dAvg[j];
      lagrangean_aux = dvec_pointer[i][nodeId - 1] * c_i + dotProd(aux_vec, y) + (rho / 2) * dotProd(aux_vec, aux_vec);
      Serial.print("Cost for "); Serial.print(i); Serial.print(" is "); Serial.println(lagrangean_aux);
      if (lagrangean_aux <= ft) {
        ft = lagrangean_aux;
        memcpy(dNode, dvec_pointer[i], nNodes * sizeof(float));
        Serial.print("Changed optimal to "); Serial.println(i);
      }
    }
  }

  if (ft == infinity)
    return false;    // Unfeasible
  return true;
}


void LedConsensus::calcMeanVector() {
  dColumn[nodeId - 1] = dNode[nodeId-1];
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
  Serial.println(state);

  switch (state) {
    // Find minima
    case 0:
      findMinima();
      state++;
      break;

    // Receive duty cycles
    case 1:
      ask_duty_cycles();
      break;

    // Compute average
    case 2:
      calcMeanVector();
      state++;
      break;

    // Receive means
    case 3:
      ask_mean();
      break;

    // Calculate y
    case 4:
      calcLagrangeMult();
      state++;
      break;

    // Update led duty cycle
    case 5:
      dNodeOverall[nodeId - 1] = dNode[nodeId - 1];
      analogWrite(ledPin, dNodeOverall[nodeId - 1] * 255.0 / 100);
      state++;
      break;

    // Receive d real
    case 6:
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
  if (state >= 1 && state <= 3)
    write(senderId, duty_cycle_ans, dNode[senderId - 1]);
}

void LedConsensus::rcv_duty_cycles(byte senderId, float value) {
  if (state != 1)
    return;

  if (!boolArray[senderId - 1]) {
    boolArray[senderId - 1] = true;
    nBool++;
    dColumn[senderId - 1] = value;
  }

  if (nBool == nNodes - 1) {
    state++;
    for (byte i = 0; i < nNodes; i++) {
      boolArray[i] = false;
    }
    nBool = 0;
  }
}

void LedConsensus::ask_mean() {
  unsigned long current_time = millis();

  if (first || current_time - last_time >= timeout) {
    for (byte i = 1; i <= nNodes; i++) {
      if (i != nodeId) {
        write(i, mean_ask, dAvg[i - 1]);
      }
    }
    last_time = current_time;
  }
}

void LedConsensus::ans_mean(byte senderId) {
  if (state >= 3 && state <= 6)
    write(senderId, mean_ans, dAvg[nodeId - 1]);
  return;
}

void LedConsensus::rcv_mean(byte senderId, float value) {
  if (state != 3)
    return;

  if (!boolArray[senderId - 1]) {
    boolArray[senderId - 1] = true;
    nBool++;
    dAvg[senderId - 1] = value;
  }

  if (nBool == nNodes - 1) {
    state++;
    for (byte i = 0; i < nNodes; i++) {
      boolArray[i] = false;
    }
    nBool = 0;
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
  if (state >= 6 || state <= 1)
    write(senderId, real_ans, dNodeOverall[nodeId - 1]);
  return;
}

void LedConsensus::rcv_real_d(byte senderId, float value) {
  if (state != 6)
    return;

  if (!boolArray[senderId - 1]) {
    boolArray[senderId - 1] = true;
    nBool++;
    dNodeOverall[senderId - 1] = value;
  }

  if (nBool == nNodes - 1) {
    // FINISHED
    state = 0;
    remainingIters--;

    for (byte i = 0; i < nNodes; i++) {
      boolArray[i] = false;
    }
    nBool = 0;
  }
}
