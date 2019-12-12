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
  if (abs(new_o - ledConsensus.getLocalO()) > 2) {
    ledConsensus.setLocalO(new_o);
    ledConsensus.startCounter();
    ledConsensus.tellOthers();
    Serial.println("Will enter consensus again");
  }
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
    for (int j = 0; j < nNodes; j++) {
      dMat[i][j] = 0;
      boolMat[i][j] = false;
    }
    c[i] = 0;
    dNode[i] = 0;
    dNodeOverall[i] = 0;
    dAvg [i] = 0;
    handshakes[i] = false;
    received = 0;
    //nodesReceived[i] = false;
    //nodesAck[i] = false;
  }
  c[nodeId - 1] = c_i;
  //allReceived = 0;
  //allAck = 0;
  // Ref setup
  if (deskOccupancy)
    setLocalL(luxRefOcc);
  else
    setLocalL(luxRefUnocc);
  // Comms setup
  waiting = false;
  first = true;
  nHand = 0;
  last_time = millis();
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
    memcpy(dMat[nodeId - 1], dNode, nNodes * sizeof(float));
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
        memcpy(dMat[nodeId - 1], dNode, nNodes * sizeof(float));
        Serial.print("Changed optimal to "); Serial.println(i);
      }
    }
  }

  if (ft == infinity)
    return false;    // Unfeasible
  return true;
}

void LedConsensus::calcMeanVector() {
  float aux = 0;
  for (byte j = 0; j < nNodes; j++) {
    for (byte i = 0; i < nNodes; i++) {
      aux = aux + dMat[i][j];
    }
    dAvg[j] = aux / nNodes;
    aux = 0;
  }
}

void LedConsensus::calcLagrangeMult() {
  for (byte i = 0; i < nNodes; i++) {
    y[i] = y[i] + rho * (dNode[i] - dAvg[i]);
  }
}

void LedConsensus::send_duty_cycle() {
  uint8_t msg[data_bytes];

  //if (firstPart) {
  for (byte i = 0; i < nNodes; i++) {
    write(0, duty_cycle_code + i, dMat[nodeId - 1][i]);
  }
  /*}
    else {
    for (byte n = 0; n < nNodes; n++) {
      if (!nodesAck[n] && n != nodeId) {
        for (byte i = 0; i < nNodes; i++) {
          write(n, duty_cycle_code + i, dMat[nodeId - 1][i]);
        }
      }
    }
    }*/
}

/*void LedConsensus::receive_ack(byte senderId) {
  if (!nodesAck[senderId - 1]) {
    nodesAck[senderId - 1] = true;
    allAck++;
  }
  }*/

void LedConsensus::receive_duty_cycle(byte senderId, char code, float value) {
  byte index = (byte)(code - duty_cycle_code);
  dMat[senderId - 1][index] = value;

  if (!boolMat[senderId - 1][index]) {
    boolMat[senderId - 1][index] = true;
    received++;

    /*if (received[senderId - 1] == nNodes) {
      allReceived++;
      nodesReceived[senderId - 1] = true;
      }*/
  }

  /*if (nodesReceived[senderId - 1]) {
    write(senderId, duty_cycle_ack, 0);
    }*/
}

void LedConsensus::calcOverallDC() {
  for (byte i = 0; i < nNodes; i++)
    dNodeOverall[i] = dMat[i][i];
}

void LedConsensus::run() {
  if (waiting) {
    tellStart();
  }
  else {
    unsigned long curr_time = millis();
    if (firstPart) {
      Serial.println("firstPart");
      findMinima();
      send_duty_cycle();
      firstPart = false;
      last_time = curr_time;
    }
    else if (received == (nNodes - 1)*nNodes || (curr_time - last_time > timeout)) {
      Serial.println("allReceived");
      firstPart = true;
      calcMeanVector();
      calcLagrangeMult();
      //calcNewO();
      //calcOverallDC();
      //dutyCycle = dNode[nodeId - 1];
      if (remainingIters == 1) {  // last iteration, update duty cycle
        Serial.println("Updated dutyCycle at last iteration");
        calcOverallDC();
        dutyCycle = dNode[nodeId - 1];
      }
      remainingIters--;

      for (byte i = 0; i < nNodes; i++) {
        for (byte j = 0; j < nNodes; j++) {
          boolMat[i][j] = false;
        }
      }
      received = 0;
      /*nodesReceived[i] = false;
        nodesAck[i] = false;*/
    }
    /*allReceived = 0;
      allAck = 0;*/
    //}
    /*else if (current_time - last_time >= timeout) {
      Serial.println("Timeout");
      send_duty_cycle();
      last_time = current_time;
      }*/
  }
}
