// Implementation of distributed control related functions

#include <Arduino.h>
#include "dist_control.h"

/*-------Variable definition--------*/
// LDR calibration
const float m[5] = { -0.67, -0.67, -0.67, 0, 0}; // LDR calibration
const float b[5] = {1.763, 1.763, 1.763, 0, 0}; // LDR calibration
float k[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Gains

// Control related variables
float luxRefUnocc = 20;
float luxRefOcc = 50;
bool deskOccupancy = false;

// Optimization
const float infinity = 1.0 / 0.0;

/*--------Function definition--------*/
float getLux(int measurement) {

  float Vldr, Rldr, lux;

  measurement = map(measurement, 0, 1023, 0, 5000);
  Vldr = Vcc - measurement;
  Rldr = (float) Vldr * R1 / (Vcc - Vldr);
  lux = pow(10, (float) (log10(Rldr) - b[nodeId - 1]) / m[nodeId - 1]);

  return lux;
}

void LedConsensus::init(byte _nodeId, byte _nNodes, float _rho, byte _c_i, float* new_y) {
  nodeId = _nodeId;
  nNodes = _nNodes;
  rho = _rho;
  c_i = _c_i;
  c[nodeId - 1] = c_i;
  memcpy(y, new_y, sizeof(y));
  firstPart = true;
  received = 0;
  for (int i = 0; i < nNodes; i++) {
    for (int j = 0; j < nNodes; j++) {
      dMat[i][j] = 0;
    }
  }
  if (deskOccupancy)
    L_i = luxRefOcc;
  else
    L_i = luxRefUnocc;
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

  if (d[nodeId - 1] <= 100 && d[nodeId - 1] >= 0 && getLocalCost(d) >= L_i - o_i)
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

float LedConsensus::getLocalCost(float* d) {
  float aux_sum = 0;

  for (byte i = 0; i < nNodes; i++)
    aux_sum = aux_sum + k[i] * d[i];

  return aux_sum;
}

void LedConsensus::getLocalD(float* d) {
  memcpy(d, dNode, nNodes * sizeof(float));
}

void LedConsensus::calcNewO() {
  int measurement = analogRead(ldrPin);
  float measuredLux = getLux(measurement);
  float expectedLux = dotProd(k, dNodep);
  float new_o = measuredLux - expectedLux;
  Serial.print("New o is "); Serial.println(new_o);
  setLocalO(new_o);
}

bool LedConsensus::findMinima() {
  // OUTPUTS:
  // dNode: array with local optimal led duty cycles (note: only need the own node duty cycle)
  // bool: true if feasible, false if unfeasible
  float newd[5];
  float zi[5];
  float knorm = 0; // TODO: put this constant global and calculated after each calibration

  for (byte i = 0; i < nNodes; i++)
    knorm = knorm + k[i] * k[i];

  // First we will try to find the solution in the interior
  for (byte i = 0; i < nNodes; i++) {
    if (i != nodeId - 1) //Not our node
      newd[i] = dAvg[i] - y[i] / rho;
    else
      newd[i] = dAvg[i] - y[i] / rho - c_i / rho;
  }
  // Now we check if this first solution is feasible
  if (f_iCalc(newd) != infinity) { // Solution is feasible
    memcpy(dNodep, dNode, nNodes * sizeof(float));
    memcpy(dNode, newd, nNodes * sizeof(float));
    memcpy(dMat[nodeId - 1], dNode, nNodes * sizeof(float));
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
      //newd5[i-1] = zi[i-1]/rho - (k[i-1]*(o_i - L_i) + 100*k[i-1]*k[nodeId-1])/(knorm - k[nodeId -1]*k[nodeId -1]) - (1/rho)*(k[i-1]/(knorm - k[nodeId -1]*k[nodeId -1]))*(-dotProd(k, zi) + k[nodeId-1]*zi[nodeId-1]);
      newd5[i] = newd4[i] - (100 * k[i] * k[nodeId - 1]) / (knorm - k[nodeId - 1] * k[nodeId - 1]);
    else
      newd5[i] = 100;
  }
  if (f_iCalc(newd5) != infinity) // Solution is feasible
    feasible[4] = true;

  // See the minimum cost between the feasible possibilities
  float lagrangean_aux = 0;
  float ft = 0;
  bool first = true;
  for (byte i = 0; i < 5; i++) {  // this 5 is fixed
    if (feasible[i]) {
      lagrangean_aux = (rho / 2) * dotProd(dvec_pointer[i], dvec_pointer[i]) - dotProd(dvec_pointer[i], zi);
      if (first || lagrangean_aux < ft) {
        ft = lagrangean_aux;
        memcpy(dNodep, dNode, nNodes * sizeof(float));
        memcpy(dNode, dvec_pointer[i], nNodes * sizeof(float));
        memcpy(dMat[nodeId - 1], dNode, nNodes * sizeof(float));
        first = false;
      }
    }
  }

  if (first)
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

  Serial.print("Sent ledConsensus ");
  for (int i = 0; i < nNodes; i++) {
    encodeMessage(msg, duty_cycle_code + i, 0, dMat[nodeId - 1][i]);
    Serial.print(dMat[nodeId - 1][i]); Serial.print(" ");
    write(0, 0, msg);
  }
  Serial.println();
}

void LedConsensus::receive_duty_cycle(can_frame frame) {
  byte senderId = (frame.can_id >> shiftId) & mask;

  byte index = (byte)(frame.data[0] - duty_cycle_code);
  float value;
  memcpy(&value, frame.data + 2, sizeof(float));

  dMat[senderId - 1][index] = value;
  Serial.print("Received "); Serial.print(senderId); Serial.print(" "); Serial.print(index); Serial.println(value);

  received++;
}

void LedConsensus::run() {
  unsigned long current_time = millis();

  if (firstPart) {
    findMinima();
    send_duty_cycle();
    firstPart = false;
    last_time = current_time;
  }
  else if ((received >= (nNodes - 1)*nNodes) || (current_time - last_time >= timeout)) {
    // received -= (nNodes-1)*nNodes;
    received = 0;   // IMPROVE
    //------ Does synhronization, fills the matrix dMat
    // maybe do the LDR sampling after this synchronization??
    Serial.println("Entered 2nd part");
    calcNewO();
    calcMeanVector();
    calcLagrangeMult();
    analogWrite(ledPin, dNode[nodeId - 1] * 255.0 / 100);
    Serial.print("Wrote ledConsensus "); Serial.println(dNode[nodeId - 1]);
    // At this point, have local duty cycle reference at d_node[nodeId - 1]
    firstPart = true;
  }
}
