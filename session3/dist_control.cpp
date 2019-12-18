// Implementation of distributed control related functions

#include "dist_control.h"

/*-------Variable definition--------*/
// LDR calibration
const float m[3] = { -0.67, -0.72, -0.718}; // LDR calibration
float b[3] = {1.763, 1.763, 1.763}; // LDR calibration
float k[3] = {0.0, 0.0, 0.0}; // Gains

// Control related variables
float luxRefUnocc = 30;
float luxRefOcc = 70;
bool deskOccupancy = false;

// Optimization
const float infinity = 1.0 / 0.0;

// Actuation
float measuredLux = 0;

extern float u_pid;
extern float u;
extern PID pid;

/*--------Function definition--------*/
float getLux(int measurement) {
  float Vldr, Rldr, lux;

  measurement = map(measurement, 0, 1023, 0, 5000);
  Vldr = Vcc - measurement;
  Rldr = (float) Vldr * R1 / (Vcc - Vldr);
  lux = pow(10, (float) (log10(Rldr) - b[nodeId - 1]) / m[nodeId - 1]);

  return lux;
}

void LedConsensus::init(byte _nodeId, byte _nNodes, float _rho, byte _c_i) {
  nodeId = _nodeId;
  nNodes = _nNodes;
  // Consensus setup
  rho = _rho;
  c_i = _c_i;
  remainingIters = maxIters;
  for (int i = 0; i < nNodes; i++) {
    y[i] = 0;
    c[i] = 0;
    dNode[i] = 0;
    dNodeOverall[i] = 0;
    dAvg[i] = 0;
    dColumn[i] = 0;
  }
  c[nodeId - 1] = c_i;
  // Ref setup
  if (deskOccupancy)
    setLocalL(luxRefOcc);
  else
    setLocalL(luxRefUnocc);
  // Comms setup
  last_time = millis();

  state = 3;
  first = true;
}

bool LedConsensus::detectChanges() {
  return (abs(u_pid) > threshold);
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
  return (d[nodeId - 1] <= 100 + tol && d[nodeId - 1] >= 0 - tol && dotProd(k, d) >= L_i - o_i - tol);
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

float LedConsensus::getLocalD() {
  return dNodeOverall[nodeId - 1];
}

float LedConsensus::getMeasuredLux() {
  return measuredLux;
}

float LedConsensus::calcExpectedLux() {
  return dotProd(k, dNodeOverall);
}

void LedConsensus::calcNewO() {
  float new_o = getLux(analogRead(ldrPin)) - calcExpectedLux();
  Serial.print("New o inside consensus is "); Serial.println(new_o);
  o_i = new_o;
}

float LedConsensus::evaluateCost(float* local_d) {
  float local_cost = 0;
  for (byte i = 0; i < nNodes; i++)
    local_cost += y[i] * (local_d[i] - dAvg[i]) + (rho / 2) * (local_d[i] - dAvg[i]) * (local_d[i] - dAvg[i]);
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
    //Serial.println("Interior is feasible");
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
      //Serial.print("Solution 1 cost "); Serial.println(cost_temp);
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
      //Serial.print("Solution 2 cost "); Serial.println(cost_temp);
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
      //Serial.print("Solution 3 cost "); Serial.println(cost_temp);
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
      //Serial.print("Solution 4 cost "); Serial.println(cost_temp);
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
      //Serial.print("Solution 5 cost "); Serial.println(cost_temp);
      if (cost_temp < cost_best) {
        memcpy(d_best, d_temp, nNodes * sizeof(float));
        cost_best = cost_temp;
      }
    }
  }

  if (cost_best != infinity) {
    memcpy(dNode, d_best, nNodes * sizeof(float));
    dColumn[nodeId - 1] = d_best[nodeId - 1];
  }
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
    //Serial.print("y"); Serial.print(i); Serial.print(" is "); Serial.println(y[i]);
  }
}

void LedConsensus::resetConsensus() {
  for (int i = 0; i < nNodes; i++) {
    y[i] = 0;
    dAvg[i] = 0;
  }
  remainingIters = maxIters;
}

void LedConsensus::run() {
  //Serial.println(state);
  switch (state) {
    // See if need to start new consensus
    case 0:
      if (detectChanges())
        state++;
      break;

    // Tell others to start
    case 1:
      ask();
      break;

    // Receive d real
    case 2:
      pid.on = false;
      ask();
      break;

    // Measure lux
    case 3:
      dNodeOverall[nodeId - 1] = u;
      measuredLux = getLux(analogRead(ldrPin));
      Serial.print("Measured lux is "); Serial.println(measuredLux);
      //Serial.print("New o is "); Serial.println(aux);
      setLocalO(measuredLux - calcExpectedLux());
      resetConsensus();
      
      /*for (byte i = 1; i <= nNodes; i++) {
        Serial.print("Led "); Serial.print(i); Serial.print(" "); Serial.println(dNodeOverall[i - 1]);
        }*/
      state++;
      break;

    // Find minima
    case 4:
      findMinima();
      state++;
      break;

    // Receive duty cycles
    case 5:
      ask();
      break;

    // Compute average
    case 6:
      calcMeanVector();
      pid.on = true;
      state++;
      break;

    // Receive means
    case 7:
      ask();
      break;

    // Calculate y
    case 8:
      calcLagrangeMult();
      remainingIters--;
      //Serial.print("Remaining iterations: "); Serial.println(remainingIters);
      if (remainingIters > 0)
        state = 4;
      else {
        dNodeOverall[nodeId - 1] = dNode[nodeId - 1];
        pid.ip = 0;
        state++;
      }
      break;
  }
}
void LedConsensus::ask() {
  unsigned long current_time = millis();

  if (state == 5) {
    if (first || current_time - last_time >= timeout) {
      for (byte i = 1; i <= nNodes; i++) {
        if (i != nodeId && !boolArray[i - 1]) {
          write(i, duty_cycle_ask, dNode[i - 1]);
        }
      }
      last_time = current_time;
      first = false;
    }
  }
  else {
    char code;
    float value;
    switch (state) {
      case 1:
        code = start_ask;
        break;
      case 2:
        code = real_ask;
        value = u;  // REVISE
        break;
      case 7:
        code = mean_ask;
        value = dAvg[nodeId - 1];
        break;
    }

    if (first) {
      write(0, code, value);
      last_time = current_time;
      first = false;
    }
    else if (current_time - last_time >= timeout) {
      for (byte i = 1; i <= nNodes; i++) {
        if (i != nodeId && !boolArray[i - 1]) {
          write(i, code, value);
        }
      }
      last_time = current_time;
    }
  }
}

void LedConsensus::ans(byte senderId, char code) {
  bool valid = false;
  char ans_code;
  float value;

  switch (code) {
    case start_ask:
      valid = (state <= 2);
      ans_code = start_ans;
      break;
    case real_ask:
      valid = (state >= 2 && state <= 5);
      ans_code = real_ans;
      value = dNodeOverall[nodeId - 1];
      break;
    case duty_cycle_ask:
      valid = (state >= 5 && state <= 7);
      ans_code = duty_cycle_ans;
      value = dNode[senderId - 1];
      break;
    case mean_ask:
      valid = (state >= 7 || state <= 2);
      ans_code = mean_ans;
      value = dAvg[nodeId - 1];
      break;
  }

  if (valid) {
    write(senderId, ans_code, value);
    if (code == start_ask && state == 0) {
      state = 2;
      return;
    }
    rcv(senderId, ans_code, value);
  }
}

void LedConsensus::rcv(byte senderId, char code, float value) {
  if (!boolArray[senderId - 1]) {
    bool valid = false;
    float *variable;

    switch (code) {
      case start_ans:
        valid = (state == 1);
        break;
      case real_ans:
        valid = (state == 2);
        variable = &(dNodeOverall[senderId - 1]);
        break;
      case duty_cycle_ans:
        valid = (state == 5);
        variable = &(dColumn[senderId - 1]);
        break;
      case mean_ans:
        valid = (state == 7);
        variable = &(dAvg[senderId - 1]);
        break;
    }

    if (valid) {
      boolArray[senderId - 1] = true;
      nBool++;
      if (code != start_ans)
        *variable = value;

      if (nBool == nNodes - 1) {
        state = (state + 1) % 9;
        resetBool();
      }
    }
  }
}

void LedConsensus::resetBool() {
  for (byte i = 0; i < nNodes; i++) {
    boolArray[i] = false;
  }
  nBool = 0;
  first = true;
}
