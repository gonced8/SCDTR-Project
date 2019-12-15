// Implementation of hub related functions

#include "pc_comms.h"

void PcComms::init(byte _nodeId, byte _nNodes) {
  nodeId = _nodeId;
  nNodes = _nNodes;
  first = true;

  received_occupancy = true;
  received_lower_bound_occupied = true;
  received_lower_bound_unoccupied = true;
  received_current_lower_bound = true;
  received_current_external = true;
  received_current_reference = true;
  received_current_cost = true;
  received_time_since_restart = true;
  received_set_occupied = true;
  received_set_occupied_value = true;
  received_set_unoccupied_value = true;
  received_set_cost = true;
  received_set_restart = true;

  bool occupancy = false;
  float lower_bound_occupied = 0;
  float lower_bound_unoccupied = 0;
  float current_lower_bound = 0;
  float current_external = 0;
  float current_reference = 0;
  float current_cost = 0;
  float time_since_restart = 0;
  float set_occupied = 0;
  float set_occupied_value = 0;
  float set_unoccupied_value = 0;
  float set_cost = 0;
  float set_restart = 0;
}

bool PcComms::getCurrentOccupancy(byte luminaire) {
  bool occupancy = 0;
  return occupancy;
}
float PcComms::getLowerBoundOccupied(byte luminaire) {
  float Lux = 0;
  return Lux;
}
float PcComms::getLowerBoundUnoccupied(byte luminaire) {
  float Lux = 0;
  return Lux;
}
float PcComms::getCurrentLowerBound(byte luminaire) {
  float Lux = 0;
  return Lux;
}
float PcComms::getCurrentExternal(byte luminaire) {
  float Lux = 0;
  return Lux;
}
float PcComms::getCurrentReference(byte luminaire) {
  float PWM = 0;
  return PWM;
}
float PcComms::getCurrentCost(byte luminaire) {
  float cost = 0;
  return cost;
}
float PcComms::getTimeSinceRestart(byte luminaire) {
  float time = 0;
  return time;
}
bool PcComms::setOccupied(byte luminaire, byte state) {
  return true;
}
bool PcComms::setOccupiedValue(byte luminaire, float Lux) {
  return true;
}
bool PcComms::setUnoccupiedValue(byte luminaire, float Lux) {
  return true;
}
bool PcComms::setCost(byte luminaire, float cost) {
  return true;
}

void askPcComm(byte to, char code) {
  switch (code) {
    case occupancy_ask:
      break;
    case lower_bound_occupied_ask:
      break;
    case lower_bound_unoccupied_ask:
      break;
    case current_lower_bound_ask:
      break;
    case current_external_ask:
      break;
    case current_reference_ask:
      break;
    case current_cost_ask:
      break;
    case time_since_restart_ask:
      break;
    case set_occupied_ask:
      break;
    case set_occupied_value_ask:
      break;
    case set_unoccupied_value_ask:
      break;
    case set_cost_ask:
      break;
    case set_restart_ask:
      break;
  }
}

void PcComms::SerialDecode() {
  String message = Serial.readString();
  byte luminaire = message.toInt();
  byte messagenum = (byte)message.substring(2).toInt();
  float multifunfloat;

  switch (messagenum) {
    case 3:
      received_occupancy = false;
      //Serial.print("! o "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunbyte);
      break;

    case 4:
      //Serial.print("! O "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 5:
      //Serial.print("! U "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 6:
      //Serial.print("! L "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 7:
      //Serial.print("! x "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 8:
      //Serial.print("! r "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 9:
      //Serial.print("! c "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 11:
      //Serial.print("! t "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case 15:
      multifunfloat = message.substring(4).toFloat();
      //Serial.println("! ack");
      break;

    case 16:
      multifunfloat = message.substring(4).toFloat();
      //Serial.println("! ack");
      break;

    case 17:
      multifunfloat = message.substring(4).toFloat();
      //Serial.println("! ack");
      break;

    case 18:
      multifunfloat = message.substring(4).toFloat();
      //Serial.println("! ack");
      break;

    case 19:
      //Serial.println("! ack");
      break;

  }

  Serial.write('!');
  Serial.write(messagenum);
  Serial.write(luminaire);
  Serial.write((char*)&multifunfloat, 4);
  Serial.write('\n');
}
