// Implementation of hub related functions

#include "pc_comms.h"

void PcComms::init(byte _nodeId, byte _nNodes) {
  nodeId = _nodeId;
  nNodes = _nNodes;
  first = true;
  last_time = millis();

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

  occupancy = 0;
  lower_bound_occupied = 0;
  lower_bound_unoccupied = 0;
  current_lower_bound = 0;
  current_external = 0;
  current_reference = 0;
  current_cost = 0;
  time_since_restart = 0;
  set_occupied = 0;
  set_occupied_value = 0;
  set_unoccupied_value = 0;
  set_cost = 0;
  set_restart = 0;

  id_occupancy = 0;
  id_lower_bound_occupied = 0;
  id_lower_bound_unoccupied = 0;
  id_current_lower_bound = 0;
  id_current_external = 0;
  id_current_reference = 0;
  id_current_cost = 0;
  id_time_since_restart = 0;
  id_set_occupied = 0;
  id_set_occupied_value = 0;
  id_set_unoccupied_value = 0;
  id_set_cost = 0;
  id_set_restart = 0;
}

void PcComms::ask() {
  unsigned long current_time = millis();

  if (first || current_time - last_time > timeout) {
    if (!received_occupancy)
      write(id_occupancy, occupancy_ask, 0);

    if (!received_lower_bound_occupied)
      write(id_lower_bound_occupied, lower_bound_occupied_ask, 0);

    if (!received_lower_bound_unoccupied)
      write(id_lower_bound_unoccupied, lower_bound_unoccupied_ask, 0);

    if (!received_current_lower_bound)
      write(id_current_lower_bound, current_lower_bound_ask, 0);

    if (!received_current_external)
      write(id_current_external, current_external_ask, 0);

    if (!received_current_reference)
      write(id_current_reference, current_reference_ask, 0);

    if (!received_current_cost)
      write(id_current_cost, current_cost_ask, 0);

    if (!received_time_since_restart)
      write(id_time_since_restart, time_since_restart_ask, 0);

    if (!received_set_occupied)
      write(id_set_occupied, set_occupied_ask, set_occupied);

    if (!received_set_occupied_value)
      write(id_set_occupied_value, set_occupied_value_ask, set_occupied_value);

    if (!received_set_unoccupied_value)
      write(id_set_unoccupied_value, set_unoccupied_value_ask, set_unoccupied_value);

    if (!received_set_cost)
      write(id_set_cost, set_cost_ask, set_cost);

    if (!received_set_restart)
      write(id_set_restart, set_restart_ask, set_restart);

    first = false;
    last_time = current_time;
  }
}

void PcComms::ans(byte senderId, char code, float value) {
  switch (code) {
    case occupancy_ask:
      write(senderId, occupancy_ans, (float) deskOccupancy);
      break;
    case lower_bound_occupied_ask:
      write(senderId, lower_bound_occupied_ans, luxRefOcc);
      break;
    case lower_bound_unoccupied_ask:
      write(senderId, lower_bound_unoccupied_ans, luxRefUnocc);
      break;
    case current_lower_bound_ask:
      write(senderId, current_lower_bound_ans, ledConsensus.L_i);
      break;
    case current_external_ask:
      write(senderId, current_external_ans, ledConsensus.o_i);
      break;
    case current_reference_ask:
      write(senderId, current_reference_ans, ledConsensus.dNodeOverall[nodeId - 1]);
      break;
    case current_cost_ask:
      write(senderId, current_cost_ans, ledConsensus.c_i);
      break;
    case time_since_restart_ask:
      write(senderId, time_since_restart_ans, millis() / 1000.0);
      break;
    case set_occupied_ask:
      write(senderId, set_occupied_ans, 0);
      deskOccupancy = value == 1;
      if (deskOccupancy)
        ledConsensus.L_i = luxRefOcc;
      else
        ledConsensus.L_i = luxRefUnocc;
      break;
    case set_occupied_value_ask:
      write(senderId, set_occupied_value_ans, 0);
      luxRefOcc = value;
      if (deskOccupancy)
        ledConsensus.L_i = luxRefOcc;
      break;
    case set_unoccupied_value_ask:
      write(senderId, set_unoccupied_value_ans, 0);
      luxRefUnocc = value;
      if (!deskOccupancy)
        ledConsensus.L_i = luxRefUnocc;
      break;
    case set_cost_ask:
      write(senderId, set_cost_ans, 0);
      ledConsensus.c_i = value;
      break;
    case set_restart_ask:
      write(senderId, set_restart_ans, 0);
      break;
  }
}

void PcComms::rcv(byte senderId, char code, float value) {
  char message[8];

  switch (code) {
    case occupancy_ans:
      if (!received_occupancy) {
        received_occupancy = true;
        occupancy = value;
      }
      break;
    case lower_bound_occupied_ans:
      if (!received_lower_bound_occupied) {
        received_lower_bound_occupied = true;
        lower_bound_occupied = value;
      }
      break;
    case lower_bound_unoccupied_ans:
      if (!received_lower_bound_unoccupied) {
        received_lower_bound_unoccupied = true;
        lower_bound_unoccupied = value;
      }
      break;
    case current_lower_bound_ans:
      if (!received_current_lower_bound) {
        received_current_lower_bound = true;
        current_lower_bound = value;
      }
      break;
    case current_external_ans:
      if (!received_current_external) {
        received_current_external = true;
        current_external = value;
      }
      break;
    case current_reference_ans:
      if (!received_current_reference) {
        received_current_reference = true;
        current_reference = value;
      }
      break;
    case current_cost_ans:
      if (!received_current_cost) {
        received_current_cost = true;
        current_cost = value;
      }
      break;
    case time_since_restart_ans:
      if (!received_time_since_restart) {
        received_time_since_restart = true;
        time_since_restart = value;
      }
      break;
    case set_occupied_ans:
      if (!received_set_occupied) {
        received_set_occupied = true;
      }
      break;
    case set_occupied_value_ans:
      if (!received_set_occupied_value) {
        received_set_occupied_value = true;
      }
      break;
    case set_unoccupied_value_ans:
      if (!received_set_unoccupied_value) {
        received_set_unoccupied_value = true;
      }
      break;
    case set_cost_ans:
      if (!received_set_cost) {
        received_set_cost = true;
      }
      break;
    case set_restart_ans:
      if (!received_set_restart) {
        received_set_restart = true;
      }
      break;
  }

  message[0] = '!';
  message[1] = code;
  message[2] = (char) senderId;
  memcpy(message + 3, &value, sizeof(float));
  message[7] = '\n';
  Serial.write(message, 8);
}

void PcComms::SerialDecode() {
  char message[6];
  Serial.print("Bytes: ");
  Serial.println(Serial.readBytes(message, 6));
  byte luminaire = (byte) message[0];       // CORRECT THIS IN ASIO
  char code = message[1];


  Serial.print("Luminaire ");
  Serial.println(luminaire);
  Serial.print("Code ");
  Serial.println((byte)code);

  switch (code) {
    case occupancy_ask:
      received_occupancy = false;
      id_occupancy = luminaire;
      //Serial.print("! o "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunbyte);
      break;

    case lower_bound_occupied_ask:
      received_lower_bound_occupied = false;
      id_lower_bound_occupied = luminaire;
      //Serial.print("! O "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case lower_bound_unoccupied_ask:
      received_lower_bound_unoccupied = false;
      id_lower_bound_unoccupied = luminaire;
      //Serial.print("! U "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case current_lower_bound_ask:
      received_current_lower_bound = false;
      id_current_lower_bound = luminaire;
      //Serial.print("! L "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case current_external_ask:
      received_current_external = false;
      id_current_external = luminaire;
      //Serial.print("! x "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case current_reference_ask:
      received_current_reference = false;
      id_current_reference = luminaire;
      //Serial.print("! r "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case current_cost_ask:
      received_current_cost = false;
      id_current_cost = luminaire;
      //Serial.print("! c "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case time_since_restart_ask:
      received_time_since_restart = false;
      id_time_since_restart = luminaire;
      //Serial.print("! t "); Serial.print(luminaire); Serial.print(" "); Serial.println(multifunfloat);
      break;

    case set_occupied_ask:
      received_set_occupied = false;
      id_set_occupied = luminaire;
      set_occupied = *(float *)(message+2);
      //Serial.println("! ack");
      break;

    case set_occupied_value_ask:
      received_set_occupied_value = false;
      id_set_occupied_value = luminaire;
      set_occupied_value = *(float *)(message+2);
      //Serial.println("! ack");
      break;

    case set_unoccupied_value_ask:
      received_set_unoccupied_value = false;
      id_set_unoccupied_value = luminaire;
      set_unoccupied_value = *(float *)(message+2);
      Serial.print("Float ");
      Serial.println(set_unoccupied_value);
      //Serial.println("! ack");
      break;

    case set_cost_ask:
      received_set_cost = false;
      id_set_cost = luminaire;
      set_cost = *(float *)(message+2);
      //Serial.println("! ack");
      break;

    case set_restart_ask:
      received_set_restart = false;
      id_set_restart = luminaire;
      set_restart = *(float *)(message+2);
      //Serial.println("! ack");
      break;
  }
  first = true;
}
