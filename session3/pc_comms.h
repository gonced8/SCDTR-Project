// Hub identification and related functions

#ifndef PCCOMMS_H
#define PCCOMMS_H

#include <Arduino.h>
#include "can_comms.h"

class PcComms {
    /*-------Variable declaration-------*/
    // Node identification
    byte nodeId;
    byte nNodes;
    bool first;
    unsigned long last_time;
    unsigned long timeout = 500;

    bool received_occupancy;
    bool received_lower_bound_occupied;
    bool received_lower_bound_unoccupied;
    bool received_current_lower_bound;
    bool received_current_external;
    bool received_current_reference;
    bool received_current_cost;
    bool received_time_since_restart;
    bool received_set_occupied;
    bool received_set_occupied_value;
    bool received_set_unoccupied_value;
    bool received_set_cost;
    bool received_set_restart;

    float occupancy;
    float lower_bound_occupied;
    float lower_bound_unoccupied;
    float current_lower_bound;
    float current_external;
    float current_reference;
    float current_cost;
    float time_since_restart;
    float set_occupied;
    float set_occupied_value;
    float set_unoccupied_value;
    float set_cost;
    float set_restart;

    byte id_occupancy;
    byte id_lower_bound_occupied;
    byte id_lower_bound_unoccupied;
    byte id_current_lower_bound;
    byte id_current_external;
    byte id_current_reference;
    byte id_current_cost;
    byte id_time_since_restart;
    byte id_set_occupied;
    byte id_set_occupied_value;
    byte id_set_unoccupied_value;
    byte id_set_cost;
    byte id_set_restart;

  public:
    /*--------Function propotypes--------*/
    void init(byte _nodeId, byte _nNodes);
    void ask();
    void ans(byte senderId, char code);
    void rcv(byte senderId, char code, float value);
    void SerialDecode();
};

#endif // HUB_H
