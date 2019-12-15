// Hub identification and related functions

#ifndef PCCOMMS_H
#define PCCOMMS_H

#include <Arduino.h>
#include "comm_codes.h"

class PcComms {
    /*-------Variable declaration-------*/
    // Node identification
    byte nodeId;
    byte nNodes;
    bool first;
    
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
    
    bool occupancy;
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

  public:
    /*--------Function propotypes--------*/
    void init(byte _nodeId, byte _nNodes);
    bool getCurrentOccupancy(byte luminaire);
    float getLowerBoundOccupied(byte luminaire);
    float getLowerBoundUnoccupied(byte luminaire);
    float getCurrentLowerBound(byte luminaire);
    float getCurrentExternal(byte luminaire);
    float getCurrentReference(byte luminaire);
    float getCurrentCost(byte luminaire);
    float getTimeSinceRestart(byte luminaire);
    bool setOccupied(byte luminaire, byte state);
    bool setOccupiedValue(byte luminaire, float Lux);
    bool setUnoccupiedValue(byte luminaire, float Lux);
    bool setCost(byte luminaire, float cost);
    bool setRestart();
    void SerialDecode();
};

#endif // HUB_H
