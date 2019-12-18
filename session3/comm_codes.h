#ifndef COMM_CODES
#define COMM_CODES

const constexpr char hub_q = 1;
const constexpr char hub_a = 2;

const constexpr char calibr_start = 3;
const constexpr char calibr_wait = 4;
const constexpr char calibr_answer = 5;

const constexpr char sync_ask = 6;
const constexpr char sync_ans = 7;

const constexpr char ctrl_send = 8;
const constexpr char ctrl_recv = 9;

const constexpr char start_ask = 10;
const constexpr char start_ans = 11;
const constexpr char duty_cycle_ask = 12;
const constexpr char duty_cycle_ans = 13;
const constexpr char mean_ask = 14;
const constexpr char mean_ans = 15;
const constexpr char real_ask = 16;
const constexpr char real_ans = 17;

const constexpr char occupancy_ask = 18;
const constexpr char lower_bound_occupied_ask = 19;
const constexpr char lower_bound_unoccupied_ask = 20;
const constexpr char current_lower_bound_ask = 21;
const constexpr char current_external_ask = 22;
const constexpr char current_reference_ask = 23;
const constexpr char current_cost_ask = 24;
const constexpr char time_since_restart_ask = 25;
const constexpr char set_occupied_ask = 26;
const constexpr char set_occupied_value_ask = 27;
const constexpr char set_unoccupied_value_ask = 28;
const constexpr char set_cost_ask = 29;
const constexpr char set_restart_ask = 30;

const constexpr char occupancy_ans = 31;
const constexpr char lower_bound_occupied_ans = 32;
const constexpr char lower_bound_unoccupied_ans = 33;
const constexpr char current_lower_bound_ans = 34;
const constexpr char current_external_ans = 35;
const constexpr char current_reference_ans = 36;
const constexpr char current_cost_ans = 37;
const constexpr char time_since_restart_ans = 38;
const constexpr char set_occupied_ans = 39;
const constexpr char set_occupied_value_ans = 40;
const constexpr char set_unoccupied_value_ans = 41;
const constexpr char set_cost_ans = 42;
const constexpr char set_restart_ans = 43;

const constexpr char sample_duty_cyle = 44;
const constexpr char sample_lux = 45;

const constexpr char wait_ask = 46;
const constexpr char wait_ans = 47;

#endif //COM_CODES
