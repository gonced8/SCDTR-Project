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

const constexpr char consensus_tell = 10;
const constexpr char consensus_rcv = 11;

// Always the last code
const constexpr char duty_cycle_ask = 12;
const constexpr char duty_cycle_code = 13;


#endif //COM_CODES
