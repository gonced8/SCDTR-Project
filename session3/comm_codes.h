#ifndef COMM_CODES
#define COMM_CODES

const constexpr char hub_q[2] = {'h', 'q'};
const constexpr char hub_a[2]  = {'h', 'a'};

const constexpr char calibr_start[2] = {'C', 'i'};
const constexpr char calibr_wait[2] = {'C', 'w'};
const constexpr char calibr_answer[2] = {'C', 'a'};

const constexpr char sync_ask[2] = {'S', 'Q'};
const constexpr char sync_ans[2] = {'S', 'A'};

const constexpr char ctrl_send = 'T';
const constexpr char ctrl_recv = 'R';

const constexpr char duty_cycle_code = '0';

const constexpr char consensus_tell[2] = {'C', 'Q'};
const constexpr char consensus_rcv[2] = {'C', 'A'};


#endif //COM_CODES
