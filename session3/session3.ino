#include <SPI.h>
#include <mcp2515.h>

#include "can_comms.h"
#include "dist_control.h"
#include "pc_comms.h"
#include "calibr.h"
#include "sync.h"
#include "PID.h"

/*----------------- Variables -----------------*/
can_frame frame;
byte senderId;
char code;
float value;
Calibration calibrator;
Sync sync;
LedConsensus ledConsensus;
PID pid;
PcComms pcComms;

/* Interruption flags */
volatile boolean sampFlag = false;

byte nodeId;
byte nNodes = 3;
#define ID0 7
#define ID1 8

bool saturateInt = false;

float dutyCycles[maxNodes];
float luxs[maxNodes];

extern bool saturateInt;
float u_pid = 0;
float u_con = 0;
float u;

#define TOTAL_MESSAGES 10000
unsigned int nMessages = 0;
unsigned int timeout = 5; // [us] = 5 ms
bool first = true;
unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long first_time = 0;
unsigned long time_sum = 0;
float mean_time;


/*----------- Function Definitions ------------*/
void handleInterrupt();
void getNodeId();
void timerIntConfig();

ISR(TIMER1_COMPA_vect) {
  sampFlag = true;
}
/*---------------------------------------------*/

void setup() {
  Serial.begin(115200);
  digitalWrite(4, HIGH); // We need an extra VCC
  getNodeId();

  // MCP2515 Configuration
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  setMasksFilters();

  // MCP2515 Mode Set
  mcp2515.setNormalMode();

  sync.init(nodeId, nNodes);
  calibrator.init(nodeId, nNodes);
  ledConsensus.init(nodeId, nNodes, 0.1, 1);
  pid.init(1, 0.1, 0, 0.01, 0.1);  // 10, 1 was initial when it worked but with overshoot
  pcComms.init(nodeId, nNodes);

  timerIntConfig();

  Serial.println("Setup done");
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {
  //Serial.print("interrupt "); Serial.println(interrupt);
  if (interrupt) { // there are new messages
    interrupt = false;

    if (mcp2515_overflow) {
      Serial.println("\t\tError: MCP2516 RX Buffers Overflow");
      mcp2515_overflow = false;
    }

    if (arduino_overflow) {
      Serial.println("\t\tError: Arduino Stream Buffers Overflow");
      arduino_overflow = false;
    }

    while ( cf_stream.get(frame) ) {
      decodeMessage(frame, senderId, code, value);

      switch (code) {
        case 50:
          write(senderId, 51, value);
          break;

        case 51:
          if (value == nMessages) {
            time_sum += (micros() - first_time);
            nMessages++;
            first = true;
          }
          break;

        // Synchronize
        case sync_ask:
          sync.answer_node(senderId);
          break;

        case sync_ans:
          sync.receive_answer(senderId);
          break;
      }
    }
  }

  if (sync.isOn())
    sync.ask_node();

  else if (nodeId == 1 && sampFlag) {
    sampFlag = false;

    if (nMessages < TOTAL_MESSAGES) {
      current_time = micros();
      if (first || (current_time - last_time > timeout)) { // send message
        write(2, 50, nMessages);
        last_time = current_time;
        if (first) {
          first_time = current_time;
        }
      }
    }
    else {
      mean_time = ((float)time_sum) / TOTAL_MESSAGES;
      Serial.print("Mean time for "); Serial.print(TOTAL_MESSAGES); Serial.print(" was "); Serial.print(mean_time); Serial.println(" us");
    }
  }

  //delay(1);
}

void handleNewMessages() {
  interrupt = false;

  if (mcp2515_overflow) {
    Serial.println("\t\tError: MCP2516 RX Buffers Overflow");
    mcp2515_overflow = false;
  }

  if (arduino_overflow) {
    Serial.println("\t\tError: Arduino Stream Buffers Overflow");
    arduino_overflow = false;
  }

  while ( cf_stream.get(frame) ) {
    decodeMessage(frame, senderId, code, value);

    /*
      Serial.print("Receiving: ");
      Serial.print("\tFrom "); Serial.print(senderId);
      Serial.print("\tCode "); Serial.print((byte)code);
      Serial.print("\tValue "); Serial.println(value);
    */

    switch (code) {
      // Synchronize
      case sync_ask:
        sync.answer_node(senderId);
        break;

      case sync_ans:
        sync.receive_answer(senderId);
        break;

      case sample_duty_cyle:
        dutyCycles[senderId - 1] = value;
        break;

      case sample_lux:
        luxs[senderId - 1] = value;
        break;

      case set_restart_ask:
        write(senderId, set_restart_ans, 0);
        if (!calibrator.isOn())
          resetFunc();
        break;

      // Consensus run
      default:
        if (code == calibr_start_ask || code == calibr_set_ask || code == calibr_measure_ask)
          calibrator.ans(senderId, code);

        else if (code == calibr_start_ans || code == calibr_set_ans || code == calibr_measure_ans)
          calibrator.rcv(senderId, code);

        if (code == real_ask || code == start_ask || code == duty_cycle_ask || code == mean_ask || code == wait_ask)
          ledConsensus.ans(senderId, code, value);

        else if (code == real_ans || code == start_ans || code == duty_cycle_ans || code == mean_ans || code == wait_ans)
          ledConsensus.rcv(senderId, code, value);

        else if (code >= occupancy_ask && code <= set_restart_ask)
          pcComms.ans(senderId, code, value);

        else if (code >= occupancy_ans && code <= set_restart_ans)
          pcComms.rcv(senderId, code, value);

    }
  }
}

void getNodeId() {
  nodeId = digitalRead(ID0);
  nodeId |= digitalRead(ID1) << 1;
}

void timerIntConfig() {
  /* Configures a timer interruption using TIMER1 (for sampling) */
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;     //reset counter
  // OCR1A = desired_period/clock_period – 1 // = clock_freq/desired_freq - 1
  //= (10 / 500*10^-6) / 1 - 1 = 19999
  OCR1A = 19999; // corresponds to 10ms (100Hz sampling!)
  TCCR1B |= (1 << WGM12); // CTC, top is OCRA
  // Set prescaler for 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  // enable compare A
  TIMSK1 |= (1 << OCIE1A);
  sei();
}
