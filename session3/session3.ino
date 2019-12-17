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
bool writeFlag = false;
volatile bool sendFlag = false;

byte nodeId; // initialize the variable to make it global
byte nNodes = 3; // TODO: should be automatically computed
constexpr byte ID0 = 7;
constexpr byte ID1 = 8;

//float u_pid;
bool saturateInt = false;
//bool resetAWU = false;

float dutyCycles[maxNodes];
float luxs[maxNodes];
char pcMessage[11];

/*----------- Function Definitions ------------*/
void handleInterrupt();
void getNodeId();
void timerIntConfig();

ISR(TIMER1_COMPA_vect) {
  sampFlag = true;
  sendFlag = false;
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

  float y_init[maxNodes] = {0.0, 0.0, 0.0};

  sync.init(nodeId, nNodes);
  calibrator.init(nodeId, nNodes);
  ledConsensus.init(nodeId, nNodes, 0.1, 10, y_init);
  pid.init(0.5, 0.1, 0, 0.01, 0.1);  // 10, 1 was initial when it worked but with overshoot
  pcComms.init(nodeId, nNodes);

  timerIntConfig();

  pcMessage[0] = '*'; pcMessage[10] = '\n';
}

void loop() {
  //Serial.print("interrupt "); Serial.println(interrupt);
  if (interrupt)  // there are new messages
    handleNewMessages();

  if (sync.isOn())
    sync.ask_node();

  else if (calibrator.isOn())
    calibrator.run(ledConsensus);

  else {
    ledConsensus.run();

    if (sampFlag) {
      /*// Calc pid duty cycle given reference and new measurement
        //if (ledConsensus.finished()) {
        u_pid = pid.calc(ledConsensus.getLocalD() * k[nodeId - 1], getLux(analogRead(ldrPin)), saturateInt);
        // Write new duty cycle
        analogWrite(ledPin, constrain((int) (u_pid * 2.55 + 0.5), 0, 255));
        Serial.print("PID wrote "); Serial.println(constrain((int) (u_pid * 2.55 + 0.5), 0, 255));
        // See saturation
        saturateInt = (u_pid <= 0.0 || u_pid >= 100.0);

            if (!sendFlag) {
                // Communication of hub to PC will go here (?)
        sendFlag = true;
        }
        if (writeFlag) {
        writeFlag = false;
        sampFlag = false;
        }*/

      sampFlag = false;

      // Stream duty cycle and lux to pc
      dutyCycles[nodeId - 1] = ledConsensus.dNodeOverall[nodeId - 1];     // Use the value from the PID instead
      luxs[nodeId - 1] = getLux(analogRead(ldrPin));

      // Send duty cycle and lux to other nodes
      write(0, sample_duty_cyle, dutyCycles[nodeId - 1]);
      write(0, sample_lux, luxs[nodeId - 1]);

      for(byte i=1; i==nNodes; i++){
        pcMessage[1] = (char) i;
        memcpy(pcMessage+2, dutyCycles+i, sizeof(float));
        memcpy(pcMessage+6, luxs+i, sizeof(float));
        Serial.write(pcMessage, sizeof(pcMessage));
      }
    }
    //

    if (Serial.available() > 0) {
      Serial.println("Entered available");
      pcComms.SerialDecode();
    }
    pcComms.ask();
  }
  delay(10);
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

    /*Serial.print("Receiving: ");
      Serial.print("\tFrom "); Serial.print(senderId);
      Serial.print("\tCode "); Serial.print((byte)code);
      Serial.print("\tValue "); Serial.println(value);*/

    switch (code) {
      // Synchronize
      case sync_ask:
        sync.answer_node(senderId);
        break;

      case sync_ans:
        sync.receive_answer(senderId);
        break;

      // Calibrate
      case calibr_start:
        calibrator.init(nodeId, nNodes);
        break;

      case calibr_answer:
        calibrator.receive_answer(senderId, value);
        break;

      case calibr_wait:
        calibrator.send_answer(senderId, value);
        break;

      case sample_duty_cyle:
        dutyCycles[senderId - 1] = value;
        break;

      case sample_lux:
        luxs[senderId - 1] = value;
        break;

      // Duty cycle
      /*case duty_cycle_ack:
        ledConsensus.receive_ack(senderId);
        break;*/
      /*
            // Consensus initial communication
            case consensus_tell:
              ledConsensus.rcvStart(senderId);
              break;

            case consensus_rcv:
              ledConsensus.rcvAns(senderId);
              break;
      */
      // Consensus run
      default:
        if (code == duty_cycle_ask || code == mean_ask || code == real_ask)
          ledConsensus.ans(senderId, code);

        else if (code == duty_cycle_ans || code == mean_ans || code == real_ans)
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
