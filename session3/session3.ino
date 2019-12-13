#include <SPI.h>
#include <mcp2515.h>

#include "can_comms.h"
#include "dist_control.h"
#include "calibr.h"
#include "sync.h"

/*----------------- Variables -----------------*/
can_frame frame;
byte senderId;
char code;
float value;
Calibration calibrator;
Sync sync;
LedConsensus ledConsensus;

byte nodeId; // initialize the variable to make it global
byte nNodes = 3; // TODO: should be automatically computed
constexpr byte ID0 = 7;
constexpr byte ID1 = 8;
/*----------- Function Definitions ------------*/
void handleInterrupt();

void getNodeId() {
  nodeId = digitalRead(ID0);
  nodeId |= digitalRead(ID1) << 1;
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
  ledConsensus.init(nodeId, nNodes, 0.1, 1, y_init);

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
  }

  delay(5);
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

    Serial.print("Receiving: ");
    Serial.print("\tFrom "); Serial.print(senderId);
    Serial.print("\tCode "); Serial.print((byte)code);
    Serial.print("\tValue "); Serial.println(value);

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
    }
  }
}
