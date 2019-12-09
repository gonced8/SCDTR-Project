#include <SPI.h>
#include <mcp2515.h>

#include "hub.h"
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

/*----------- Function Definitions ------------*/
void handleInterrupt();

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

  float y_init[maxNodes] = {0.0, 0.0, 0.0, 0.0, 0.0};

  sync.init(nodeId, nNodes);
  calibrator.init(nodeId, nNodes);
  ledConsensus.init(nodeId, nNodes, 1, 1, y_init);

  Serial.println("Setup done.");
}

void loop() {
  //Serial.print("interrupt "); Serial.println(interrupt);
  if (interrupt)  // there are new messages
    handleNewMessages();

  if (sync.isOn())
    sync.ask_node();

  else if (calibrator.isOn())
    calibrator.run(ledConsensus);

  else
    ledConsensus.run();

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

    Serial.print("\tReceiving: ");
    Serial.print("Code "); Serial.println(code);
    Serial.print("From "); Serial.println(senderId);

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
        calibrator.receive_answer(senderId);
        break;

      case calibr_wait:
        calibrator.send_answer(senderId);
        break;

      default:
        if (code >= duty_cycle_code || code < duty_cycle_code + nNodes)
          ledConsensus.receive_duty_cycle(senderId, code, value);
        break;
    }
  }
}
