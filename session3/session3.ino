#include <SPI.h>
#include <mcp2515.h>

#include "hub.h"
#include "can_comms.h"
#include "dist_control.h"
#include "calibr.h"
#include "sync.h"

/*----------------- Variables -----------------*/
can_frame frame;
uint8_t msg[data_bytes];
char code[2];
float value;
byte senderId;
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
    if (ledConsensus.finished()) {
      // THIS WRITE WILL LATER BE IN THE PID! disturbance calculated for previous dutyCycle
      measuredLux = getLux(analogRead(ldrPin));
      calcDisturbance(ledConsensus, measuredLux);
      analogWrite(ledPin, dutyCycle * 255.0 / 100);
      Serial.print("Wrote to led "); Serial.println(dutyCycle);
      //////
    } else {
      ledConsensus.run();
    }
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
    decodeMessage(frame, senderId, code, &value);

    Serial.print("\tReceiving: ");
    Serial.print((char)msg[0]); Serial.print(' '); Serial.println((char)msg[1]);
    Serial.print("From "); Serial.println(senderId);

    switch (code[0]) {
      // Synchronize
      case sync_ask[0]:
        switch (code[1]) {
          case sync_ask[1]:
            sync.answer_node(senderId);
            break;

          case sync_ans[1]:
            sync.receive_answer(frame);
            break;
        }
        break;

      // Calibrate
      case calibr_start[0]:
        switch (code[1]) {
          case calibr_start[1]:
            calibrator.init(nodeId, nNodes);
            break;

          case calibr_answer[1]:
            calibrator.receive_answer(senderId);
            break;

          case calibr_wait[1]:
            calibrator.send_answer(senderId);
            break;
        }
        break;

      // Consensus initial communication
      case consensus_tell[0]:
        switch (code[1]) {
          case consensus_tell[1]:
            ledConsensus.rcvStart(senderId);
            break;

          case consensus_rcv[1]:
            ledConsensus.rcvAns(frame);
            break;
        }
        break;

      default:
        if (code[0] >= duty_cycle_code || code[0] < duty_cycle_code + nNodes)
          ledConsensus.receive_duty_cycle(frame);
    }
  }
}
