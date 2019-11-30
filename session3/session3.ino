#include <SPI.h>
#include <mcp2515.h>

#include "hub.h"
#include "can_comms.h"
#include "dist_control.h"
#include "calibr.h"

/*----------------- Variables -----------------*/
can_frame frame;
uint8_t msg[data_bytes]; char code0, code1; float value;
Calibration calibrator;

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

  // See if this board is the hub
  // hubFinder();

  /*
    // Convert measured values to lux
    float measured_lux;

    float disturbance = getLux(measurements[0]);
    Serial.print("Value of disturbance"); Serial.print(" is equal to ");
    Serial.println(disturbance);

    for (int i = 1; i <= nNodes; i++) {
    measured_lux = getLux(measurements[i]);
    k[i - 1] = measured_lux / max_lux;
    Serial.print("Value of k"); Serial.print(nodeId); Serial.print(i); Serial.print(" is equal to ");
    Serial.println(k[i - 1]);
    }
  */

  calibrator.start(nodeId, nNodes);

  Serial.println("Setup done.");
}

void loop() {
  Serial.print("interrupt "); Serial.println(interrupt);
  if (interrupt)  // there are new messages
    handleInterrupt();

  if (calibrator.isOn())
    calibrator.run();

  delay(500);
}

void handleInterrupt() {
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
    for (int i = 0; i < data_bytes; i++)
      msg[i] = frame.data[i];

    decodeMessage(msg, code0, code1, value);

    Serial.print("\tReceiving: ");
    Serial.print(msg[0]); Serial.print(' '); Serial.println(msg[1]);

    switch (code0) {
      case calibr_start[0]:
        switch (code1) {
          case calibr_start[1]:
            calibrator.start(nodeId, nNodes);
            break;

          case calibr_wait[1]:
            calibrator.receiveAck();
            break;
        }
        break;
    }
  }
}
