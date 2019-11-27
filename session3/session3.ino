#include <SPI.h>
#include <mcp2515.h>

#include "hub.h"
#include "can_comms.h"
#include "dist_control.h"
#include "calibr.h"

/*----------------- Variables -----------------*/
unsigned long counter = 0;
CALIBRATION calibrator;

/*----------- Function Definitions ------------*/


/*---------------------------------------------*/

void setup() {
  Serial.begin(115200);
  digitalWrite(4, HIGH); // We need an extra VCC
  getNodeId();

  // MCP2515 Configuration
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  setMasksFilters(); 

  // MCP2515 Mode Set
  mcp2515.setNormalMode();

  // See if this board is the hub
  hubFinder();

  // System calibration
  float* measurements;
  calibrator.init(nodeId, nNodes);
  calibrator.run(measurements);
  float measured_lux;
  for(int ii = 0; ii < nNodes; ii++) {
      measured_lux = getLux(measurements[ii]);
      k[ii] = (float) measured_lux / 100;
      Serial.print("Value of k"); Serial.print(nodeId); Serial.print(ii); Serial.print(" is equal to "); 
      Serial.println(k[ii]);
  }
}

void loop() {
  unsigned long c;
  MCP2515::ERROR err;
  if(hub){
    for(int i = 0; i < 3; i++) { //send 3 msgs in a burst
      Serial.print("Sending: ");
      Serial.println(counter);
      Serial.println(write(i, counter++));
    }
  }
  delay(10);
  while(read(c) == MCP2515::ERROR_OK) {//all buffers
    Serial.print("\t\t\tReceiving:");
    Serial.println(c);
    analogWrite(ledPin, c%255);
  }
  //Serial.println(read(c));
  delay(10); //time to breathe
}
