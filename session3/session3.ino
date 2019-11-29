#include <SPI.h>
#include <mcp2515.h>

#include "hub.h"
#include "can_comms.h"
#include "dist_control.h"
#include "calibr.h"

/*----------------- Variables -----------------*/
unsigned long counter = 0;
float *measurements;
CALIBRATION calibrator;

/*----------- Function Definitions ------------*/


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
  hubFinder();

  // System calibration
  calibrator.init(nodeId, nNodes, measurements);
  calibrator.run(measurements);
  // Convert measured values to lux
  float measured_lux;
  
  float disturbance = getLux(measurements[0]);
  Serial.print("Value of disturbance"); Serial.print(" is equal to "); 
  Serial.println(disturbance);

  for(int i = 1; i <= nNodes; i++) {
      measured_lux = getLux(measurements[i]);
      k[i-1] = measured_lux / max_lux;
      Serial.print("Value of k"); Serial.print(nodeId); Serial.print(i); Serial.print(" is equal to "); 
      Serial.println(k[i-1]);
  }
}

void loop() {
  can_frame frame;
  unsigned long c;
  my_can_msg msg;

  MCP2515::ERROR err;
  if(hub){
    for(int i = 1; i <= nNodes; i++) { //send 3 msgs in a burst
      Serial.print("Sending: ");
      Serial.println(counter);
      Serial.println(write(i, 0, counter++));
    }
  }
  delay(10);
  while(cf_stream.get(frame) == 1) {//all buffers
    Serial.print("\t\t\tReceiving:");
    for(int i = 0; i < 4; i++) msg.bytes[i] = frame.data[i];
    c = msg.value;
    analogWrite(ledPin, c%255);
  }
  //Serial.println(read(c));
  delay(10); //time to breathe
}
