#include <DueTimer.h>
#include "CANOpen_SAM3X.h"

extern uint32_t GetTick = 0;

CANopenNodeSAM3X canopenNode;

void myHandler() {
  GetTick++;
  canopen_app_interrupt();
}

void setup() {

  Serial.begin(115200);
  
  canopenNode.CANHandle = CAN0;
  canopenNode.desiredNodeID = 1;
  canopenNode.baudrate = 1000;
  canopen_app_init(&canopenNode);
  // canopen_app_process();

  Timer1.attachInterrupt(myHandler);
  Timer1.start(1000);  // Calls every 50ms
}

void loop() {

  canopen_app_process();

  OD_RAM.x6000_velocity++;
  OD_RAM.x6001_temperature=(uint8_t)rand();
  OD_RAM.x6003_speed=rand();
}