#include <Wire.h>

#include "Robot_Control_Interface.h"
#include "Hardware.h" 



void setup() {
  byte init_status;
  Serial.begin(9600);

  delay(10000);

  init_status = hardware_init();
  
}


void loop() {


}
