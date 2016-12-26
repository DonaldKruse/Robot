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


  cmd_move_forward (500);
  delay(1000);
  cmd_move_counterclockwise(90);  

}
