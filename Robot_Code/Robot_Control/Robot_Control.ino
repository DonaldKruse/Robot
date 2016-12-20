#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <NewPing.h>
#include "Robot_Control_Interface.h"
#include "Hardware.h" 



/**********************************************************************************/
/*     Name: get_heading()                                                        */
/*     Description: Reads heading from magnetometer.  Returns one byte error      */    
/*                  code.                                                         */
/**********************************************************************************/

/* Assign a unique ID to the magnitometer  */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

byte get_bearing (float *bearing)
{
  /***** TODO - Rewrite to use only integer math with lookup table for atan2 *****/

  /* Read bearing from HMC5883L */
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Calculate bearing when the magnetometer is level, then correct for signs of axis.
  *bearing = atan2(event.magnetic.y, event.magnetic.x);
  
// TODO - commented out.  This is not useful with calibrating the sensor.
//  float declinationAngle = 0.15; // ABQ declination in radians.
//  *bearing += declinationAngle;
//  
//  // Correct for when signs are reversed.
//  if(*bearing < 0)
//    *bearing += 2*PI;
//    
//  // Check for wrap due to addition of declination.
//  if(*bearing > 2*PI)
//    *bearing -= 2*PI;

 
  // Convert radians to degrees for readability.
  *bearing = *bearing * 180/M_PI; 

  return ERR_SUCCESS;
}

/**********************************************************************************/
/*     Name: get_range()                                                          */
/*     Description: Reads distance to nearest obstacle from sonar sensor.         */
/*                  Returns one byte error code with success or failure.          */
/**********************************************************************************/

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

byte get_range(unsigned int *distance)
{

  /* HY-SRF05 */
  unsigned int echoTime;
  delay(50);

  echoTime = sonar.ping_median(ITERATIONS);
  *distance = sonar.convert_in(echoTime);

  if (distance > 0) {
    return ERR_SUCCESS;
  } else {
    return ERR_OUT_OF_SENSOR_RANGE;
  }
}

/**********************************************************************************/
/*     Name:  move_forward, move_reverse, turn_left                               */
/*     Description: Commands the drive motors to execute moves at commanded speed */
/*                  and duration.                                                 */
/**********************************************************************************/
byte move_forward(unsigned int move_time, unsigned int speed) {

  if (speed > 250) speed = 250;
  
  digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
  digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
  analogWrite(PWM_STARBOARD_DRIVE, speed);

  digitalWrite(FORWARD_PORT_DRIVE, HIGH);
  digitalWrite(REVERSE_PORT_DRIVE, LOW);
  analogWrite(PWM_PORT_DRIVE, speed);  

  delay(move_time);

  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0);  
}

byte move_reverse(unsigned int move_time, unsigned int speed) {

  if (speed > 250) speed = 250;

 
  digitalWrite(FORWARD_STARBOARD_DRIVE, LOW);
  digitalWrite(REVERSE_STARBOARD_DRIVE, HIGH);
  analogWrite(PWM_STARBOARD_DRIVE, speed);
  
  digitalWrite(FORWARD_PORT_DRIVE, LOW);
  digitalWrite(REVERSE_PORT_DRIVE, HIGH);
  analogWrite(PWM_PORT_DRIVE, speed);  
  delay(move_time);

  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0);  
}

// TODO -- Not used. 
//byte turn_right(double heading, unsigned int speed) {
//  
//  float bearing;
//  
//  if (speed > 250) speed = 250;
//
//  get_bearing(&bearing);
//  digitalWrite(FORWARD_STARBOARD_DRIVE, LOW);
//  digitalWrite(REVERSE_STARBOARD_DRIVE, HIGH);
//  analogWrite(PWM_STARBOARD_DRIVE, speed);
//  
//  digitalWrite(FORWARD_PORT_DRIVE, HIGH);
//  digitalWrite(REVERSE_PORT_DRIVE, LOW);
//  analogWrite(PWM_PORT_DRIVE, speed);  
//  
//  while (abs(bearing-heading) > 3) {  
//    delay(10);
//  }
//  analogWrite(PWM_STARBOARD_DRIVE, 0);
//  analogWrite(PWM_PORT_DRIVE, 0);  
//}


byte turn_left(float heading, unsigned int speed) {
  
  float bearing;
  int counter;

  if (speed > 250) speed = 250;

  get_bearing(&bearing);
  
  digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
  digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
  analogWrite(PWM_STARBOARD_DRIVE, speed);

  digitalWrite(FORWARD_PORT_DRIVE, LOW);
  digitalWrite(REVERSE_PORT_DRIVE, HIGH);
  analogWrite(PWM_PORT_DRIVE, speed);  
  counter = 0;
  while ((abs(bearing-heading) > 10) && (counter < 10)) { 
    get_bearing(&bearing); 
    delay(10);
    counter++;
  }
  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0);  
}

byte all_stop(unsigned int move_time) {


  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0);  

  delay(move_time);
}

void setup() {
  Serial.begin(9600);

  delay(10000);

  /*******************************/
  /*  HMC5883L Setup             */
  /*******************************/  
    /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  /*******************************/
  /* Initialize Drive Motors     */
  /*******************************/
  pinMode(PWM_STARBOARD_DRIVE, OUTPUT);
  pinMode(FORWARD_STARBOARD_DRIVE, OUTPUT);
  pinMode(REVERSE_STARBOARD_DRIVE, OUTPUT);
  pinMode(PWM_PORT_DRIVE, OUTPUT);
  pinMode(FORWARD_PORT_DRIVE, OUTPUT);
  pinMode(REVERSE_PORT_DRIVE, OUTPUT);
  

}

void loop() {

  unsigned int range;
  float bearing;
  byte error_return;

get_bearing(&bearing);
Serial.print("Bearing: ");
Serial.println(bearing);

  error_return = get_range(&range); 
  if (error_return == ERR_OUT_OF_SENSOR_RANGE) range = MAX_DISTANCE;
Serial.print("Range: ");
Serial.println(range);
  if (range >= 0) {
    if (range < 6) {
        move_reverse(100,80);
       
        digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
        digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
        analogWrite(PWM_STARBOARD_DRIVE, 80);
  
        digitalWrite(FORWARD_PORT_DRIVE, LOW);
        digitalWrite(REVERSE_PORT_DRIVE, HIGH);
        analogWrite(PWM_PORT_DRIVE, 80); 
        delay(100); 
    } else {
      get_bearing(&bearing); 
      move_forward(800,80);
      turn_left(bearing,80);
    }
  }


}
