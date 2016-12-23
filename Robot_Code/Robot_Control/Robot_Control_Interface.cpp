
#include "Robot_Control_Interface.h"
#include "Hardware.h"

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

/* Assign a unique ID to the magnitometer  */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

/* Internal Reading */
//TODO--Define these functions.
//byte cmd_read_volts()
//byte cmd_read_tempature();
//byte cmd_test(byte command); 

compass_cal_struct compass_cal = {35.73, -17.64, 55.64, 2.55};  // Hardcoded values for Prototype A.

byte hardware_init()
{
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

/* Movement and Heading Commands, Transport */


/**
 * Moves forward
 * 
 * distance: our distance to move in mm 
 * movspeed: speed at which to move
 * 
 * returns error code
 */
byte cmd_move_forward(unsigned int distance, unsigned int movspeed) {
  
  if (movspeed > 250) movspeed = 250;
  
  digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
  digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
  analogWrite(PWM_STARBOARD_DRIVE, movspeed);

  digitalWrite(FORWARD_PORT_DRIVE, HIGH);
  digitalWrite(REVERSE_PORT_DRIVE, LOW);
  analogWrite(PWM_PORT_DRIVE, movspeed);  


  // TODO - compute move_time from distance. For now use default time of 100 ms
  //delay(move_time);
  delay(100);

  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0);  

  return ERR_SUCCESS; //TODO--Consider surrounding with try-catch 
}
/**
 * Moves in reverse
 * 
 * distance: our distance to move in mm 
 * movspeed: speed at which to move
 * 
 * returns error code
 */
byte cmd_move_reverse(byte distance, byte movspeed) {
  if (movspeed > 250) movspeed = 250;

 
  digitalWrite(FORWARD_STARBOARD_DRIVE, LOW);
  digitalWrite(REVERSE_STARBOARD_DRIVE, HIGH);
  analogWrite(PWM_STARBOARD_DRIVE, movspeed);
  
  digitalWrite(FORWARD_PORT_DRIVE, LOW);
  digitalWrite(REVERSE_PORT_DRIVE, HIGH);
  analogWrite(PWM_PORT_DRIVE, movspeed);  
  // TODO - compute move_time from distance. For now use default time of 100 ms
  //delay(move_time);
  delay(100);

  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0); 

  return ERR_SUCCESS; //TODO--Consider surrounding with try-catch 
}

byte cmd_move_clockwise(unsigned int angle, unsigned int movspeed);

/**
 * Turns counter-clockwise
 * 
 * angle: angle to turn
 * movspeed: speed at which to turn
 * 
 * returns error code
 */
byte cmd_move_counterclockwise(unsigned int angle, unsigned int movspeed) {
  unsigned int bearing;
  int counter;

  if (movspeed > 250) movspeed = 250;

  cmd_read_bearing(&bearing);
  
  digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
  digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
  analogWrite(PWM_STARBOARD_DRIVE, movspeed);

  digitalWrite(FORWARD_PORT_DRIVE, LOW);
  digitalWrite(REVERSE_PORT_DRIVE, HIGH);
  analogWrite(PWM_PORT_DRIVE, movspeed);  
  counter = 0;
  while ((abs(bearing-angle) > 10) && (counter < 10)) { 
    cmd_read_bearing(&bearing); 
    delay(10);
    counter++;
  }
  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0);  

  //TODO--use getBearing to help determine success/failure
  return ERR_SUCCESS; //TODO--consider using try-catch
}

/**
 * Gets the bearing 
 * 
 * bearing: pointer to the bearing in degrees, used as return value
 * 
 * returns error code
 */
byte cmd_read_bearing(unsigned int* bearing) {
  /***** TODO - Rewrite to use only integer math with lookup table for atan2 *****/

  float flt_bearing;
  
  /* Read bearing from HMC5883L */
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Calculate bearing when the magnetometer is level, then correct for signs of axis.

  /* Correct the vector for the previously measured zero point (calibration)
   *  then calculate the bearing.
   */
  flt_bearing = atan2(event.magnetic.y - (compass_cal.max_y - compass_cal.min_y)/2, 
                      event.magnetic.x - (compass_cal.max_x - compass_cal.min_x)/2);
  
  float declinationAngle = 0.15; // ABQ declination in radians.
  *bearing += declinationAngle;
  
  // Correct for when signs are reversed.
  if(*bearing < 0)
    *bearing += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(*bearing > 2*PI)
    *bearing -= 2*PI;

 
  // Convert radians to unsigned degrees.
  flt_bearing = flt_bearing * 180/M_PI; 
  if (flt_bearing < 0) flt_bearing += 360;

  *bearing = round(flt_bearing);
  

  return ERR_SUCCESS; //TODO--consider try-catch
}

/**
 * reads range to nearest object in mm
 * 
 * range: pointer to range value
 * 
 * returns error code
 */
byte cmd_read_range(unsigned int* range) {
  
  /* HY-SRF05 */
  unsigned int echoTime;
  unsigned int distance;
  delay(50);

  echoTime = sonar.ping_median(ITERATIONS);
  distance = sonar.convert_cm(echoTime);
  distance *= 10; //convert to mm

  *range = distance;
  
  if (distance > 0) {
    return ERR_SUCCESS;
  } else {
    return ERR_OUT_OF_SENSOR_RANGE;
  }
}

/**
 * Detects min and max for x & y components of magnetic field vector and stores them in memory.
 * 
 * returns error code
 */
byte cmd_calibrate_compass() {

  /*  Current version is meant to be called repeatedly while robot is rotated manually. */
  /*    Steps to use:                                                                   */
  /*     1) Add commands (see below)  to main loop to call this routine and then serial print max/min
   *      values.
   *     2) Set dafaut values of compass_cal to {1000,-1000,1000,-1000}
   *     3) Rotate robot several times while recording max values in serial monitor.  
   *     4) Update default values for compass_cal.
   *     
   *       // Code for main loop
   *       compass_cal_struct* compass_cal_pntr;
   *       cmd_calibrate_compass();
   *       compass_cal_pntr = get_compass_cal();
   *       Serial.print("X_max: ");  
   *       Serial.print(compass_cal_pntr->max_x); 
   *       Serial.print(", X_min: ");
   *       Serial.print(compass_cal_pntr->min_x);
   *       Serial.print("; Y_max: ");
   *       Serial.print(compass_cal_pntr->max_y);
   *       Serial.print(", Y_min: ");
   *       Serial.println(compass_cal_pntr->min_y);
   *       delay(500);
   */

  /* TODO -- Update routine to allow robot to auto-calibrate.  Move compass_cal to 
   *  non-volatile memory.
   */
    /* Read bearing from HMC5883L */
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  if (event.magnetic.x < compass_cal.min_x) compass_cal.min_x = event.magnetic.x;
  if (event.magnetic.y < compass_cal.min_y) compass_cal.min_y = event.magnetic.y;
  if (event.magnetic.x > compass_cal.max_x) compass_cal.max_x = event.magnetic.x;
  if (event.magnetic.y > compass_cal.max_y) compass_cal.max_y = event.magnetic.y;
  
  return ERR_SUCCESS;
}

/**
 * get the current compass calibration values
 * 
 * returns a pointer to the calibration struct
 */
 compass_cal_struct* get_compass_cal()  {
   return &compass_cal;
 }

