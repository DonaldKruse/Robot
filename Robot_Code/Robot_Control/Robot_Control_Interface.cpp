#include "Robot_Control_Interface.h"



#define ERR_SUCCESS 0x0000
#define ERR_ILLEGAL_COMMAND 0x01
#define ERR_ILLEGAL_PARAMETER 0x02
#define ERR_UNKNOWN_FAILURE 0x03
#define ERR_OUT_OF_SENSOR_RANGE 0x04

// Distance units in millimeters
// Angular distance in degrees

#define CMD_MOVE_FORWARD 0x01             // Parameters: Distance, Speed; Return: Command,Status
#define CMD_MOVE_REVERSE 0x02             // Parameters: Distance, Speed; Return: Command,Status
#define CMD_MOVE_CLOCKWISE 0x03           // Parameters: Angle, Speed; Return: Command,Status
#define CMD_MOVE_COUNTERCLOCKWISE 0x04    // Parameters: Angle, Speed; Return: Command, Status
#define CMD_READ_BEARING 0x05             // Parameters: None; Return: Command, Bearing
#define CMD_READ_RANGE 0x06               // Parameters: None; Return: Command, Range
#define CMD_READ_VOLTS 0x07               // Parameters: None; Return: Command, Volts
#define CMD_READ_TEMPATURE 0x08           // Parameters: None; Return: Command, Tempature
#define CMD_TEST 0x09                     // Parameters: Command; Return: Command, ErrorCode


/************************************************************/
/* Delete above                                             */
/************************************************************/

/* Internal Reading */
byte cmd_read_volts();
byte cmd_read_tempature();
byte cmd_test(byte command); 


/* Movement and Heading Commands, Transport */


/**
 * Moves forward
 * 
 * distance: our distance to move in mm 
 * movspeed: speed at which to move
 * 
 * returns error code
 */
byte cmd_move_forward(byte distance, byte movmovspeed) {
  
  if (movspeed > 250) movspeed = 250;
  
  digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
  digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
  analogWrite(PWM_STARBOARD_DRIVE, movspeed);

  digitalWrite(FORWARD_PORT_DRIVE, HIGH);
  digitalWrite(REVERSE_PORT_DRIVE, LOW);
  analogWrite(PWM_PORT_DRIVE, movspeed);  

  delay(move_time);

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
  delay(move_time);

  analogWrite(PWM_STARBOARD_DRIVE, 0);
  analogWrite(PWM_PORT_DRIVE, 0); 

  return ERR_SUCCESS; //TODO--Consider surrounding with try-catch 
}

byte cmd_move_clockwise(byte angle, byte movspeed);

/**
 * Turns counter-clockwise
 * 
 * angle: angle to turn
 * movspeed: speed at which to turn
 * 
 * returns error code
 */
byte cmd_move_counterclockwise(byte angle, byte movspeed) {
  float bearing;
  int counter;

  if (movspeed > 250) movspeed = 250;

  get_bearing(&bearing);
  
  digitalWrite(FORWARD_STARBOARD_DRIVE, HIGH);
  digitalWrite(REVERSE_STARBOARD_DRIVE, LOW);
  analogWrite(PWM_STARBOARD_DRIVE, movspeed);

  digitalWrite(FORWARD_PORT_DRIVE, LOW);
  digitalWrite(REVERSE_PORT_DRIVE, HIGH);
  analogWrite(PWM_PORT_DRIVE, movspeed);  
  counter = 0;
  while ((abs(bearing-heading) > 10) && (counter < 10)) { 
    get_bearing(&bearing); 
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
byte cmd_read_bearing(byte* bearing) {
  /***** TODO - Rewrite to use only integer math with lookup table for atan2 *****/

  /* Read bearing from HMC5883L */
    /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Calculate bearing when the magnetometer is level, then correct for signs of axis.
  *bearing = atan2(event.magnetic.y, event.magnetic.x);
  
// TODO - commented out.  This is not useful without calibrating the sensor.
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

  return ERR_SUCCESS; //TODO--consider try-catch
}

/**
 * reads range to nearest object in mm
 * 
 * range: pointer to range value
 * 
 * returns error code
 */
byte cmd_read_range(byte* range) {
  
  /* HY-SRF05 */
  unsigned int echoTime;
  delay(50);

  echoTime = sonar.ping_median(ITERATIONS);
  *distance = sonar.convert_cm(echoTime);
  *distance *= 10; //convert to mm
  
  if (distance > 0) {
    return ERR_SUCCESS;
  } else {
    return ERR_OUT_OF_SENSOR_RANGE;
  }
}

