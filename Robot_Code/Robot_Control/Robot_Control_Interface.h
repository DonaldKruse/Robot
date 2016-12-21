#include <Arduino.h>

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

/* Internal Reading */
byte cmd_read_volts();
byte cmd_read_tempature();
byte cmd_test(byte command); 

/* Function to initialize data structures, actuators, and sensors.  Call once. */
byte hardware_init();

/* Movement and Heading Commands, Transport */
byte cmd_move_forward(unsigned int distance, unsigned int movspeed);
byte cmd_move_reverse(unsigned int distance, unsigned int movspeed);
byte cmd_move_clockwise(unsigned int angle, unsigned int  movspeed);
byte cmd_move_counterclockwise(unsigned int angle, unsigned int movspeed);
byte cmd_read_bearing(unsigned* bearing);
byte cmd_read_range(unsigned* range);
