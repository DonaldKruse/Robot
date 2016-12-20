#define ERR_SUCCESS 0x0000
#define ERR_ILLEGAL_COMMAND 0x01
#define ERR_ILLEGAL_PARAMETER 0x02
#define ERR_UNKNOWN_FAILURE 0x03
#define ERR_OUT_OF_SENSOR_RANGE 0x04

// Distance units in millimeters
// Angular distance in degrees/2 (ie 0-180)

#define CMD_MOVE_FORWARD 0x01             // Parameters: Distance; Return: Command,Status
#define CMD_MOVE_REVERSE 0x02             // Parameters: Distance; Return: Command,Status
#define CMD_MOVE_CLOCKWIZE 0x03           // Parameters: Angle; Return: Command,Status
#define CMD_MOVE_COUNTERCLOCKWISE 0x04    // Parameters: Angle; Return: Command, Status
#define CMD_READ_BEARING 0x05             // Parameters: None; Return: Command, Bearing
#define CMD_READ_RANGE 0x06

// CMD_READ_VOLTS
// CMD_READ_TEMPERATURE
// CMD_TEST

