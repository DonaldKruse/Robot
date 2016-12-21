#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <NewPing.h>

/* Assign a unique ID to the magnitometer  */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);




/* Definitions for Sonar ranging sensor - HY-SRF05 */
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 2000
#define ITERATIONS 2

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

/* Pin definitions for transport contol module */
#define PWM_STARBOARD_DRIVE 6
#define FORWARD_STARBOARD_DRIVE 4
#define REVERSE_STARBOARD_DRIVE 8
#define PWM_PORT_DRIVE 9
#define FORWARD_PORT_DRIVE 7
#define REVERSE_PORT_DRIVE 5
