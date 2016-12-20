#include <NewPing.h>
 
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200
#define ITERATIONS 3
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);

}

void loop() {
  unsigned int echoTime;
  unsigned int distance;
  delay(50);

  echoTime = sonar.ping_median(ITERATIONS);
  distance = sonar.convert_in(echoTime);

  if (distance > 0) {
    Serial.print("Ping: ");
    // Serial.print(echoTime)/1000;
    // Serial.print(" , ");
    Serial.print(distance);
    Serial.println(" in");
  }

}
