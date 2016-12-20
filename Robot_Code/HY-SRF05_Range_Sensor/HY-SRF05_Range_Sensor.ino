#define SONAR_TRIGGER_PIN     2
#define SONAR_ECHO_PIN        3

unsigned int measure_distance()
{
   // Trigger the SRF05:
   digitalWrite(SONAR_TRIGGER_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(SONAR_TRIGGER_PIN, LOW);

   // Wait for Echo Pulse
   unsigned long pulse_length = pulseIn(SONAR_ECHO_PIN, HIGH);

   // Ensure the ultrasonic "beep" has faded away
   delay(50);

   // Convert Pulse to Distance (inches)
   // pulse_length/58 = cm or pulse_length/148 = inches
   return( (unsigned int) (pulse_length / 148) );
}

void setup() {
   pinMode(SONAR_TRIGGER_PIN, OUTPUT);
   pinMode(SONAR_ECHO_PIN, INPUT);
   Serial.begin(9600);
}

void loop() {
   unsigned int current_distance = measure_distance();
   Serial.println(current_distance);
   delay(125);

}
