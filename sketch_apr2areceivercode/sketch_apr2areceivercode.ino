#include <SoftwareSerial.h>
String c = "";
SoftwareSerial xbeeSerial(15, 14); // RX, TX pins
// int wakePin = A10;
void setup() {
  Serial.begin(9600); // Initialize serial monitor
  xbeeSerial.begin(9600); // Initialize XBee serial communication
  // pinMode(wakePin,OUTPUT);
}

void loop() {
  if (xbeeSerial.available()) { // Check if data is available to read
    c = xbeeSerial.readString();
    Serial.println(c); // Print a newline after receiving all data
    xbeeSerial.println(c);
    delay(500);
  }
}
