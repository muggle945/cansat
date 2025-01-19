#include <SoftwareSerial.h>

// Define XBee TX and RX pins
#define XBEE_TX_PIN 2   // Replace with the digital pin connected to XBee's RX pin
#define XBEE_RX_PIN 3   // Replace with the digital pin connected to XBee's TX pin

SoftwareSerial xbeeSerial(XBEE_RX_PIN, XBEE_TX_PIN); // RX, TX

void setup() {
  Serial.begin(9600);     // Initialize serial communication for debugging
  xbeeSerial.begin(9600); // Initialize XBee communication

  // Wait for the XBee module to initialize
  delay(1000);
}

void loop() {
  // Transmit "Hello World" to the coordinator
  xbeeSerial.print("Hello World");

  // Wait for a brief period before sending next message
  delay(5000);
}
