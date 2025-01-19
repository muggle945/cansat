
void setup() {
  // Initialize the USB Serial port for communication with the computer
  Serial.begin(9600);
  
  // Initialize Serial2 with the same baud rate as your device
  Serial2.begin(9600);

  // Wait for the Serial port to connect. This is necessary for some Teensy configurations.


  Serial.println("Ready to receive data on Serial2...");
}

void loop() {
  // Check if there is any data available on Serial2
  if (Serial2.available()) {
    // Read the incoming byte from Serial2
    byte incomingByte = Serial2.read();
    
    // Send the byte to the USB Serial so it can be viewed in the Serial Monitor
    Serial.println(incomingByte);
  }
  else{
    Serial.println("Not working");
  }
}
