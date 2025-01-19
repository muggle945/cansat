#include <SoftwareSerial.h>

SoftwareSerial SIM7670Serial(2, 3); // RX, TX

void sendATCommand(const char* cmd, const char* expectedResponse, unsigned long timeout) {
  SIM7670Serial.println(cmd);
  String response = "";
  unsigned long startTime = millis();
  bool responseOK = false;

  while (millis() - startTime < timeout) {
    while (SIM7670Serial.available() > 0) {
      char c = SIM7670Serial.read();
      response += c;
    }
    if (response.indexOf(expectedResponse) != -1) {
      responseOK = true;
      break; // found it
    }
  }
  Serial.println(response);

  if (responseOK)
    Serial.println("Response OK");
  else
    Serial.println("Timeout without expected Response");

}

void setup() {
  Serial.begin(115200);
  SIM7670Serial.begin(115200);
  delay(2000); // Allow 2 seconds for the module to initialize
  sendATCommand("AT+CGMM", "OK", 10000); // check communication
  sendATCommand("AT+CMGF=1", "OK", 10000); // set SMS format to text

}

void sendSMS(String number, String message) {
  String cmd = "AT+CMGS=\"" + number + "\"\r\n";
  SIM7670Serial.print(cmd);
  delay(100);
  SIM7670Serial.println(message);
  delay(100);
  SIM7670Serial.write(0x1A); // send ctrl-z
  delay(100);
}
void loop() {
   sendSMS("+918527336556", "Hello from Arduino!");
  delay(3000);

   //SIM7670Serial.println("ATD +123456789;"); // Replace +123456789 with the desired phone number
  //delay(1000);

 if(SIM7670Serial.available()) {
    String response = SIM7670Serial.readString();
    Serial.println(response);
    // Check if the response indicates a call has been connected
    if(response.indexOf("CONNECT") != -1) {
      // Call connected
      Serial.println("Call Connected");
      // Add code here to perform actions during the call
    }
  }  
}
