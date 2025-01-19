#include <SoftwareSerial.h>

SoftwareSerial SIM7670Serial(2, 3); // RX, TX

void setup() {
    Serial.begin(115200);
    SIM7670Serial.begin(115200);
    sendATCommand("AT", "OK", 5000);  // Initial AT command to check communication
    checkNetworkStatus();
    sendATCommand("AT+CMGF=1", "OK", 5000); // Set SMS format to text mode
}

void loop() {
    sendSMS("+917652802989", "Hello from Arduino!");
    delay(10000); // Delay to prevent constant sending
}

void sendATCommand(const char* cmd, const char* expectedResponse, unsigned long timeout) {
    SIM7670Serial.println(cmd);
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        while (SIM7670Serial.available()) {
            char c = SIM7670Serial.read();
            response += c;
        }
        if (response.indexOf(expectedResponse) != -1) {
            Serial.println(response); // Print the full response
            Serial.println("Response OK");
            return;
        }
    }
    Serial.println(response); // Print the received response even if not as expected
    Serial.println("Timeout or incorrect response");
}

void sendSMS(String number, String message) {
    String command = "AT+CMGS=\"" + number + "\"";
    sendATCommand(command.c_str(), ">", 5000); // Convert String to const char* with c_str()
    SIM7670Serial.println(message);
    SIM7670Serial.write(26); // Ctrl-Z to send SMS
    delay(1000); // Wait to complete send
}

void checkNetworkStatus() {
    sendATCommand("AT+CREG?", "+CREG: 0,1", 5000); // Check network registration
    sendATCommand("AT+CSQ", "+CSQ:", 5000); // Check signal quality
}