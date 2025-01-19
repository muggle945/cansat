#include <Wire.h>
#include "Adafruit_BME680.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LoRa.h>

#define rxGPS 0
#define txGPS 1

#define SEALEVELPRESSURE_HPA (1013.25)
#define CS_PIN 10 // Chip select pin for BME680
#define LORA_CS 7     // Chip select pin for LoRa (NSS)
#define LORA_RST 9    // Reset pin for LoRa
#define LORA_IRQ 2    // IRQ pin for LoRa (can be changed as needed)

const int mq2Pin = A9;   // MQ-2 for smoke and flammable gases
const int mq7Pin = A10;  // MQ-7 for carbon monoxide
const int mq135Pin = A11; // MQ-135 for air quality (various gases)

unsigned long latitude = 0;
unsigned long longitude = 0;
float speed;

Adafruit_BME680 bme(CS_PIN);
Adafruit_MPU6050 mpu;

SoftwareSerial gpsSerial(rxGPS, txGPS);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  while (!Serial);
  Serial.println(F("Begin"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize LoRa module
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
  }
  Serial.println("LoRa initialized.");
}

void loop() {
  unsigned long startTime = millis();

  // Check GPS data for 5 seconds
  while (millis() - startTime < 2000) {  
    GPS();
  }

  // Now handle the other sensors
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Read sensor data

  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0;
  float humidity = bme.readHumidity();
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  int smoke = analogRead(mq2Pin); // Read MQ-2 sensor value
  int CO = analogRead(mq7Pin);    // Read MQ-7 sensor value
  int AQ = analogRead(mq135Pin);  // Read MQ-135 sensor value

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  // If GPS has valid data, convert to unsigned long
  
  String word = String(temperature) + "," + String(pressure) + "," + String(humidity) + "," + String(altitude) + "," +
                String(latitude) + "," + String(longitude) + "," + String(smoke) + "," + String(CO) + "," + 
                String(AQ) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," +
                String(gx) + "," + String(gy) + "," + String(gz);

  Serial.println(word);

  // Send data over LoRa
  LoRa.beginPacket();
  LoRa.print(word);
  LoRa.endPacket();

  delay(2000);
}

void GPS() {
  while (gpsSerial.available()) {
   if (gps.encode(gpsSerial.read())){
    if(gps.location.isValid()){ 
      Serial.print("SATS: ");  
      Serial.println(gps.satellites.value());
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("ALT: ");
      Serial.println(gps.altitude.meters());
      Serial.print("SPEED: ");
      Serial.println(gps.speed.mps());}
   }
  }
}
