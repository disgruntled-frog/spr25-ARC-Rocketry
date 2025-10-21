#include <Adafruit_GPS.h>
#include "SoftwareSerial.h"
// -------------------- Custom pins --------------------
#define GPSSerial Serial1
#define RX1_PIN 16 // GPS TX -> ESP32 RX
#define TX1_PIN 17 // GPS RX -> ESP32 TX (optional)

String lora_RX_address = "2";

Adafruit_GPS GPS(&GPSSerial);

HardwareSerial LoRaSerial(2); // use Serial2

SoftwareSerial lora(4, 5);

#define GPSECHO false // true = echo raw NMEA for debugging

uint32_t timer = 0;

// -------------------- Helper: convert NMEA to decimal degrees --------------------
float convertToDecimal(float nmeaValue) {
  int deg = int(nmeaValue / 100);        // degrees
  float min = nmeaValue - deg * 100;     // minutes
  return deg + min / 60.0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 GPS Decimal Degrees Test");

  // Initialize Serial1 on custom pins
  GPSSerial.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
  lora.begin(9600);


  // Configure GPS output and update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC + GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz
  GPS.sendCommand(PGCMD_ANTENNA);               // antenna status

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);           // request firmware version
}

void loop() {
  // Read GPS characters
  char c = GPS.read();
  if (GPSECHO && c) Serial.write(c);

  // Check for a full new NMEA sentence
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Print parsed GPS info every second
  if (millis() - timer > 1000) {
    timer = millis();
    Serial.print("Fix: "); Serial.println(GPS.fix);

    //if (GPS.fix) {
      float lat = convertToDecimal(GPS.latitude);
      if (GPS.lat == 'S') lat = -lat;

      float lon = convertToDecimal(GPS.longitude);
      if (GPS.lon == 'W') lon = -lon;

      Serial.print("Latitude: "); Serial.println(lat, 6);
      Serial.print("Longitude: "); Serial.println(lon, 6);
      Serial.print("Altitude (m): "); Serial.println(GPS.altitude);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.println("----------------------");
      String msg = String(lat,6) + "," + String(lon,6);
      
      double length =msg.length();
      lora.println("AT+SEND=" + lora_RX_address + ","+ length +","+msg); 
      Serial.println("msg sent");
      Serial.println();
      Serial.println("----------------------");

    //} else {
    //  Serial.println("Waiting for GPS fix...");
    //  Serial.println("----------------------");
    //}
  }
}
