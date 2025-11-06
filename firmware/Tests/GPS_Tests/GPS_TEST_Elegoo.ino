#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// === Pin connections ===
// GPS TX -> Arduino D4
// (GPS RX -> Arduino D3 is optional)
// VCC -> 3.3V or 5V (check board label)
// GND -> GND

SoftwareSerial gpsSerial(4, 3); // RX = 4, TX = 3
TinyGPSPlus gps;

unsigned long lastPrint = 0;
bool hasFix = false;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println(F("=== MIKROE-1032 GPS Test ==="));
  Serial.println(F("Initializing GPS..."));
  Serial.println(F("Make sure the antenna has a clear view of the sky.\n"));
}

void loop() {
  // Feed data from GPS into TinyGPS++
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Print update every second
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();

    // Check fix status
    if (gps.location.isValid() && gps.satellites.value() > 0) {
      if (!hasFix) {
        Serial.println(F("\n✅ GPS Fix Acquired!\n"));
        hasFix = true;
      }

      Serial.println(F("------ GPS Data ------"));
      Serial.print(F("Latitude:  "));
      Serial.println(gps.location.lat(), 6);
      Serial.print(F("Longitude: "));
      Serial.println(gps.location.lng(), 6);
      Serial.print(F("Satellites: "));
      Serial.println(gps.satellites.value());
      Serial.print(F("Altitude (m): "));
      Serial.println(gps.altitude.meters());
      Serial.print(F("HDOP: "));
      Serial.println(gps.hdop.hdop());
      Serial.println(F("----------------------\n"));
    } 
    else {
      if (!hasFix) {
        Serial.println(F("Searching for satellites..."));
      } else {
        Serial.println(F("⚠️ Lost GPS fix — reacquiring..."));
        hasFix = false;
      }
    }
  }
}
