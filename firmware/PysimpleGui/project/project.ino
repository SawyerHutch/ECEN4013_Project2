#include <TinyGPSPlus.h>

TinyGPSPlus gps;
unsigned long lastUpdate = 0;
const int LED_PIN = 13;  // Teensy onboard LED

void setup() {
  Serial.begin(115200);     // USB serial (to PC)
  Serial1.begin(9600);      // GPS UART
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Teensy GPS starting...");
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (millis() - lastUpdate >= 1000) {  // update every second
    lastUpdate = millis();

    bool fix = gps.location.isValid() && gps.satellites.value() > 0;
    digitalWrite(LED_PIN, fix ? HIGH : LOW);

    // Output in machine-readable format
    if (fix) {
      Serial.print("LAT,");  Serial.print(gps.location.lat(), 6);
      Serial.print(",LON,"); Serial.print(gps.location.lng(), 6);
      Serial.print(",ALT,"); Serial.print(gps.altitude.meters(), 1);
      Serial.print(",SATS,"); Serial.print(gps.satellites.value());
      Serial.print(",HDOP,"); Serial.println(gps.hdop.value());
    } else {
      Serial.println("LAT,0,LON,0,ALT,0,SATS,0,HDOP,0");
    }
  }
}
