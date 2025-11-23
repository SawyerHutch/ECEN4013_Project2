#include <TinyGPSPlus.h>

TinyGPSPlus gps;
unsigned long lastUpdate = 0;
const int LED_PIN = 13;  // Teensy onboard LED
bool ledState = false;

void setup() {
  Serial.begin(115200);     // USB serial (to computer)
  Serial1.begin(9600);      // GPS UART (MIKROE-1032)
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Starting GPS test...");
  Serial.println("Waiting for GPS data...\n");
}

void loop() {
  // Read GPS data from UART
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Update every 2 seconds
  if (millis() - lastUpdate >= 2000) {
    lastUpdate = millis();

    Serial.println("----- GPS STATUS -----");

    // LED logic:
    // - Blink if no fix
    // - Solid ON if fixed
    if (gps.location.isValid() && gps.satellites.value() > 0) {
      digitalWrite(LED_PIN, HIGH);  // solid ON when fix
    } else {
      ledState = !ledState;         // blink when no fix
      digitalWrite(LED_PIN, ledState);
    }

    // GPS info output
    if (gps.location.isValid()) {
      Serial.print("Latitude:  ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Location:  No fix yet");
    }

    if (gps.altitude.isValid()) {
      Serial.print("Altitude:  ");
      Serial.print(gps.altitude.meters());
      Serial.println(" m");
    }

    if (gps.satellites.isValid()) {
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
    } else {
      Serial.println("Satellites: Searching...");
    }

    if (gps.hdop.isValid()) {
      Serial.print("HDOP: ");
      Serial.println(gps.hdop.value());
    }

    Serial.println("----------------------\n");
  }
}
