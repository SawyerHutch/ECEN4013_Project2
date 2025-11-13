#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>

// ----- GPS setup -----
TinyGPSPlus gps;
unsigned long lastUpdate = 0;
const int LED_PIN = 13;  // Teensy onboard LED
bool ledState = false;

// ----- IMU setup -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // I2C address 0x28 (default)

void setup() {
  Serial.begin(115200);     // USB serial (to computer)
  Serial1.begin(9600);      // GPS UART (to GPS module)
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Starting GPS + IMU...");

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1);
  }

  delay(1000); // allow IMU to initialize
  bno.setExtCrystalUse(true); // optional: improves accuracy
}

void loop() {
  // ----- GPS reading -----
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // ----- Update every 2 seconds -----
  if (millis() - lastUpdate >= 2000) {
    lastUpdate = millis();

    // LED logic for GPS fix
    bool fix = gps.location.isValid() && gps.satellites.value() > 0;
    digitalWrite(LED_PIN, fix ? HIGH : LOW);

    // ----- Debug Info for Serial Monitor -----
    Serial.println("----- GPS STATUS -----");
    if (gps.location.isValid()) {
      Serial.print("Latitude:  "); Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Location: No fix yet");
    }

    if (gps.altitude.isValid()) {
      Serial.print("Altitude:  "); Serial.print(gps.altitude.meters()); Serial.println(" m");
    }

    if (gps.satellites.isValid()) {
      Serial.print("Satellites: "); Serial.println(gps.satellites.value());
    } else {
      Serial.println("Satellites: Searching...");
    }

    Serial.println("----------------------");

    // ----- IMU Reading -----
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.println("----- IMU STATUS -----");
    Serial.print("Orientation: ");
    Serial.print(event.orientation.x); Serial.print(", ");
    Serial.print(event.orientation.y); Serial.print(", ");
    Serial.println(event.orientation.z);
    Serial.println("----------------------");

    // ----- ✅ Machine-readable output for Python GUI -----
    Serial.print("LAT,"); Serial.print(gps.location.isValid() ? gps.location.lat() : 0, 6);
    Serial.print(",LON,"); Serial.print(gps.location.isValid() ? gps.location.lng() : 0, 6);
    Serial.print(",ALT,"); Serial.print(gps.altitude.isValid() ? gps.altitude.meters() : 0, 1);
    Serial.print(",SATS,"); Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.print(",HDOP,"); Serial.print(gps.hdop.isValid() ? gps.hdop.value() : 0);
    Serial.print(",AX,"); Serial.print(event.orientation.x, 2);
    Serial.print(",AY,"); Serial.print(event.orientation.y, 2);
    Serial.print(",AZ,"); Serial.println(event.orientation.z, 2);
  }
}
