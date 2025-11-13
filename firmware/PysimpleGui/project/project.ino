#include <TinyGPSPlus.h>

TinyGPSPlus gps;
HardwareSerial &gpsSerial = Serial1;

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  // RESET GPS
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  delay(100);
  digitalWrite(9, HIGH);

  gpsSerial.begin(9600);
  Serial.println("Teensy GPS ready - COLD START");
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
    gps.encode(c);
  }

  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    Serial.print("SATS_IN_VIEW: ");
    Serial.println(gps.satellites.value());
    last = millis();
  }

  if (gps.location.isUpdated()) {
    Serial.print("LAT,");  Serial.print(gps.location.lat(), 6);
    Serial.print(",LON,"); Serial.print(gps.location.lng(), 6);
    Serial.print(",ALT,"); Serial.print(gps.altitude.meters(), 1);
    Serial.print(",SATS,"); Serial.println(gps.satellites.value());
  }

  delay(10);
}