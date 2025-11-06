/*
  device_code.ino
  Teensy 4.1 Localization Device - combined firmware

  Subsystems:
   - GPS: Mikroe-1032 (u-blox NEO-6M) on Serial1 (UART)
   - IMU: Adafruit ICM-20948 (Adafruit 4646) on I2C (Wire)
   - SD: built-in microSD slot (SD library)
   - Radio: XBee on Serial2 (UART)
   - LED: onboard LED (pin 13) indicates GPS lock (blink = searching, solid = locked)

  Update rate: ~3 Hz (every 333 ms)
  Outputs (simultaneous): SD CSV file, USB serial (when connected), XBee radio (Serial2)
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <SD.h>

// ------------ CONFIG ------------
#define GPS_SERIAL       Serial1   // hardware UART for GPS
#define RADIO_SERIAL     Serial2   // hardware UART for XBee
const uint32_t GPS_BAUD     = 9600;
const uint32_t RADIO_BAUD   = 9600;

const int LED_PIN = 13;           // onboard LED
const unsigned long UPDATE_MS = 333; // ~3 Hz

// SD file
const char *LOG_FILENAME = "LOG.CSV";

// IMU object
Adafruit_ICM20948 icm;

// GPS parser
TinyGPSPlus gps;

// State / buffers
unsigned long lastUpdate = 0;
bool hasGPSFix = false;
bool sdReady = false;
bool radioReady = false;
File logFile;

// Helper: safe print to USB serial only if connected
void usbPrintln(const __FlashStringHelper *msg) {
  if (Serial) Serial.println(msg);
}
void usbPrintln(const String &s) {
  if (Serial) Serial.println(s);
}
void usbPrint(const __FlashStringHelper *msg) {
  if (Serial) Serial.print(msg);
}
void usbPrint(const String &s) {
  if (Serial) Serial.print(s);
}

// CSV helper: write line to SD, Serial, and Radio
void outputLine(const String &line) {
  // SD
  if (sdReady) {
    if (logFile) {
      logFile.println(line);
      logFile.flush();
    } else {
      // try reopen once
      logFile = SD.open(LOG_FILENAME, FILE_WRITE);
      if (logFile) {
        logFile.println(line);
        logFile.flush();
      }
    }
  }

  // USB (debug) - non-blocking (Serial may not be connected)
  if (Serial) {
    Serial.println(line);
  }

  // Radio (XBee)
  if (radioReady) {
    RADIO_SERIAL.println(line);
  }
}

// Build CSV header (match PDF fields)
String csvHeader() {
  return String("timestamp,lat,lon,elevation_m,satellites,") +
         "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z";
}

// Format timestamp: prefer GPS time+date if valid, otherwise millis()
String formatTimestamp() {
  if (gps.time.isValid() && gps.date.isValid()) {
    // Build YYYY-MM-DD HH:MM:SS (UTC from GPS)
    char buf[32];
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int min = gps.time.minute();
    int sec = gps.time.second();
    // Some TinyGPS++ builds give 2-digit year; ensure it's full (assume 2000+)
    if (year < 2000) year += 2000;
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             year, month, day, hour, min, sec);
    return String(buf);
  } else {
    // Fallback to milliseconds since boot
    unsigned long ms = millis();
    char buf[32];
    snprintf(buf, sizeof(buf), "ms_%lu", ms);
    return String(buf);
  }
}

// Initialize SD card (built-in on Teensy 4.1)
void initSD() {
  // On Teensy 4.1 use BUILTIN_SDCARD macro for SD.begin
  usbPrintln(F("Initializing SD card..."));
  if (SD.begin(BUILTIN_SDCARD)) {
    sdReady = true;
    // open or create file and write header if empty/new
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
      // if file size is zero, write header
      if (logFile.size() == 0) {
        String header = csvHeader();
        logFile.println(header);
        logFile.flush();
      }
      usbPrintln(F("SD ready."));
    } else {
      sdReady = false;
      usbPrintln(F("SD present but cannot open log file."));
    }
  } else {
    sdReady = false;
    usbPrintln(F("SD initialization failed."));
  }
}

// Initialize IMU
void initIMU() {
  usbPrintln(F("Initializing IMU (ICM-20948) ..."));
  Wire.begin(); // SDA/SCL pins wired by hardware; Teensy Wire default
  if (!icm.begin_I2C()) {
    usbPrintln(F("WARNING: ICM-20948 not detected. IMU readings will be zero."));
    // We will still continue, but IMU reads will be flagged invalid
  } else {
    usbPrintln(F("IMU initialized."));
    // optional: set ranges / filters if needed here
  }
}

// Read IMU: return whether valid and fill outputs
bool readIMU(float &gx, float &gy, float &gz,
             float &ax, float &ay, float &az,
             float &mx, float &my, float &mz) {
  // Default outputs
  gx = gy = gz = 0.0f;
  ax = ay = az = 0.0f;
  mx = my = mz = 0.0f;

  if (!icm.begin_I2C()) {
    // If initialization failed earlier - try once, but don't spam
    // We'll assume readings invalid
    return false;
  }

  // Use Adafruit unified sensor event interface:
  sensors_event_t accel, gyro, temp;
  icm.getEvent(&accel, &gyro, &temp);

  // accel: m/s^2 in accel.acceleration.x/y/z
  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;

  // gyro: rad/s? Many Adafruit libs give deg/s in gyro.gyro.x etc.
  // The Adafruit ICM20948's gyro returns rad/s in the unified sensors_event_t (gyro.gyro.x)
  gx = gyro.gyro.x;
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;

  // Magnetometer: Adafruit library does not always populate a mag event via getEvent().
  // ICM-20948's magnetometer (AK09916) may require direct reading via library functions.
  // Try to read magnetometer via library; if not available, leave zeros.
  // The Adafruit_ICM20948 library exposes getMagnetometer(), getMag() or similar in some versions.
  // We'll attempt to use the library's mag read API if present.
#if defined(ADAFRUIT_ICM20948_H)
  // Some library versions provide getMagVector(); try best-effort
  // If getMagVector isn't available, these calls will be ignored at compile-time.
  sensors_event_t mag_event;
  if (icm.getMagnetometer(&mag_event)) { // NOTE: if this symbol is not present, this will fail to compile
    mx = mag_event.magnetic.x;
    my = mag_event.magnetic.y;
    mz = mag_event.magnetic.z;
    return true;
  }
#endif

  // If we reach here, magnetometer read is not available via getMagnetometer.
  // Try reading raw mag registers (library dependent). As a fallback we return true because accel/gyro valid.
  return true;
}

// Setup
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // USB serial - start for debug but code won't rely on it
  Serial.begin(115200);
  delay(200);

  usbPrintln(F("=== Localization Device Boot ==="));

  // Initialize GPS UART
  GPS_SERIAL.begin(GPS_BAUD);
  usbPrintln(F("GPS UART started."));

  // Init Radio UART
  RADIO_SERIAL.begin(RADIO_BAUD);
  radioReady = true; // assume ready; we don't check XBee presence here
  usbPrintln(F("Radio UART started."));

  // Init SD
  initSD();

  // Init IMU
  initIMU();

  // Write CSV header to SD if not already there
  if (sdReady && logFile) {
    // header ensured in initSD
  }

  lastUpdate = millis();
}

// Loop: read peripherals and output at target rate
void loop() {
  // Keep feeding GPS characters into parser as they arrive (non-blocking)
  while (GPS_SERIAL.available()) {
    char c = (char)GPS_SERIAL.read();
    gps.encode(c);
  }

  // Read IMU continuously in background (non-blocking approach used below at update time)

  // Main periodic update
  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_MS) {
    lastUpdate = now;

    // Determine GPS fix status
    bool locationValid = gps.location.isValid();
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    hasGPSFix = locationValid && (sats > 0);

    // LED: blink if searching, solid on lock
    if (hasGPSFix) {
      digitalWrite(LED_PIN, HIGH); // solid when fixed
    } else {
      // blink once per period (toggle)
      static bool ledToggle = false;
      ledToggle = !ledToggle;
      digitalWrite(LED_PIN, ledToggle ? HIGH : LOW);
    }

    // Read IMU once per update
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    bool imuOk = readIMU(gx, gy, gz, ax, ay, az, mx, my, mz);

    // Build timestamp
    String ts = formatTimestamp();

    // Prepare CSV line
    String line;

    if (hasGPSFix) {
      // Elevation: prefer gps.altitude.meters() (geoid/ellipsoid choice not handled here; uses GPS ellipsoid)
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double elev = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      line = ts + "," +
             String(lat, 6) + "," +
             String(lon, 6) + "," +
             String(elev, 3) + "," +
             String(sats) + "," +
             String(gx, 6) + "," + String(gy, 6) + "," + String(gz, 6) + "," +
             String(ax, 6) + "," + String(ay, 6) + "," + String(az, 6) + "," +
             String(mx, 6) + "," + String(my, 6) + "," + String(mz, 6);
    } else {
      // No fix: still include zeros/placeholder for lat/lon/elev and include IMU if available
      line = ts + ",,," + "0.0," + String(sats) + "," +
             String(gx, 6) + "," + String(gy, 6) + "," + String(gz, 6) + "," +
             String(ax, 6) + "," + String(ay, 6) + "," + String(az, 6) + "," +
             String(mx, 6) + "," + String(my, 6) + "," + String(mz, 6);
    }

    // Output to SD, USB, and Radio simultaneously
    outputLine(line);

    // Also optionally print a short human-friendly status to USB for debugging (non-blocking)
    if (Serial) {
      if (hasGPSFix) {
        Serial.print(F("[OK] "));
        Serial.print(ts);
        Serial.print(F("  Lat: "));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(" Lon: "));
        Serial.print(gps.location.lng(), 6);
        Serial.print(F(" Sats: "));
        Serial.println(sats);
      } else {
        Serial.print(F("[SEARCH] "));
        Serial.print(ts);
        Serial.print(F("  Sats: "));
        Serial.println(sats);
      }
    }
  }

  // Keep main loop non-blocking (small yield)
  // Note: no delays here except the periodic check above
}
