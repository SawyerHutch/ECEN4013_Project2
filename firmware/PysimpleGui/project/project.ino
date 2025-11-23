#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <SPI.h>

// ----- GPS setup -----
TinyGPSPlus gps;
unsigned long lastUpdate = 0;
const int LED_PIN = 13;  // Teensy onboard LED
bool ledState = false;

// ----- IMU setup -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // I2C address 0x28 (default)

// ----- SD Card setup -----
const int SD_CS_PIN = BUILTIN_SDCARD;
File logFile;

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

  // Initialize SD card
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Open or create log file
  logFile = SD.open("datalog.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("Could not open datalog.csv!");
    while (1);
  }

  // Write header if file is empty
  if (logFile.size() == 0) {
    logFile.println(
      "Date,Time,Satellites,Latitude,Longitude,Elevation MSL (m),"
      "X Accel (m/s^2),Y Accel (m/s^2),Z Accel (m/s^2),"
      "X Mag (uT),Y Mag (uT),Z Mag (uT),"
      "X Gyro (rps),Y Gyro (rps),Z Gyro (rps)"
    );
    logFile.flush();
  }
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
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    Serial.println("----- IMU STATUS -----");
    Serial.print("Gyroscope (rad/s): ");
    Serial.print(gyro.x()); Serial.print(", ");
    Serial.print(gyro.y()); Serial.print(", ");
    Serial.println(gyro.z());

    Serial.print("Acceleration (m/s^2): ");
    Serial.print(accel.x()); Serial.print(", ");
    Serial.print(accel.y()); Serial.print(", ");
    Serial.println(accel.z());

    Serial.print("Magnetic field (uT): ");
    Serial.print(mag.x()); Serial.print(", ");
    Serial.print(mag.y()); Serial.print(", ");
    Serial.println(mag.z());
    Serial.println("----------------------");


    // ----- ✅ Machine-readable output for Python GUI -----
    Serial.print("LAT,"); Serial.print(gps.location.isValid() ? gps.location.lat() : 0, 6);
    Serial.print(",LON,"); Serial.print(gps.location.isValid() ? gps.location.lng() : 0, 6);
    Serial.print(",ALT,"); Serial.print(gps.altitude.isValid() ? gps.altitude.meters() : 0, 1);
    Serial.print(",SATS,"); Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.print(",GX,"); Serial.print(gyro.x(), 3);
    Serial.print(",GY,"); Serial.print(gyro.y(), 3);
    Serial.print(",GZ,"); Serial.print(gyro.z(), 3);
    Serial.print(",AX,"); Serial.print(accel.x(), 3);
    Serial.print(",AY,"); Serial.print(accel.y(), 3);
    Serial.print(",AZ,"); Serial.print(accel.z(), 3);
    Serial.print(",MX,"); Serial.print(mag.x(), 3);
    Serial.print(",MY,"); Serial.print(mag.y(), 3);
    Serial.print(",MZ,"); Serial.println(mag.z(), 3);


    // ----- ✅ CSV Logging on SD Card -----
    // Prepare date and time strings (use GPS date/time if valid, else fallback)
    String dateStr = gps.date.isValid() ? 
      String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year()) : "NA";
    
    String timeStr = gps.time.isValid() ? 
      String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) : "NA";

    // Build CSV row string matching header
    String row =
      dateStr + "," + timeStr + "," +
      (gps.satellites.isValid() ? String(gps.satellites.value()) : "0") + "," +
      (gps.location.isValid() ? String(gps.location.lat(), 6) : "0") + "," +
      (gps.location.isValid() ? String(gps.location.lng(), 6) : "0") + "," +
      (gps.altitude.isValid() ? String(gps.altitude.meters(), 2) : "0") + "," +

      String(accel.x(), 3) + "," + String(accel.y(), 3) + "," + String(accel.z(), 3) + "," +
      String(mag.x(), 3) + "," + String(mag.y(), 3) + "," + String(mag.z(), 3) + "," +
      String(gyro.x(), 3) + "," + String(gyro.y(), 3) + "," + String(gyro.z(), 3);

    logFile.println(row);

    // Flush every 2 seconds for file integrity
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 2000) {
      logFile.flush();
      lastFlush = millis();
    }
  }
}
