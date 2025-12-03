#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <SPI.h>

// ----- GPS setup -----
TinyGPSPlus gps;
unsigned long lastUpdate = 0;
unsigned long lastBlink = 0;
const int LED_PIN = 28;  // PCB LED
bool ledState = false;
bool fix = false;

// ----- IMU setup -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // I2C address 0x28 (default)

// ----- SD Card setup -----
const int SD_CS_PIN = BUILTIN_SDCARD;
File logFile;

// ----- Xbee3 setup -----
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;
unsigned long counter = 0;

// ----- Data Storage ------
struct SensorData {
  bool gpsFix;
  uint32_t numSatLocked;
  double longitude;
  double latitude;
  double altitude;
  double gyroX;
  double gyroY;
  double gyroZ;
  double accelX;
  double accelY;
  double accelZ;
  double magX;
  double magY;
  double magZ;
  String dateStr;
  String timeStr;
};

SensorData sensorPacket;

void initSensorData(SensorData &d){
  d.gpsFix       = false;
  d.numSatLocked = 0;

  d.longitude    = 0.0;
  d.latitude     = 0.0;
  d.altitude     = 0.0;

  d.gyroX        = 0.0;
  d.gyroY        = 0.0;
  d.gyroZ        = 0.0;

  d.accelX       = 0.0;
  d.accelY       = 0.0;
  d.accelZ       = 0.0;

  d.magX         = 0.0;
  d.magY         = 0.0;
  d.magZ         = 0.0;

  d.dateStr      = "";
  d.timeStr      = "";
}

void gpsUpdate(SensorData &d){
  d.longitude = gps.location.lng();
  d.latitude = gps.location.lat();
  d.altitude = gps.altitude.meters();
  d.numSatLocked = gps.satellites.value();
  d.gpsFix = true;
}

void imuUpdate(SensorData &d){
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  d.gyroX = gyro.x();
  d.gyroY = gyro.y();
  d.gyroZ = gyro.z();
  d.accelX = accel.x();
  d.accelY = accel.y();
  d.accelZ = accel.z();
  d.magX = mag.x();
  d.magY = mag.y();
  d.magZ = mag.z();
}

void timeUpdate(SensorData &d){
    d.dateStr = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
    d.timeStr =  String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
}

void transmitGUI(SensorData &sensorPacket) {
  Serial.print("\t Starting GUI transmit...");
  // ----- ✅ Machine-readable output for Python GUI -----
  Serial.print("LAT,");
  Serial.print(sensorPacket.gpsFix ? sensorPacket.latitude : 0.0, 6);
  Serial.print(",LON,");
  Serial.print(sensorPacket.gpsFix ? sensorPacket.longitude : 0.0, 6);
  Serial.print(",ALT,");
  Serial.print(sensorPacket.gpsFix ? sensorPacket.altitude : 0.0, 1);
  Serial.print(",SATS,");
  Serial.print(sensorPacket.numSatLocked);
  Serial.print(",GX,");
  Serial.print(sensorPacket.gyroX, 3);
  Serial.print(",GY,");
  Serial.print(sensorPacket.gyroY, 3);
  Serial.print(",GZ,");
  Serial.print(sensorPacket.gyroZ, 3);
  Serial.print(",AX,");
  Serial.print(sensorPacket.accelX, 3);
  Serial.print(",AY,");
  Serial.print(sensorPacket.accelY, 3);
  Serial.print(",AZ,");
  Serial.print(sensorPacket.accelZ, 3);
  Serial.print(",MX,");
  Serial.print(sensorPacket.magX, 3);
  Serial.print(",MY,");
  Serial.print(sensorPacket.magY, 3);
  Serial.print(",MZ,");
  Serial.println(sensorPacket.magZ, 3);
  Serial.println("\t ...done.");
}
void transmitXB(SensorData &sensorPacket){
  Serial.println("\t Starting transmit to XBee3...");

  // Transmitting Data in the order: GPS -> IMU -> Anything else
  Serial2.print("gpsFix: ");
  Serial2.println(sensorPacket.gpsFix);

  Serial2.print("numSatLocked: ");
  Serial2.println(sensorPacket.numSatLocked);

  Serial2.print("longitude: ");
  Serial2.println(sensorPacket.longitude, 6);

  Serial2.print("latitude: ");
  Serial2.println(sensorPacket.latitude, 6);

  Serial2.print("altitude: ");
  Serial2.println(sensorPacket.altitude, 2);

  Serial2.print("gyroX: ");
  Serial2.println(sensorPacket.gyroX, 3);

  Serial2.print("gyroY: ");
  Serial2.println(sensorPacket.gyroY, 3);

  Serial2.print("gyroZ: ");
  Serial2.println(sensorPacket.gyroZ, 3);

  Serial2.print("accelX: ");
  Serial2.println(sensorPacket.accelX, 3);

  Serial2.print("accelY: ");
  Serial2.println(sensorPacket.accelY, 3);

  Serial2.print("accelZ: ");
  Serial2.println(sensorPacket.accelZ, 3);

  Serial2.print("magX: ");
  Serial2.println(sensorPacket.magX, 3);

  Serial2.print("magY: ");
  Serial2.println(sensorPacket.magY, 3);

  Serial2.print("magZ: ");
  Serial2.println(sensorPacket.magZ, 3);

  Serial2.print("dateStr: ");
  Serial2.println(sensorPacket.dateStr);

  Serial2.print("timeStr: ");
  Serial2.println(sensorPacket.timeStr);
 
  Serial.println("\t ...done.");
}
void transmitSD(SensorData &sensorPacket){

    Serial.println("\t Starting transmit to SD...");
    // Build CSV row string matching header
    String row =
      sensorPacket.dateStr + "," + sensorPacket.timeStr + "," +
      String(sensorPacket.numSatLocked)  + "," +
      String(sensorPacket.latitude)  + "," +
      String(sensorPacket.longitude)  + "," +
      String(sensorPacket.altitude)  + "," +
      String(sensorPacket.accelX) + "," + String(sensorPacket.accelY) + "," + String(sensorPacket.accelZ) + "," +
      String(sensorPacket.magX) + "," + String(sensorPacket.magY) + "," + String(sensorPacket.magZ) + "," +
      String(sensorPacket.gyroX) + "," + String(sensorPacket.gyroY) + "," + String(sensorPacket.gyroZ);

    logFile.println(row);

    Serial.println("\t ...done.");
    // Flush every 2 seconds for file integrity
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 2000) {
      logFile.flush();
      lastFlush = millis();
    }
}

void transmit(SensorData &sensorPacket){
  Serial.println("Starting Transmit...");
  transmitSD(sensorPacket);
  transmitXB(sensorPacket);
  transmitGUI(sensorPacket);
  Serial.println("Transmit done.");
}

void setup() {
  initSensorData(sensorPacket);
  Serial.begin(115200);     // USB serial (to computer)
  Serial1.begin(9600);      // GPS UART (to GPS module)
  Serial2.begin(9600);      // Radio UART (to Xbee3)
 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Starting GPS...");
  Serial.println("Starting IMU...");
  Serial.println("Starting Xbee...");

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

  // ----- SATELLITE LOCK LED -----
  if (!sensorPacket.gpsFix) {
    unsigned long now = millis();
    // Blink every 300 ms
    if (now - lastBlink >= 500) {
      lastBlink = now;
      // toggle LED state
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
  else {
    // if fix becomes true, LED goes solid ON (or OFF, your choice)
    digitalWrite(LED_PIN, HIGH);   // or LOW
  }

  // ----- MAIN UPDATE LOOP (2 Sec) -----
  if (millis() - lastUpdate >= 2000) {
    lastUpdate = millis();
   

    Serial.println("----- GPS STATUS -----");
    bool fix = gps.location.isValid();
    if(fix){
      gpsUpdate(sensorPacket);
      Serial.print("Latitude:  "); Serial.println(sensorPacket.latitude);
      Serial.print("Longitude: "); Serial.println(sensorPacket.longitude);
      Serial.print("Altitude:  "); Serial.print(sensorPacket.altitude); Serial.println(" m");
      Serial.print("Satellites: "); Serial.println(sensorPacket.numSatLocked);
    }
    else{
      sensorPacket.gpsFix = false;
      Serial.println("Satellites: Searching...");
      Serial.println("Location: No fix yet");
    }

    Serial.println("----------------------");
   
    // ----- IMU Reading -----

    Serial.println("----- IMU STATUS -----");
    imuUpdate(sensorPacket);
    Serial.print("Gyroscope (rad/s): ");
    Serial.print(sensorPacket.gyroX); Serial.print(", ");
    Serial.print(sensorPacket.gyroY); Serial.print(", ");
    Serial.println(sensorPacket.gyroZ);

    Serial.print("Acceleration (m/s^2): ");
    Serial.print(sensorPacket.accelX); Serial.print(", ");
    Serial.print(sensorPacket.accelY); Serial.print(", ");
    Serial.println(sensorPacket.accelZ);

    Serial.print("Magnetic field (uT): ");
    Serial.print(sensorPacket.magX); Serial.print(", ");
    Serial.print(sensorPacket.magY); Serial.print(", ");
    Serial.println(sensorPacket.magZ);
    Serial.println("----------------------");

    timeUpdate(sensorPacket);

    // ----- Transmit to all sources -----
    transmit(sensorPacket);
  }
}