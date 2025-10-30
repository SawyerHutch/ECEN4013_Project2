#include <SdFat.h>
SdFat sd;
File logFile;

void sd_init() {
    if (!sd.begin(BUILTIN_SDCARD)) Serial.println("SD init failed!");
    logFile = sd.open("data.csv", FILE_WRITE);
    logFile.println("lat,lon,ax,ay,az");
}

void sd_log(const GPSData &gps, const IMUData &imu) {
    if (logFile) {
        logFile.print(gps.latitude, 6); logFile.print(',');
        logFile.print(gps.longitude, 6); logFile.print(',');
        logFile.print(imu.accelX, 3); logFile.print(',');
        logFile.print(imu.accelY, 3); logFile.print(',');
        logFile.println(imu.accelZ, 3);
        logFile.flush();
    }
}
