#include "usb_stream.h"

void usb_init() {
    Serial.println("USB serial streaming initialized");
}

void usb_send(const GPSData &gps, const IMUData &imu) {
    Serial.print("USB,");
    Serial.print(gps.latitude, 6); Serial.print(",");
    Serial.print(gps.longitude, 6); Serial.print(",");
    Serial.print(imu.accelX, 3); Serial.print(",");
    Serial.print(imu.accelY, 3); Serial.print(",");
    Serial.println(imu.accelZ, 3);
}
