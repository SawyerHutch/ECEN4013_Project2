#include "radio_tx.h"

void radio_init() {
    Serial.println("Radio TX initialized");
}

void radio_send(const GPSData &gps, const IMUData &imu) {
    Serial.println("Sending telemetry via radio (stub)");
}
