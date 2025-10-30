#include <Arduino.h>
#include "config.h"
#include "gps.h"
#include "imu.h"
#include "sd_logger.h"
#include "usb_stream.h"
#include "radio_tx.h"
#include "display.h"

GPSData gpsData;
IMUData imuData;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Localization Device Booting ===");

    gps_init();
    imu_init();
    sd_init();
    usb_init();
    radio_init();
    display_init();

    Serial.println("=== Initialization Complete ===");
}

void loop() {
    // Update sensors
    if (gps_read(gpsData))
        gps_print(gpsData);

    imu_read(imuData);

    // Log to SD
    sd_log(gpsData, imuData);

    // Send over USB + radio
    usb_send(gpsData, imuData);
    radio_send(gpsData, imuData);

    // Update display
    display_update(gpsData, imuData);

    delay(200);
}
