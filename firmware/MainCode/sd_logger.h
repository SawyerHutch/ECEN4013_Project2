#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include "gps.h"
#include "imu.h"

void sd_init();
void sd_log(const GPSData &gps, const IMUData &imu);

#endif
