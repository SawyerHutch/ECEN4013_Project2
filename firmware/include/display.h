#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "gps.h"
#include "imu.h"

void display_init();
void display_update(const GPSData &gps, const IMUData &imu);

#endif
