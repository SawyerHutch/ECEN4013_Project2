#ifndef RADIO_TX_H
#define RADIO_TX_H

#include <Arduino.h>
#include "gps.h"
#include "imu.h"

void radio_init();
void radio_send(const GPSData &gps, const IMUData &imu);

#endif
