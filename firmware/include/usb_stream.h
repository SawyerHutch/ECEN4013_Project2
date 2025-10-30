#ifndef USB_STREAM_H
#define USB_STREAM_H

#include <Arduino.h>
#include "gps.h"
#include "imu.h"

void usb_init();
void usb_send(const GPSData &gps, const IMUData &imu);

#endif
