#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    uint8_t fixQuality;
};

void gps_init();
bool gps_read(GPSData &data);
void gps_print(const GPSData &data);

#endif
