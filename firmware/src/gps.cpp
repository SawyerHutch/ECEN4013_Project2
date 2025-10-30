#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGPS;

void gps_init() {
    Wire.begin();
    if (!myGPS.begin()) Serial.println("u-blox GPS not detected!");
}

bool gps_read(GPSData &data) {
    if (myGPS.getPVT()) {
        data.latitude  = myGPS.getLatitude() / 1e7;
        data.longitude = myGPS.getLongitude() / 1e7;
        data.altitude  = myGPS.getAltitude() / 1000.0;
        data.fixQuality = myGPS.getFixType();
        return true;
    }
    return false;
}
