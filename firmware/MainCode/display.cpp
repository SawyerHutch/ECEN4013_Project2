#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); // example pins

void display_init() {
    lcd.begin(16, 2);
    lcd.print("Device Ready");
}

void display_update(const GPSData &gps, const IMUData &imu) {
    lcd.setCursor(0, 1);
    lcd.print(gps.latitude, 2);
    lcd.print(",");
    lcd.print(gps.longitude, 2);
}
