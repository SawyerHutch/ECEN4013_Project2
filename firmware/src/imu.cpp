#include <Adafruit_ICM20948.h>
Adafruit_ICM20948 icm;

void imu_init() {
    if (!icm.begin_I2C()) Serial.println("IMU not detected!");
}

void imu_read(IMUData &data) {
    sensors_event_t accel, gyro, temp;
    icm.getEvent(&accel, &gyro, &temp);
    data.accelX = accel.acceleration.x;
    data.accelY = accel.acceleration.y;
    data.accelZ = accel.acceleration.z;
    data.gyroX  = gyro.gyro.x;
    data.gyroY  = gyro.gyro.y;
    data.gyroZ  = gyro.gyro.z;
}
