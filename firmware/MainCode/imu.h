#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct IMUData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
};

void imu_init();
void imu_read(IMUData &data);

#endif
