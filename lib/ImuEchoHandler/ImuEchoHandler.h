#pragma once 

#include "configs.h"
#include <functional>
#include <Wire.h>
#include "freertos/semphr.h"

#include "SensorDriver.h"

// Using i2c bus 2
#define IMU_ECHO_I2C_BUS Wire1

class ImuEchoHandler {
public:
    ImuEchoHandler() {};

    void init();

    void set_imu_data(SensorData *const _data) {
        IMU_ECHO_I2C_BUS.slaveWrite((uint8_t*)_data, sizeof(SensorData));
    }

private:
    static void onReceive(int numBytes) {
        while(IMU_ECHO_I2C_BUS.available()){
            IMU_ECHO_I2C_BUS.read();
        }
    };

    static void onRequest() {
    };
};

