#pragma once 

#include "configs.h"
#include <functional>
#include <Wire.h>

#include "SensorDriver.h"

// Using i2c bus 2
#define IMU_ECHO_I2C_BUS Wire1

class ImuEchoHandler {
public:
    ImuEchoHandler() {};

    void init() {
        Serial.setDebugOutput(true);
        IMU_ECHO_I2C_BUS.onReceive(onReceive);
        IMU_ECHO_I2C_BUS.onRequest(onRequest);

        IMU_ECHO_I2C_BUS.setBufferSize(I2C_BUFFER_SIZE);

        // uint8_t slaveAddr, int sda, int scl, uint32_t frequency
        IMU_ECHO_I2C_BUS.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_FREQ);
        log_i("ImuEchoHandler initialized");
    }

    void set_imu_data(SensorData *const _data) {
        IMU_ECHO_I2C_BUS.slaveWrite((uint8_t*)_data, sizeof(SensorData));
    }
private:
    static void onReceive(int numBytes) {
        while(IMU_ECHO_I2C_BUS.available()){
        //     Serial.write(IMU_ECHO_I2C_BUS.read());
        }
    };

    static void onRequest() {
        IMU_ECHO_I2C_BUS.print("Recv");
    };
};