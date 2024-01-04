#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include "configs.h"

// Choose the sensors to use
#define USE_ICM20948_SPI      // For Single Agent Avionic board
// #define USE_ICM20948_I2C   // Not implemented yet
// #define USE_MPU6050        // Using I2C for MPU6050

#define USE_IMU_INT
#define USE_IMU_DMP

#define USE_BMP280_SPI
// #define USE_BMP280_I2C

#include <Adafruit_BMP280.h>

#ifdef USE_MPU6050
  #include <Adafruit_MPU6050.h>
#elif defined(USE_ICM20948_SPI)
  #include "ICM_20948.h"
#else 
  #error "No IMU selected"
#endif

#include <Adafruit_Sensor.h>
#ifdef USE_BMP280_I2C
#include <Wire.h>
#endif

#include "freertos/semphr.h"
#include "CommProtocol.h"
#include "PacketTypes.h"

#define PRESSURE_TO_ALTITUDE(PRESSURE, SEA_LEVEL_HPA)                          \
  (44330 * (1.0 - pow(((PRESSURE) / 100) / (SEA_LEVEL_HPA), 0.1903)))

class Sensors {
public:

  enum SENSOR_ERROR {
    SENSOR_OK = 0,
    IMU_INIT_ERROR,
    DMP_INIT_ERROR,
    BARO_INIT_ERROR,
  };

#ifdef USE_MPU6050
  Adafruit_MPU6050 imu;
#elif defined(USE_ICM20948_SPI)
  ICM_20948_SPI imu;
#endif

  Adafruit_BMP280 baro;

  Sensors();

  int init(int update_rate = 50);

  void state_packet_gen(StatePacket *const _packet);

  SensorData *const get_sensor_data() {
    return &_data;
  };

  bool available() { return _sensor_updated; };

  void update();

protected:
  SemaphoreHandle_t _data_mutex;
  SensorData _data;

private:
  uint8_t _update_interval; // ms
  bool _sensor_updated;

  bool imu_enabled;
  bool baro_enabled;

  static volatile bool isrFired;
  static volatile bool sensorSleep;
  static volatile bool canToggle;

  int imu_init();
  static void imu_int_handler();
};

#endif
