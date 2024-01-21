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
#include "Perf.hpp"

#define PRESSURE_TO_ALTITUDE(PRESSURE, SEA_LEVEL_HPA)                          \
  (44330 * (1.0 - pow(((PRESSURE) / 100) / (SEA_LEVEL_HPA), 0.1903)))

constexpr uint16_t const_ceil(float num)
{
    return (static_cast<float>(static_cast<uint16_t>(num)) == num)
        ? static_cast<uint16_t>(num)
        : static_cast<uint16_t>(num) + ((num > 0) ? 1 : 0);
}

class Sensors {
public:

  static Perf imu_health;

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

  int update();

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

  // Note: These scales settings requires to meet the values in ICM_20948::initializeDMP(void)
  // Overwrite ICM_20948::initializeDMP(void) to change the scales.
  static const ICM_20948_ACCEL_CONFIG_FS_SEL_e acc_fs_sel{gpm4};
  static const ICM_20948_GYRO_CONFIG_1_FS_SEL_e gyro_fs_sel{dps500};
  
  static constexpr float G_TO_MS2{9.80665};     // G to m/s^2

  // Helper definitions for ICM_20948
  static constexpr uint16_t _acc_scales[4]{2, 4, 8, 16};            // ±2, ±4, ±8, ±16 (G)
  static constexpr uint16_t _gyro_scales[4]{250, 250, 1000, 2000};  // ±250, ±500, ±1000, ±2000 (dps)

  static constexpr uint16_t acc_odr{225};  // Hz
  static constexpr uint16_t gyro_odr{225};  // Hz

  // ACC  ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  // GYRO ODR is computed as follows: 1.1   kHz/(1+GYRO_SMPLRT_DIV[7:0]).   19 = 55Hz.    InvenSense Nucleo example uses 19 (0x13).
  static constexpr uint8_t _smplr_a{const_ceil(1125.0f / acc_odr) - 1};
  static constexpr uint8_t _smplr_g{const_ceil(1100.0f / gyro_odr) - 1};

  static constexpr float _acc_scale{_acc_scales[acc_fs_sel]};      // ±2, ±4, ±8, ±16 (G)
  static constexpr float _gyro_scale{_gyro_scales[gyro_fs_sel]};   // ±250, ±500, ±1000, ±2000 (dps)

  // See datasheet for these scale factors
  static constexpr float _gyro_scale_factor{static_cast<float>(2 << 14) / _gyro_scale};  // LSB/dps
  static constexpr float _acc_scale_factor{static_cast<float>(2 << 14) / _acc_scale};    // LSB/G

  int imu_init();
  static void imu_int_handler();
};

#endif
