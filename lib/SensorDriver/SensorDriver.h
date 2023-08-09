#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "CommProtocol.h"

#define PRESSURE_TO_ALTITUDE(PRESSURE, SEA_LEVEL_HPA)                          \
  (44330 * (1.0 - pow(((PRESSURE) / 100) / (SEA_LEVEL_HPA), 0.1903)))

class Sensors {
public:
  Adafruit_MPU6050 imu;
  Adafruit_BMP280 baro;

  Sensors();

  void init(int sda = 21, int scl = 22);

  const StatePacket *const state_packet_gen(uint8_t _state);

private:
  StatePacket s_packet;
  bool imu_enabled;
  bool baro_enabled;
};

#endif
