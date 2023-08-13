#include "SensorDriver.h"

Sensors::Sensors() : imu_enabled(false), baro_enabled(false) {}

void Sensors::init(int sda, int scl) {

  // IMU Setup
  Wire.setPins(sda, scl);
  if (imu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    imu_enabled = true;
    log_i("MPU6050 initialized!");
  } else
    log_e("Failed to find MPU6050 chip");

  if (baro.begin(BMP280_ADDRESS_ALT)) {
    baro.setSampling(Adafruit_BMP280::MODE_FORCED,  /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,   /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    baro_enabled = true;
    log_i("BMP280 initialized!");
  } else
    log_e("Could not find a valid BMP280 sensor, check wiring or "
          "try a different address!, SensorID was: 0x%x",
          baro.sensorID());
}

bool Sensors::state_packet_gen(StatePacket *const _packet) {
  if (imu_enabled) {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    _packet->acc = a.acceleration;
    _packet->gyro = g.gyro;
    _packet->temperature = temp.temperature;
  }

  if (baro_enabled) {
    _packet->pressure = baro.readPressure();
    const float seaLevelhPa = 1013.25;
    _packet->altitude = PRESSURE_TO_ALTITUDE(_packet->pressure, seaLevelhPa);
    //   44330 * (1.0 - pow((_packet->pressure / 100) / seaLevelhPa, 0.1903));
  }

  return true;
}
