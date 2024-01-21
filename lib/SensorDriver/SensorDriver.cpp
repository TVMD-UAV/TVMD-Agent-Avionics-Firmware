#include "SensorDriver.h"

volatile bool Sensors::isrFired = false;
volatile bool Sensors::sensorSleep = false;
volatile bool Sensors::canToggle = false;

Sensors::Sensors() : imu_enabled(false), baro_enabled(false), 
#ifdef USE_BMP280_SPI
  baro(BMP_CS_PIN, &SPI_PORT)
#endif
{}

int Sensors::init(int update_rate) {
  _data_mutex = xSemaphoreCreateMutex();
  _sensor_updated = false;
  _update_interval = 1000 / update_rate;

  SPI_PORT.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);

  if (imu_init() != SENSOR_OK) {
    return IMU_INIT_ERROR;
  }

  #ifdef USE_BMP280_SPI
  if (baro.begin()) {
  #elif defined(USE_BMP280_I2C)
  if (baro.begin(BMP280_ADDRESS_ALT)) {
  #endif
    baro.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

    baro_enabled = true;
    log_i("BMP280 initialized!");
  } else {
    log_e("Could not find a valid BMP280 sensor, check wiring or "
          "try a different address!, SensorID was: 0x%x",
          baro.sensorID());
    return BARO_INIT_ERROR;
  }
  
  return SENSOR_OK;
}

void Sensors::state_packet_gen(StatePacket *const _packet) {
  if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&(_packet->s), &_data, sizeof(_data));
    xSemaphoreGive(_data_mutex);
  }
  _sensor_updated = false;
}

int Sensors::imu_init(){
  // IMU Setup
#ifdef USE_MPU6050
  // TODO: sda and scl are not defined here.
  Wire.setPins(sda, scl);
  if (imu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    imu_enabled = true;
    log_d("MPU6050 initialized!");
  } else
    log_e("Failed to find MPU6050 chip");
#elif defined(USE_ICM20948_SPI)
  #ifdef IMU_DEBUG_PRINT
    imu.enableDebugging();
  #endif

  bool initialized = false;
  for (int i=0; !initialized && i < 5; ++i) {
    imu.begin(IMU_CS_PIN, SPI_PORT);
    if (imu.status != ICM_20948_Stat_Ok) {
      log_e("Failed to initialize IMU, sensor status: %s", imu.statusString());
      SPI_PORT.end();
      delay(500);
    }
    else
      initialized = true;
  }
  if (!initialized) return SENSOR_ERROR::IMU_INIT_ERROR;

  bool success = true; // Use success to show if the DMP configuration was successful
  // Initializing ICM DMP
  #ifdef USE_IMU_DMP
    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Enable any additional sensors / features
    // success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    // success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (imu.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (imu.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (imu.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

    // Now we're going to set up interrupts. There are a lot of options, but for this test we're just configuring the interrupt pin and enabling interrupts to tell us when new data is ready
    /*
      ICM_20948_Status_e  cfgIntActiveLow         ( bool active_low );
      ICM_20948_Status_e  cfgIntOpenDrain         ( bool open_drain );
      ICM_20948_Status_e  cfgIntLatch             ( bool latching );                          // If not latching then the interrupt is a 50 us pulse

      ICM_20948_Status_e  cfgIntAnyReadToClear    ( bool enabled );                           // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first

      ICM_20948_Status_e  cfgFsyncActiveLow       ( bool active_low );
      ICM_20948_Status_e  cfgFsyncIntMode         ( bool interrupt_mode );                    // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

      ICM_20948_Status_e  intEnableI2C            ( bool enable );
      ICM_20948_Status_e  intEnableDMP            ( bool enable );
      ICM_20948_Status_e  intEnablePLL            ( bool enable );
      ICM_20948_Status_e  intEnableWOM            ( bool enable );
      ICM_20948_Status_e  intEnableWOF            ( bool enable );
      ICM_20948_Status_e  intEnableRawDataReady   ( bool enable );
      ICM_20948_Status_e  intEnableOverflowFIFO   ( uint8_t bm_enable );
      ICM_20948_Status_e  intEnableWatermarkFIFO  ( uint8_t bm_enable );
  */
    imu.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
    imu.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
    imu.cfgIntLatch(true);      // Latch the interrupt until cleared
    log_i("cfgIntLatch returned: %s", imu.statusString());

    // Check success
    if (success)
      log_i("DMP enabled!");
    else {
      log_i("Enable DMP failed! Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
      return SENSOR_ERROR::DMP_INIT_ERROR;
    }
#endif // USE_IMU_DMP

  // Setup interrupts
#ifdef USE_IMU_INT
    pinMode(IMU_INT_PIN, INPUT_PULLUP);                                   // Using a pullup b/c ICM-20948 Breakout board has an onboard pullup as well and we don't want them to compete
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imu_int_handler, FALLING); // Set up a falling interrupt

    // Enable interrupts on dmp ready
    imu.intEnableDMP(true);
    log_d("intEnableDMP returned: %s", imu.statusString());

    //  // Note: weirdness with the Wake on Motion interrupt being always enabled.....
    //  uint8_t zero_0 = 0xFF;
    //  ICM_20948_execute_r( &imu._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
    //  SERIAL_PORT.print("INT_EN was: 0x"); SERIAL_PORT.println(zero_0, HEX);
    //  zero_0 = 0x00;
    //  ICM_20948_execute_w( &imu._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
#endif // USE_IMU_INT
  
  imu_enabled = true;

  return SENSOR_ERROR::SENSOR_OK;
#endif
}

void Sensors::imu_int_handler(){
  isrFired = true;
}

/**
 * @brief Update the sensor data
 * 
 */
void Sensors::update() {
  static time_t last_update_time = millis();

  // TODO: load data into packet
  if (imu_enabled) {
    #ifdef USE_MPU6050
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    _data.acc.x = a.acceleration.x;
    _data.acc.y = a.acceleration.y;
    _data.acc.z = a.acceleration.z;
    _data.gyro.x = g.gyro.x;
    _data.gyro.y = g.gyro.y;
    _data.gyro.z = g.gyro.z;
    _data.temperature = temp.temperature;
    #elif defined(USE_ICM20948_SPI)
    if (isrFired) {
      isrFired = false;
      // Read any DMP data waiting in the FIFO
      // Note:
      //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
      //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
      //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
      //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
      //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
      icm_20948_DMP_data_t data;
      imu.readDMPdataFromFIFO(&data);

      if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
      {
        //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
        //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
        //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
        //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
        //SERIAL_PORT.println( data.header, HEX );

        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
          // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
          // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
          // The quaternion data is scaled by 2^30.

          //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

          // Scale to +/- 1
          const double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
          const double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
          const double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
          const double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

          if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
            _data.orientation.q0 = q0;
            _data.orientation.q1 = q1;
            _data.orientation.q2 = q2;
            _data.orientation.q3 = q3;
            xSemaphoreGive(_data_mutex);
          }

          // SERIAL_PORT.println(data.Quat9.Data.Accuracy);
        }

        if ((data.header & DMP_header_bitmap_Accel) > 0) {
          // We have asked for orientation data so we should receive Quat9
          if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
            _data.acc.x = (float)data.Raw_Accel.Data.X;
            _data.acc.y = (float)data.Raw_Accel.Data.Y;
            _data.acc.z = (float)data.Raw_Accel.Data.Z;
            xSemaphoreGive(_data_mutex);
          }
        }

        if ((data.header & DMP_header_bitmap_Gyro) > 0) {
          // We have asked for orientation data so we should receive Quat9
          if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
            _data.gyro.x = (float)data.Raw_Gyro.Data.X;
            _data.gyro.y = (float)data.Raw_Gyro.Data.Y;
            _data.gyro.z = (float)data.Raw_Gyro.Data.Z;
            xSemaphoreGive(_data_mutex);
          }
        }

        if ((data.header & DMP_header_bitmap_Compass) > 0) {
          // We have asked for orientation data so we should receive Quat9 
          if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
            _data.compass.x = (float)data.Compass.Data.X;
            _data.compass.y = (float)data.Compass.Data.Y;
            _data.compass.z = (float)data.Compass.Data.Z;
            xSemaphoreGive(_data_mutex);
          }
        }
      }

      imu.clearInterrupts();  // This would be efficient... but not compatible with Uno
    }
    #endif
  }

  if (baro_enabled & (millis() - last_update_time > _update_interval)) {
    const float seaLevelhPa = 1013.25;
    if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
      _data.pressure = baro.readPressure();
      _data.altitude = PRESSURE_TO_ALTITUDE(_data.pressure, seaLevelhPa);
      xSemaphoreGive(_data_mutex);
    }
    last_update_time = millis();
    _sensor_updated = true;
  }
}
