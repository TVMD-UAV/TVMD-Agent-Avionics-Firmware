#include <ImuEchoHandler.h>

void ImuEchoHandler::init() 
{
    IMU_ECHO_I2C_BUS.onReceive(onReceive);
    IMU_ECHO_I2C_BUS.onRequest(onRequest);

    IMU_ECHO_I2C_BUS.setBufferSize(I2C_BUFFER_SIZE);

    // uint8_t slaveAddr, int sda, int scl, uint32_t frequency
    IMU_ECHO_I2C_BUS.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_FREQ);
    log_i("ImuEchoHandler initialized");
}
