#ifndef CONFIGS_H
#define CONFIGS_H

#include <IPAddress.h>

// #define WRITE_AGENT_ID
// #define AGENT_ID_TO_WRITE 1
#define MAX_NUM_AGENTS 4

// 0: server
// 1: normal agent
// 2: Wire occupied agent
// #define AGENT_ID_TO_WRITE 1

// #define COMM_DEBUG_PRINT
#define USE_SERIAL Serial

/* WiFi Settings */
#define WIFI_SSID "esp_websocket"
#define WIFI_PSWD "esp_ros_pingpong"
#define COMM_PORT 11411

const IPAddress server_IP(192, 168, 4, 1);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);

/* Websocket Connection Settings */
#define MAX_TIME_OUT 1000
#define NUM_HEALTH_CHECK 10

/* Agent Settings */
#ifdef WRITE_AGENT_ID
#ifdef SERVER
#define AGENT_ID 0
#else
#define AGENT_ID AGENT_ID_TO_WRITE
#endif
#endif

/* EEPROM Settings */
#define EEPROM_SIZE 1
#define EEPROM_AGENT_ID_ADDR 0

/* Motor Setup */
#ifdef SERVER
#else
// Acting as an agent
#define HAS_MOTOR
#define MOTOR_X_SERVO_PIN 39
#define MOTOR_Y_SERVO_PIN 40
#define MOTOR_ESC_P1_PIN 41
#define MOTOR_ESC_P2_PIN 42
#endif

/* IMU Settings */

// Default using HSPI
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define IMU_SCK_PIN 36
#define IMU_MISO_PIN 37
#define IMU_MOSI_PIN 35
#define IMU_CS_PIN 38     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define IMU_INT_PIN 13    // Which pin you connect INT to

/* BMP Settings */
#define BMP_CS_PIN 14

/* Indicator Settings */
#define LED_COUNT 2
#define LED_PIN 21


/* Debugging */
#define SERIAL_PORT Serial

/* IMU Debugging */
#define IMU_DEBUG_PRINT

/* Instruction Handler */
#define EXT_I2C_SDA_PIN 7
#define EXT_I2C_SCL_PIN 6

#define INSTR_I2C_CLOCK_FREQ 400000 // Hz, using fast mode
#define INSTR_I2C_SLAVE_ADDR 0x66

#ifdef SERVER
#define INSTR_I2C_BUFFER_SIZE 1024
#else
#define INSTR_I2C_BUFFER_SIZE 256
#endif 

/* IMU Echo Handler */
// #define ENABLE_SERVER_IMU_ECHO
#define I2C_SDA_PIN 9
#define I2C_SCL_PIN 8

#define I2C_CLOCK_FREQ 400000 // Hz, using fast mode
#define I2C_SLAVE_ADDR 0x66
#define I2C_BUFFER_SIZE 256


#endif