#ifndef CONFIGS_H
#define CONFIGS_H


// #define SERVER
// #define WRITE_AGENT_ID

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
#endif

#endif