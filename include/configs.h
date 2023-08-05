#ifndef CONFIGS_H
#define CONFIGS_H

// #define COMM_DEBUG_PRINT
#define USE_SERIAL Serial

#define WIFI_SSID "esp_websocket"
#define WIFI_PSWD "esp_ros_pingpong"
#define COMM_PORT 11411

#ifdef SERVER
#else
// Acting as an agent
#define HAS_MOTOR
#endif

#endif