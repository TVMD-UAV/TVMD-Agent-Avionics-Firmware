// #define WEBSOCKET_PINGPONG
#ifdef WEBSOCKET_PINGPONG
#include "benchmark.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// #define SERVER

#ifdef SERVER
#include <WebSocketsServer.h>
WebSocketsServer webSocket = WebSocketsServer(81);
#else
#include <WebSocketsClient.h>
WebSocketsClient webSocket;
#endif

#define USE_SERIAL Serial
Benchmark ben(100);

// target
// IPAddress server(192, 168, 0, 10);
// const uint16_t serverPort = 11411;
const char *ssid = "esp_websocket";
const char *password = "esp_ros_pingpong";

String topic_ping = "esp_comm/ping";
String topic_pong = "esp_comm/pong";

typedef struct PACKET {
  uint32_t time;
  int32_t id;
  char data[400];
} Packet;

Packet packet;

#ifdef SERVER
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {

  switch (type) {
  case WStype_DISCONNECTED:
    USE_SERIAL.printf("[%u] Disconnected!\n", num);
    break;

  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(num);
    USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                      ip[1], ip[2], ip[3], payload);

    // send message to client
    webSocket.sendTXT(num, "Connected");
  } break;

  case WStype_TEXT:
    USE_SERIAL.printf("[%u] get Text: %s at %ld\n", num, payload, micros());
    break;

  case WStype_BIN:
    Packet pong_packet;
    memcpy((void *)&pong_packet, payload, length * sizeof(uint8_t));
    ben.feed_data(pong_packet.id, pong_packet.time, micros());

    break;
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
    break;
  }
}
#else
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {

  switch (type) {
  case WStype_DISCONNECTED:
    USE_SERIAL.printf("[WSc] Disconnected!\n");
    break;

  case WStype_CONNECTED:
    USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

    // send message to server when Connected
    webSocket.sendTXT("Connected");
    break;

  case WStype_TEXT:
    USE_SERIAL.printf("[WSc] get text: %s\n", payload);
    webSocket.sendTXT(payload);

    break;
  case WStype_BIN:
    webSocket.sendBIN(payload, length * sizeof(uint8_t));

    break;
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
    break;
  }
}
#endif

#ifdef SERVER
void startWiFiAP() {
  WiFi.softAP(ssid, password);
  Serial.println("AP started");
  Serial.println("IP address: " + WiFi.softAPIP().toString());
}
#else
void startWiFiClient() {
  Serial.println("Connecting to " + (String)ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());
}

#endif

void setup() {
  // USE_SERIAL.begin(921600);
  USE_SERIAL.begin(115200);
  USE_SERIAL.println();
  USE_SERIAL.println();

#ifdef SERVER
  // Or start the ESP as AP
  startWiFiAP();
  Serial.println("Starting Web Socket Server");
#else
  // Connect to a WiFi network
  startWiFiClient();
#endif

#ifdef SERVER

  // Start the server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
#else
  webSocket.begin("192.168.4.1", 81, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
#endif
}

unsigned long last_publish_time = 0;
unsigned long last_summary_time = 0;
int greeting_number = 1;

void loop() {
  // do anything here
  webSocket.loop();

  uint8_t id = 0;

#ifdef SERVER
  if (millis() - last_publish_time >= 10) {
    packet.id += 1;
    packet.time = micros();
    if (webSocket.clientIsConnected(id)) {
      webSocket.sendBIN(id, (uint8_t *)&packet, sizeof(packet));
      // USE_SERIAL.printf("Publishing ping: id=%d, time=%ld, at %ld\n",
      // packet.id, packet.time, micros());
    }
    last_publish_time = millis();
  }

  if (millis() - last_summary_time >= 5000) {
    Serial.printf("fps: %f, latency: %f, packet_lost: %d\n", ben.get_fps(),
                  ben.get_latency(), ben.packet_lost());
    last_summary_time = millis();
  }
#endif
}
#endif