// #define MQTT_PING_PONG
#ifdef MQTT_PING_PONG
#include "benchmark.h"
#include <Arduino.h>

#include <PicoMQTT.h>
Benchmark ben(100);

#define SERVER

const char *ssid = "esp_mqtt";
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
PicoMQTT::Server mqtt;
#else
PicoMQTT::Client mqtt("192.168.4.1");
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
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  packet.time = micros();
  packet.id = 0;

#ifdef SERVER
  // Or start the ESP as AP
  startWiFiAP();
  Serial.println("Starting MQTT broker");
#else
  // Connect to a WiFi network
  startWiFiClient();
#endif

#ifdef SERVER
  mqtt.subscribe("esp_comm/pong", [](const char *topic, const char *payload) {
    // payload might be binary, but PicoMQTT guarantees that it's
    // zero-terminated
    Packet pong_packet;
    memcpy((void *)&pong_packet, payload, sizeof(pong_packet));
    // Serial.printf("Received pong in '%s': id=%ld, time=%ld, at %ld\n", topic,
    // pong_packet.id, pong_packet.time, micros());
    ben.feed_data(pong_packet.id, pong_packet.time, micros());
  });
#else
  mqtt.subscribe("esp_comm/ping", [](const char *topic, const char *payload) {
    // payload might be binary, but PicoMQTT guarantees that it's
    // zero-terminated
    memcpy((void *)&packet, payload, sizeof(packet));
    // Serial.printf("Received ping in '%s': id=%ld, time=%ld\n", topic,
    // packet.id, packet.time);

    // Pong
    mqtt.publish(topic_pong, (uint8_t *)&packet, sizeof(packet), 0, false, 0);
  });
#endif

  // Start the broker
  mqtt.begin();
}

unsigned long last_publish_time = 0;
unsigned long last_summary_time = 0;
int greeting_number = 1;

void loop() {
  // do anything here
  mqtt.loop();

#ifdef SERVER
  if (millis() - last_publish_time >= 10) {
    // We're publishing to a topic, which we're subscribed too, but these
    // message will *not* be delivered locally. String topic = "picomqtt/esp-" +
    // WiFi.macAddress();

    packet.id += 1;
    packet.time = micros();
    mqtt.publish(topic_ping, (char *)&packet, sizeof(packet), 0, false, 0);
    // Serial.printf("Publishing ping in '%s': id=%ld, time=%ld, at %ld\n",
    // topic_ping, packet.id, packet.time, micros());
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