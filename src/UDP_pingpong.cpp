#ifdef UDP_PING_PONG
#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>

#include "benchmark.h"
#define SERVER

const char *ssid = "esp_udp";
const char *password = "esp_ros_pingpong";
const uint16_t udp_port = 11411;
AsyncUDP udp;
Benchmark ben(100);

typedef struct PACKET {
  uint32_t time;
  int32_t id;
  char data[400];
} Packet;
Packet t_packet;
Packet ping_packet;
Packet pong_packet;

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

  t_packet.time = micros();
  t_packet.id = 0;

#ifdef SERVER
  // Or start the ESP as AP
  startWiFiAP();
  Serial.println("Starting UDP server");
#else
  // Connect to a WiFi network
  startWiFiClient();
#endif

#ifdef SERVER

  if (udp.listen(udp_port)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {
      // Serial.print("UDP Packet Type: ");
      // Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
      // Serial.print(", From: ");
      // Serial.print(packet.remoteIP());
      // Serial.print(":");
      // Serial.print(packet.remotePort());
      // Serial.print(", To: ");
      // Serial.print(packet.localIP());
      // Serial.print(":");
      // Serial.print(packet.localPort());
      // Serial.print(", Length: ");
      // Serial.print(packet.length());
      // Serial.print(", Data: ");
      // Serial.write(packet.data(), packet.length());
      // Serial.println();

      memcpy((void *)&pong_packet, packet.data(), packet.length());
      // Serial.printf("Received pong from '%s': id=%ld, time=%ld, at %ld\n",
      //     packet.remoteIP().toString(), pong_packet.id, pong_packet.time,
      //     micros());
      ben.feed_data(pong_packet.id, pong_packet.time, micros());
      // reply to the client
      // packet.printf("Got %u bytes of data", packet.length());
    });
  }

#else
  if (udp.listen(udp_port)) {
    Serial.println("UDP connected");
    udp.onPacket([](AsyncUDPPacket packet) {
      // Serial.print("UDP Packet Type: ");
      // Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
      // Serial.print(", From: ");
      // Serial.print(packet.remoteIP());
      // Serial.print(":");
      // Serial.print(packet.remotePort());
      // Serial.print(", To: ");
      // Serial.print(packet.localIP());
      // Serial.print(":");
      // Serial.print(packet.localPort());
      // Serial.print(", Length: ");
      // Serial.print(packet.length());
      // Serial.print(", Data: ");
      // Serial.write(packet.data(), packet.length());
      // Serial.println();
      // //reply to the client
      // packet.printf("Got %u bytes of data", packet.length());
      memcpy((void *)&ping_packet, packet.data(), packet.length());
      Serial.printf("Received ping from '%s': id=%ld, time=%ld\n",
                    packet.remoteIP().toString(), ping_packet.id,
                    ping_packet.time);
      udp.broadcastTo((uint8_t *)&ping_packet, sizeof(ping_packet), udp_port);
    });
    // Send unicast
    t_packet.id = 1234;
    t_packet.time = micros();
    udp.broadcastTo((uint8_t *)&t_packet, sizeof(t_packet), udp_port);
  }
#endif
}

unsigned long last_publish_time = 0;
unsigned long last_summary_time = 0;
int greeting_number = 1;

void loop() {
// Send broadcast
#ifdef SERVER
  if (millis() - last_publish_time >= 20) {
    t_packet.id += 1;
    t_packet.time = micros();
    udp.broadcastTo((uint8_t *)&t_packet, sizeof(t_packet), udp_port);
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