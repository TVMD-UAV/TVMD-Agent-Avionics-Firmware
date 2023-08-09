#include "Core.h"

/*
 * TODO:
 * ctrl, state function callback
 * task scheduler
 * connection timeout solutions
 */

unsigned long last_publish_time = 0;
unsigned long last_servo_time = 0;
unsigned long last_addr_time = 0;

void setup() {
  Serial.begin(115200);

#ifdef SERVER
  packet.time = 0;
  packet.id = 0;
#else
  Core::set_armed(true);
#endif
}

void loop() {

#ifdef SERVER
  if (millis() - last_addr_time >= 500) {
    display_connected_devices();
    last_addr_time = millis();
  }

  if (millis() - last_publish_time >= 20) {
    packet.id += 1;
    packet.time = micros();

    comm.send(CommProtocol::PACKET_TYPE::CTRL_CMDS, &packet);

    last_publish_time = millis();
    // display_connected_devices();
  }

  if (millis() - last_summary_time >= 5000) {
    Serial.printf("fps: %f, latency: %f, packet_lost: %d\n", comm.ben.get_fps(),
                  comm.ben.get_latency(), comm.ben.packet_lost());
    last_summary_time = millis();
  }
#else
#define SERVO_TEST
#ifdef SERVO_TEST
  if (millis() - last_servo_time >= 10) {
    float t = millis() / 1000.0;
    Core::x_servo.write(60.0f * sin(t));
    Core::y_servo.write(60.0f * cos(t));
    last_servo_time = millis();
  }
#endif
#endif
};

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
  // mqtt.subscribe("esp_comm/pong", [](const char * topic, const char *
  // payload) {
  //     // payload might be binary, but PicoMQTT guarantees that it's
  //     zero-terminated Packet pong_packet; memcpy((void*)&pong_packet,
  //     payload, sizeof(pong_packet)); Serial.printf("Received pong in '%s':
  //     id=%ld, time=%ld, at %ld\n", topic, pong_packet.id, pong_packet.time,
  //     micros());
  // });

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
  //   mqtt.subscribe("esp_comm/ping", [](const char * topic, const char *
  //   payload) {
  //     // payload might be binary, but PicoMQTT guarantees that it's
  //     zero-terminated memcpy((void*)&packet, payload, sizeof(packet));
  //     Serial.printf("Received ping in '%s': id=%ld, time=%ld\n", topic,
  //     packet.id, packet.time);

  //     // Pong
  //     mqtt.publish(topic_pong, (char*)&packet, sizeof(packet), 0, false, 0);
  //   });
  // if(udp.connect(IPAddress(0,0,0,0), udp_port)) {
  // if(udp.connect(IPAddress(192,168,4,1), udp_port)) {
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
    // We're publishing to a topic, which we're subscribed too, but these
    // message will *not* be delivered locally. String topic = "picomqtt/esp-" +
    // WiFi.macAddress();

    t_packet.id += 1;
    t_packet.time = micros();
    // mqtt.publish(topic_ping, (char*)&packet, sizeof(packet), 0, false, 0);
    udp.broadcastTo((uint8_t *)&t_packet, sizeof(t_packet), udp_port);
    // Serial.printf("Publishing ping from %s: id=%ld, time=%ld, at %ld\n",
    // WiFi.localIP().toString(), t_packet.id, t_packet.time, micros());
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