#include "CommProtocol.h"

Perf CommClient::ctrl_health;
Perf CommClient::state_health;

void CommClient::init(uint8_t agent_id) {
  webSocket.begin(_serv_addr, _port, "/");
  webSocket.onEvent([&](WStype_t type, uint8_t *payload, size_t length) {
    Packet ping_packet;
    switch (type) {
    case WStype_DISCONNECTED:
      // log_w("[WSc] Disconnected!, WiFi: %s\n",
      //                   (WiFi.isConnected() ? "Connected" : "Disconnected"));
      break;

    case WStype_CONNECTED:
      // log_v("[WSc] Connected to url: %s\n", payload);
      // send message to server when Connected
      // webSocket.sendTXT("Connected");
      break;

    case WStype_TEXT:
      // log_v("[WSc] get text: %s\n", payload);
      break;

    case WStype_BIN:
      last_update_time = millis();
      CommProtocol::callback_router(payload, length);
      break;

    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
    }
  });

  webSocket.setReconnectInterval(200);

  ctrl_health = Perf(NUM_HEALTH_CHECK);
  state_health = Perf(NUM_HEALTH_CHECK);

  CommProtocol::init(agent_id);
}

bool CommClient::send(const Packet *const packet, const size_t len) {
  if ((WiFi.status() == WL_CONNECTED) && webSocket.isConnected()) {
    return sendToBIN(0, (uint8_t *)packet, len);
  }
  return false;
}

void CommClient::update() {
  if (WiFi.isConnected()) {
    // websocket routine
    webSocket.loop();
  }
}