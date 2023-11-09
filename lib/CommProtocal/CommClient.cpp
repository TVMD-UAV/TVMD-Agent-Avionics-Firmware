#include "CommProtocol.h"

Benchmark CommClient::ctrl_health;
Benchmark CommClient::state_health;

void CommClient::init(uint8_t agent_id) {
  webSocket.begin(_serv_addr, _port, "/");
  webSocket.onEvent([&](WStype_t type, uint8_t *payload, size_t length) {
    Packet ping_packet;
    switch (type) {
    case WStype_DISCONNECTED:
#ifdef COMM_DEBUG_PRINT
#endif
      USE_SERIAL.printf("[WSc] Disconnected!, WiFi: %s\n",
                        (WiFi.isConnected() ? "Connected" : "Disconnected"));
      set_connection_lost(0);
      break;

    case WStype_CONNECTED:
#ifdef COMM_DEBUG_PRINT
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
#endif
      // send message to server when Connected
      webSocket.sendTXT("Connected");
      break;

    case WStype_TEXT:
#ifdef COMM_DEBUG_PRINT
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);
#endif
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
  webSocket.setReconnectInterval(5000);

  ctrl_health = Benchmark(NUM_HEALTH_CHECK);
  state_health = Benchmark(NUM_HEALTH_CHECK);

  CommProtocol::init(agent_id);
}

bool CommClient::send(const Packet *const packet, const size_t len) {
  if (WiFi.status() == WL_CONNECTED & webSocket.isConnected()) {
    return sendToBIN(0, (uint8_t *)packet, len);
  }
  return false;
}

void CommClient::update() {
  if (WiFi.isConnected()) {
    // websocket routine
    webSocket.loop();

    // websocket connection timeout
    if (millis() - last_update_time > MAX_TIME_OUT)
      set_connection_lost(0);
  } else {
    // WiFi lost
    set_connection_lost(0);
  }
}