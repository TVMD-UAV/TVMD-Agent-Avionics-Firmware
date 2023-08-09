#include "CommProtocol.h"

void CommClient::init() {
  webSocket.begin(_serv_addr, _port, "/");
  webSocket.onEvent([&](WStype_t type, uint8_t *payload, size_t length) {
    Packet ping_packet;
    switch (type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!, WiFi: %s\n",
                        (WiFi.isConnected() ? "Connected" : "Disconnected"));
      break;

    case WStype_CONNECTED:
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

      // send message to server when Connected
      webSocket.sendTXT("Connected");
      break;

    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);
      //   webSocket.sendTXT(payload);

      break;
    case WStype_BIN: 
      CommProtocol::callback_router(payload, length);
      // webSocket.sendBIN(payload, length * sizeof(uint8_t));
      // TODO: The data type should be determined
#ifdef COMM_DEBUG_PRINT
      //   memcpy((void *)&ping_packet, payload, length * sizeof(uint8_t));
      //   USE_SERIAL.printf("Receiving pong: id=%d, time=%ld, at %ld\n",
      //                     ping_packet.id, ping_packet.time, micros());
#endif

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
  CommProtocol::init();
}

bool CommClient::send(PACKET_TYPE type, const Packet *const packet) {

  if (WiFi.status() == WL_CONNECTED & webSocket.isConnected()) {
    uint32_t packet_len = CommProtocol::get_packet_len(type);
    return sendToBIN(0, (uint8_t *)packet, packet_len);
  }
  return false;
}

void CommClient::update() {
  if (WiFi.isConnected()) {
    webSocket.loop();
  } else {
    // Reconnecting WiFi
    // WiFi.reconnect();
    // while (!WiFi.status() == WL_CONNECTED) {
    //   delay(500);
    // }
  }
}