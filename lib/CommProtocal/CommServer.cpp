#include "CommProtocol.h"

CommServer::CommServer(uint16_t port)
    : webSocket(port), CommProtocol(), ben(100){};

void CommServer::init() {
  webSocket.begin();
  webSocket.onEvent(
      [&](uint8_t client_id, WStype_t type, uint8_t *payload, size_t length) {
        // TODO: determine the data type
        // Packet *pong_packet;
        // CommProtocol::allocate_packet_by_length(pong_packet, length);

        switch (type) {
        case WStype_DISCONNECTED:
#ifdef COMM_DEBUG_PRINT
          log_d("[%u] Disconnected!\n", client_id);
#endif
          break;

        case WStype_CONNECTED: {
          IPAddress ip = webSocket.remoteIP(client_id);
#ifdef COMM_DEBUG_PRINT
          log_d("[%u] Connected from %d.%d.%d.%d url: %s\n", client_id, ip[0],
                ip[1], ip[2], ip[3], payload);

      // send message to client
      //   webSocket.sendTXT(client_id, "Connected");
#endif
        } break;

        case WStype_TEXT:
#ifdef COMM_DEBUG_PRINT
          log_d("[%u] get Text: %s at %ld\n", client_id, payload, micros());
#endif
          break;

        case WStype_BIN: {
          switch (length * sizeof(uint8_t)) {
          case sizeof(MsgPacket):
            break;

          case sizeof(CtrlPacket): {
            CtrlPacket pong_packet;
            memcpy((void *)&pong_packet, payload, length * sizeof(uint8_t));
            ben.feed_data(pong_packet.id, pong_packet.time, micros());
            break;
          }

          case sizeof(StatePacket): {
            StatePacket pong_packet;
            memcpy((void *)&pong_packet, payload, length * sizeof(uint8_t));
            Serial.printf("[Acc] %f, %f, %f\n", pong_packet.acc.x,
                          pong_packet.acc.y, pong_packet.acc.z);
            break;
          }

          case sizeof(Packet):
          default:
            break;
          }
        }
#ifdef COMM_DEBUG_PRINT
          log_d("Receiving pong: id=%d, time=%ld, at %ld\n", pong_packet.id,
                pong_packet.time, micros());
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
}

bool CommServer::send(PACKET_TYPE type, const Packet *const packet) {
  uint32_t packet_len = CommProtocol::get_packet_len(type);
#ifdef COMM_DEBUG_PRINT
  log_d("Publishing ping: id=%d, time=%ld, at %ld\n", packet->id, packet->time,
        micros());
#endif
  return broadcastBIN((uint8_t *)packet, packet_len);
}

void CommServer::update() { webSocket.loop(); }