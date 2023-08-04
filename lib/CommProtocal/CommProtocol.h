#ifndef COMM_PROTOCOL

#include "benchmark.h"
#include "configs.h"

#define COMM_PROTOCOL
#include <WiFiClientSecure.h>

#include <WebSocketsClient.h>
#include <WebSocketsServer.h>

struct Packet {
  uint32_t time;
  int32_t id;
  virtual ~Packet() = default; // To make Packet polymorphic, and thus a
                               // dynamic_cast can be used.
};

struct CtrlPacket : public Packet {
  double eta_x;
  double eta_y;
  double omega_p1;
  double omega_p2;
};

struct MsgPacket : public Packet {
  uint8_t msg[400];
};

class CommProtocol {
public:
  const char *topic_ctrl_commands = "ctrl/commands";
  const char *topic_ctrl_instruct = "ctrl/instruct";
  const char *topic_state_agent = "state/agent";

  enum PACKET_TYPE { CTRL_CMDS = 0, CTRL_INST, STATE_AGN, MSG, RAW };

  //   CommProtocol();

  void virtual init() = 0;

  virtual bool send(PACKET_TYPE type, Packet *packet) = 0;

  virtual void update();

protected:
  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) = 0;

  int get_packet_len(PACKET_TYPE type);

  void allocate_packet_by_length(Packet *packet, size_t length);
};

void CommProtocol::allocate_packet_by_length(Packet *packet, size_t length) {
  switch (length * sizeof(uint8_t)) {
  case sizeof(MsgPacket):
    packet = new MsgPacket();
    break;

  case sizeof(CtrlPacket):
    packet = new CtrlPacket();
    break;

  case sizeof(Packet):
  default:
    packet = new Packet();
    break;
  }
}

int CommProtocol::get_packet_len(const PACKET_TYPE type) {
  uint32_t packet_len = 0;
  switch (type) {
  case PACKET_TYPE::CTRL_CMDS:
    packet_len = sizeof(CtrlPacket);
    // CtrlPacket *t_packet = dynamic_cast<CtrlPacket *>(packet);
    break;

  case PACKET_TYPE::CTRL_INST:
    packet_len = sizeof(CtrlPacket);
    break;

  case PACKET_TYPE::MSG:
    packet_len = sizeof(MsgPacket);
    break;

  default:
    packet_len = sizeof(Packet);
  }
  return packet_len;
}

class CommServer : CommProtocol {
public:
  CommServer(uint16_t port);

  // Setup event listener
  void init();

  virtual bool send(PACKET_TYPE type, Packet *packet);

  virtual void update();

  Benchmark ben;

protected:
  bool broadcastBIN(uint8_t *payload, size_t length = 0) {
    return webSocket.broadcastBIN(payload, length);
  };

  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) {
    return webSocket.sendBIN(client_id, payload, length);
  };

private:
  WebSocketsServer webSocket;
};

CommServer::CommServer(uint16_t port)
    : webSocket(port), CommProtocol(), ben(100){};

void CommServer::init() {
  webSocket.begin();
  webSocket.onEvent(
      [&](uint8_t client_id, WStype_t type, uint8_t *payload, size_t length) {
        // TODO: determine the data type
        Packet *pong_packet;
        // CommProtocol::allocate_packet_by_length(pong_packet, length);
        switch (length * sizeof(uint8_t)) {
        case sizeof(MsgPacket):
          pong_packet = new MsgPacket();
          break;

        case sizeof(CtrlPacket):
          pong_packet = new CtrlPacket();
          break;

        case sizeof(Packet):
        default:
          pong_packet = new Packet();
          break;
        }

        switch (type) {
        case WStype_DISCONNECTED:
#ifdef COMM_DEBUG_PRINT
          USE_SERIAL.printf("[%u] Disconnected!\n", client_id);
#endif
          break;

        case WStype_CONNECTED: {
          IPAddress ip = webSocket.remoteIP(client_id);
#ifdef COMM_DEBUG_PRINT
          USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n",
                            client_id, ip[0], ip[1], ip[2], ip[3], payload);

      // send message to client
      //   webSocket.sendTXT(client_id, "Connected");
#endif
        } break;

        case WStype_TEXT:
#ifdef COMM_DEBUG_PRINT
          USE_SERIAL.printf("[%u] get Text: %s at %ld\n", client_id, payload,
                            micros());
#endif
          break;

        case WStype_BIN:
          memcpy((void *)pong_packet, payload, length * sizeof(uint8_t));
          ben.feed_data(pong_packet->id, pong_packet->time, micros());
#ifdef COMM_DEBUG_PRINT
          USE_SERIAL.printf("Receiving pong: id=%d, time=%ld, at %ld\n",
                            pong_packet.id, pong_packet.time, micros());
#endif

          break;
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
          break;
        }
        free(pong_packet);
      });
}

bool CommServer::send(PACKET_TYPE type, Packet *packet) {
  uint32_t packet_len = CommProtocol::get_packet_len(type);
#ifdef COMM_DEBUG_PRINT
  USE_SERIAL.printf("Publishing ping: id=%d, time=%ld, at %ld\n", packet->id,
                    packet->time, micros());
#endif
  return broadcastBIN((uint8_t *)packet, packet_len);
}

void CommServer::update() { webSocket.loop(); }

class CommClient : CommProtocol {
public:
  CommClient(uint16_t port, IPAddress serv_ip)
      : webSocket(), CommProtocol(), _port(port), _serv_addr(serv_ip){};

  // Setup event listener
  void init();

  bool send(PACKET_TYPE type, Packet *packet);

  virtual void update();

protected:
  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) {
    return webSocket.sendBIN(payload, length);
  };

private:
  uint16_t _port;
  IPAddress _serv_addr;
  WebSocketsClient webSocket;
};

void CommClient::init() {
  webSocket.begin(_serv_addr, _port, "/");
  webSocket.onEvent([&](WStype_t type, uint8_t *payload, size_t length) {
    Packet ping_packet;
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
      //   webSocket.sendTXT(payload);

      break;
    case WStype_BIN:
      webSocket.sendBIN(payload, length * sizeof(uint8_t));
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
}

bool CommClient::send(PACKET_TYPE type, Packet *packet) {
  uint32_t packet_len = CommProtocol::get_packet_len(type);
  return sendToBIN(0, (uint8_t *)packet, packet_len);
}

void CommClient::update() { webSocket.loop(); }
#endif
