#ifndef COMM_PROTOCOL

#include "benchmark.h"
#include "configs.h"

#define COMM_PROTOCOL
#include <WiFiClientSecure.h>

#include "Adafruit_Sensor.h"
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

struct StatePacket : public Packet {
  sensors_vec_t acc;
  sensors_vec_t gyro;
  float temperature;
  float pressure;
  float altitude;
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

  virtual bool send(PACKET_TYPE type, const Packet *const packet) = 0;

  virtual void update();

protected:
  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) = 0;

  int get_packet_len(PACKET_TYPE type);

  void allocate_packet_by_length(Packet *packet, size_t length);
};

class CommServer : CommProtocol {
public:
  CommServer(uint16_t port);

  // Setup event listener
  void init();

  virtual bool send(PACKET_TYPE type, const Packet *const packet);

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

class CommClient : CommProtocol {
public:
  CommClient(uint16_t port, IPAddress serv_ip)
      : webSocket(), CommProtocol(), _port(port), _serv_addr(serv_ip){};

  // Setup event listener
  void init();

  bool send(PACKET_TYPE type, const Packet *const packet);

  bool isConnected() { return webSocket.isConnected(); };

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

#endif
