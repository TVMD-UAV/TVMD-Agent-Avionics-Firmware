#ifndef COMM_PROTOCOL

#include "benchmark.h"
#include "configs.h"

// Imported from WebSockets.h
#ifdef ARDUINO_ARCH_AVR
#error Version 2.x.x currently does not support Arduino with AVR since there is no support for std namespace of c++.
#error Use Version 1.x.x. (ATmega branch)
#else
#include <functional>
#endif

#define COMM_PROTOCOL
#include <WiFiClientSecure.h>

#include "Adafruit_Sensor.h"
#include <WebSocketsClient.h>
#include <WebSocketsServer.h>

struct Packet {
  uint32_t time;
  uint8_t id;
  uint8_t agent_id;
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
  uint8_t state;
  sensors_vec_t acc;
  sensors_vec_t gyro;
  float temperature;
  float pressure;
  float altitude;
};

struct InstructPacket : public Packet {
  uint32_t instruction;
};

struct MsgPacket : public Packet {
  uint8_t msg[400];
};

typedef std::function<void(CtrlPacket&)> CtrlCallbackFunc;
typedef std::function<void(StatePacket&)> StateCallbackFunc;
typedef std::function<void(InstructPacket&)> InstructCallbackFunc;

class CommProtocol {
public:
  const char *topic_ctrl_commands = "ctrl/commands";
  const char *topic_ctrl_instruct = "ctrl/instruct";
  const char *topic_state_agent = "state/agent";

  enum PACKET_TYPE { CTRL_CMDS = 0, CTRL_INST, STATE_AGN, MSG, RAW };

  CommProtocol();

  void virtual init();

  virtual bool send(PACKET_TYPE type, const Packet *const packet) = 0;

  virtual void update() = 0;

  void set_ctrl_callback(CtrlCallbackFunc func) {_ctrl_callback = func;};

  void set_state_callback(StateCallbackFunc func) {_state_callback = func;};

  void set_instruct_callback(InstructCallbackFunc func) {_instruct_callback = func;};

protected:
  CtrlCallbackFunc _ctrl_callback;
  StateCallbackFunc _state_callback;
  InstructCallbackFunc _instruct_callback;
  
  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) = 0;

  int get_packet_len(PACKET_TYPE type);

  void allocate_packet_by_length(Packet *packet, size_t length);

  void callback_router(uint8_t *payload, size_t length);
};

class CommServer : public CommProtocol {
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

class CommClient : public CommProtocol {
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
