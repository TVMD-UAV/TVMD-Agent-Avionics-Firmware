#ifndef COMM_PROTOCOL

#include "Perf.hpp"
#include "configs.h"
#include "freertos/semphr.h"

// Imported from WebSockets.h
#ifdef ARDUINO_ARCH_AVR
#error Version 2.x.x currently does not support Arduino with AVR since there is no support for std namespace of c++.
#error Use Version 1.x.x. (ATmega branch)
#else
#include <functional>
#endif

// #define COMM_DEBUG_PRINT
#define COMM_PROTOCOL
#include <WiFiClientSecure.h>

#include "PacketTypes.h"
#include "Adafruit_Sensor.h"
#include <WebSocketsClient.h>
#include <WebSocketsServer.h>


struct AgentData {
  StatePacket packet;
  uint8_t client_id;
};

typedef std::function<void(const CtrlPacket &)> CtrlCallbackFunc;
typedef std::function<void(const StatePacket &)> StateCallbackFunc;
typedef std::function<void(const InstructPacket &)> InstructCallbackFunc;
typedef std::function<void(uint8_t agent_id)> DisconnectCallbackFunc;

class CommProtocol {
public:
  const char *topic_ctrl_commands = "ctrl/commands";
  const char *topic_ctrl_instruct = "ctrl/instruct";
  const char *topic_state_agent = "state/agent";

  enum PACKET_TYPE {
    CTRL_CMDS = 0,
    CTRL_CMDS_ARRAY,
    CTRL_INST,
    STATE_AGN,
    MSG,
    RAW
  };

  CommProtocol();

  void virtual init(uint8_t agent_id);

  virtual bool send(const Packet *const packet, const size_t len) = 0;

  virtual void update() = 0;

#pragma region[callback function setup]
  void set_ctrl_callback(CtrlCallbackFunc func) { _ctrl_callback = func; };

  void set_state_callback(StateCallbackFunc func) { _state_callback = func; };

  void set_instruct_callback(InstructCallbackFunc func) {
    _instruct_callback = func;
  };

  void set_disconnect_callback(DisconnectCallbackFunc func) {
    _disconnect_callback = func;
  };
#pragma endregion[callback function setup]

protected:
  CtrlCallbackFunc _ctrl_callback;
  StateCallbackFunc _state_callback;
  InstructCallbackFunc _instruct_callback;
  DisconnectCallbackFunc _disconnect_callback;

  uint8_t _agent_id;

  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) = 0;

  void allocate_packet_by_length(Packet *packet, size_t length);

  uint8_t callback_router(uint8_t *payload, size_t length);

  virtual inline void set_connection_lost(uint8_t agent_id) = 0;

  bool check_agent_id_valid(uint8_t agent_id) {
    #ifdef SERVER
    return agent_id > 0 && agent_id <= MAX_NUM_AGENTS;
    #else 
    return agent_id != 0;
    #endif
  };
};

/* Communication server for navigator based on Websocket
 *
 */
class CommServer : public CommProtocol {
public:
  static Perf state_health[MAX_NUM_AGENTS];
  static const uint8_t MAX_NUM_CID = 10;

  CommServer(uint16_t port);

  // Setup event listener
  void init(uint8_t agent_id);

  virtual bool send(const Packet *const packet, const size_t len);

  virtual void update() { webSocket.loop(); };

  /**
   * This function query the agent id from agent id map starting from
   * the given cid (including itself). Once an active agent is found, 
   * the agent id is returned and the corresponding client id is stored 
   * in cid.
   * 
   * @param cid The client id to start the query
   * @return The agent id of the found agent. -1 if no active agent is found.
   */
  static int query_agent_id(uint8_t &cid) {
    for (int i = cid; i < MAX_NUM_CID; i++) {
      if (agent_id_map[i] != 0) {
        cid = i;
        return agent_id_map[i];
      }
    }
    return -1;
  };

protected:
  static SemaphoreHandle_t _map_mutex;
  static uint8_t agent_id_map[MAX_NUM_CID];

  bool broadcastBIN(uint8_t *payload, size_t length = 0) {
    return webSocket.broadcastBIN(payload, length);
  };

  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) {
    return webSocket.sendBIN(client_id, payload, length);
  };

  void callback_router(uint8_t client_id, uint8_t *payload, size_t length);

  virtual inline void set_connection_lost(uint8_t agent_id) {
    _disconnect_callback(agent_id);
  };

private:
  WebSocketsServer webSocket;
};

/* Communication client for agents based on Websocket
 *
 */
class CommClient : public CommProtocol {
public:
  CommClient(uint16_t port, const IPAddress serv_ip)
      : webSocket(), CommProtocol(), _port(port), _serv_addr(serv_ip){};

  static Perf ctrl_health;
  static Perf state_health;

  // Setup event listener
  void init(uint8_t agent_id);

  bool send(const Packet *const packet, const size_t len);

  bool isConnected() { return webSocket.isConnected(); };

  virtual void update();

protected:
  // Only require for agents
  time_t last_update_time;

  virtual bool sendToBIN(uint8_t client_id, uint8_t *payload,
                         size_t length = 0) {
    return webSocket.sendBIN(payload, length);
  };

  virtual inline void set_connection_lost(uint8_t agent_id) {
    _disconnect_callback(0);
  };

private:
  uint16_t _port;
  IPAddress _serv_addr;
  WebSocketsClient webSocket;
};

#endif
