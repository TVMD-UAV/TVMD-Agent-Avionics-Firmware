#include "CommProtocol.h"

#ifdef SERVER
SemaphoreHandle_t CommServer::_map_mutex = NULL;
uint8_t CommServer::agent_id_map[10] = {0};
Perf CommServer::state_health[MAX_NUM_AGENTS];

CommServer::CommServer(uint16_t port) : webSocket(port), CommProtocol(){
};

void CommServer::init(uint8_t agent_id) {
  _map_mutex = xSemaphoreCreateMutex();
  webSocket.begin();
  webSocket.onEvent(
      [&](uint8_t client_id, WStype_t type, uint8_t *payload, size_t length) {
        switch (type) {
        case WStype_DISCONNECTED: 
          // log_w("Client [%u] Disconnected!\n", client_id);
          break;

        case WStype_CONNECTED: {
#ifdef COMM_DEBUG_PRINT
          IPAddress ip = webSocket.remoteIP(client_id);
          log_v("[%u] Connected from %d.%d.%d.%d url: %s\n", client_id, ip[0],
                ip[1], ip[2], ip[3], payload);
#endif
        } break;

        case WStype_TEXT:
          // log_v("[%u] get Text: %s at %ld\n", client_id, payload, micros());
          break;

        case WStype_BIN:
          callback_router(client_id, payload, length);
          break;

        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
          break;
        }
      });

  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    state_health[i] = Perf(NUM_HEALTH_CHECK);
  }

  CommProtocol::init(agent_id);
}

void CommServer::callback_router(uint8_t client_id, uint8_t *payload,
                                 size_t length) {
  uint8_t agent_id = CommProtocol::callback_router(payload, length);

  // set agent id map
  if (agent_id_map[client_id] == 0) {
    if (check_agent_id_valid(agent_id) && (xSemaphoreTake(_map_mutex, 0) == pdTRUE)){
      agent_id_map[client_id] = agent_id;
      xSemaphoreGive(_map_mutex);
    }
    else 
      log_e("Agent id out of range: cid: %d. aid: %d\n", client_id, agent_id);
  } 
}

bool CommServer::send(const Packet *const packet, const size_t len) {
  return broadcastBIN((uint8_t *)packet, len);
}

#endif