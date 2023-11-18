#include "Core.h"

/**
 * Continuously sending states to the navigator
 */
#ifndef SERVER
void Core::state_feedback(void *parameter) {
  
  static time_t last_summary_time = 0;
  while (true) {
    sensor.update();

    // Beacon message to server
    #ifdef COMM_SETUP
    if (sensor.available()){
      if (xSemaphoreTake(_state_mutex, portMAX_DELAY) == pdTRUE) {
        sensor.state_packet_gen(&_state_packet);
        _state_packet.state = _state;
        _state_packet.agent_id = _agent_id;
        _state_packet.id += 1;

        // send the packet to server
        if (comm.send(&_state_packet, sizeof(_state_packet))) {
          CommClient::state_health.feed_data(micros());
          last_summary_time = millis();
        }
        xSemaphoreGive(_state_mutex);
      }
    }
    #endif
    
    // pass control to another task waiting to be executed
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // yield();
  }
}
#endif

void Core::websocket_loop(void *parameter) {
#ifdef COMM_SETUP
  for (;;) {
    comm.update();

#ifdef SERVER
    // check for connection lost
    for (int i = 0; i < MAX_NUM_AGENTS; i++) {  
      if (xSemaphoreTake(_agents_mutex, portMAX_DELAY) == pdTRUE) {
        if (millis() - agents[i].packet.time > MAX_TIME_OUT)
          agents[i].packet.state = AGENT_STATE::LOST_CONN;
        xSemaphoreGive(_agents_mutex);
      }
    }

    if (_state == AGENT_STATE::ARMING) {
      // check for all agents armed
      if (check_all_agent_alive(AGENT_STATE::ARMED))
        set_state(AGENT_STATE::ARMED);
    }

    if (_packet_ready) {
      if (xSemaphoreTake(_packet_mutex, portMAX_DELAY) == pdTRUE) {
        _packet.time = micros();
        Core::comm.send(&_packet, sizeof(CtrlPacketArray));
        xSemaphoreGive(_packet_mutex);
      }
      _packet_ready = false;
    }
#endif

    // To allow other threads have chances to join
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
    // yield();
  }
#endif
}
