#include "Core.h"

TaskHandle_t Core::websocket_task_handle;
TaskHandle_t Core::indicator_task_handle;
TaskHandle_t Core::instruction_handle;
TaskHandle_t Core::regular_task_handle;

#if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
TaskHandle_t Core::state_feedback_handle;
#endif
/**
 * Continuously sending states to the navigator
 */
#if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
void Core::state_feedback(void *parameter) {
  
  while (true) {
    if (sensor.update() == 0) {
      #ifndef SERVER
      // Beacon message to navigator
      #ifdef COMM_SETUP
      if (sensor.available()){
        if (xSemaphoreTake(_state_mutex, 0) == pdTRUE) {
          sensor.state_packet_gen(&_state_packet);
          _state_packet.state = _state;
          _state_packet.agent_id = _agent_id;
          _state_packet.id += 1;
          xSemaphoreGive(_state_semphr);
          xSemaphoreGive(_state_mutex);
        }
      }
      #endif
      #else
      // Beacon message to the companion computer
      if (xSemaphoreTake(_state_mutex, 0) == pdTRUE) {
        imu_echo.set_imu_data(sensor.get_sensor_data());
        xSemaphoreGive(_state_mutex);
      }
      #endif
    }
    // pass control to another task waiting to be executed
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
#endif

void Core::websocket_loop(void *parameter) {
#ifdef COMM_SETUP
  for (;;) {
    comm.update();
    // To allow other threads have chances to join
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
#endif
}

void Core::instruction_loop(void *parameter) {
  for (;;) {
    instruction_handler.update();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Core::regular_update(void *parameter) {
  // This process must br run at the process the same with the websocket loop
  for (;;) {
    state_machine_update();

    #ifdef SERVER
    // Synchronize the packet
    if (xSemaphoreTake(_packet_semphr, 0) == pdTRUE) {
      if (xSemaphoreTake(_packet_mutex, 0) == pdTRUE) {
        if (_packet_ready) {
          Core::comm.send(&_packet, sizeof(CtrlPacketArray));
          _packet_ready = false;
        }
        xSemaphoreGive(_packet_mutex);
      }
    }
    #endif

    #if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
    // send the packet to server
    if (xSemaphoreTake(_state_semphr, 0) == pdTRUE) {
      if (xSemaphoreTake(_state_mutex, 0) == pdTRUE) {
        if (comm.send(&_state_packet, sizeof(_state_packet))) {
          CommClient::state_health.feed_data(micros());
        }
        xSemaphoreGive(_state_mutex);
      }
    }
    #endif 
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
};
