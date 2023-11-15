#include "Core.h"

uint8_t Core::_agent_id;
AGENT_STATE Core::_state;
TaskHandle_t Core::websocket_task_handle;
TaskHandle_t Core::indicator_task_handle;
InstructionHandler Core::instruction_handler;

#ifdef SERVER
SemaphoreHandle_t Core::_agents_mutex = NULL;
volatile AgentData Core::agents[MAX_NUM_AGENTS];
CommServer Core::comm = CommServer(COMM_PORT);
#else
SemaphoreHandle_t Core::_ctrl_mutex = NULL;
SemaphoreHandle_t Core::_state_mutex = NULL;

StatePacket Core::_state_packet;
CtrlPacket Core::_ctrl_packet;

TaskHandle_t Core::state_feedback_handle;

CommClient Core::comm = CommClient(COMM_PORT, server_IP);
Sensors Core::sensor = Sensors();

ServoMotorDriver Core::x_servo(
    ServoMotorConfigs(MotorConfigs(MOTOR_X_SERVO_PIN, 1100, 1900, 1500, 180, 4096), 0));
ServoMotorDriver Core::y_servo(
    ServoMotorConfigs(MotorConfigs(MOTOR_Y_SERVO_PIN, 1100, 1900, 1500, 180, 4096), 0));
ESCMotorDriver Core::esc_p1(
    ESCMotorConfigs(MotorConfigs(MOTOR_ESC_P1_PIN, 1000, 2000, 1500, 100, 4096), 95));
ESCMotorDriver Core::esc_p2(
    ESCMotorConfigs(MotorConfigs(MOTOR_ESC_P2_PIN, 1000, 2000, 1500, 100, 4096), 95));
#endif

Indicator Core::indicator;

void Core::init() {
  #ifdef SERVER
  _agents_mutex = xSemaphoreCreateMutex();
  #else 
  _ctrl_mutex = xSemaphoreCreateMutex();
  _state_mutex = xSemaphoreCreateMutex();
  #endif

  indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::FLASHING, INDICATOR_GREEN);

  // Attaching indicator task to core 0 with the minimum task priority (1)
  xTaskCreatePinnedToCore(indicator_update, /* Function to implement the task */
                          "indicator_updating",   /* Name of the task */
                          2000,                   /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          1,                      /* Priority of the task */
                          &indicator_task_handle, /* Task handle. */
                          0); /* Core where the task should run */

  get_agent_id();

  // Waiting for WiFi connection
  indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::DOUBLE_IMPULSE, INDICATOR_GREEN);
  Wifi_connection_setup();

#ifndef SERVER // Initialize sensors and actuators
  if (sensor.init() != Sensors::SENSOR_ERROR::SENSOR_OK){
    log_e("Sensor init failed!");
    indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::FAST, INDICATOR_RED);  
    while (true) 
      ;
  };

  x_servo.init();
  y_servo.init();
  esc_p1.init();
  esc_p2.init();
#endif
  indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::FAST, INDICATOR_GREEN);

#ifdef COMM_SETUP // Initialize websocket services
#ifdef SERVER
  /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](CtrlPacket &packet) {
    // TODO: do something
  });

  comm.set_state_callback([](StatePacket &packet) {
    uint8_t aidx;
    if (xSemaphoreTake(_agents_mutex, portMAX_DELAY) == pdTRUE) {
      aidx = get_aidx(packet.agent_id);
      memcpy((void*)&(agents[aidx].packet), &packet, sizeof(packet));
      CommServer::state_health[aidx].feed_data(packet.id, packet.time, micros());
      xSemaphoreGive(_agents_mutex);
    }
    agents[aidx].packet.time = millis();

    // check agent state
    if (_state == AGENT_STATE::LOST_CONN) {
      if (check_all_agent_alive())
        set_state(AGENT_STATE::INITED);
    }
  });

  comm.set_instruct_callback([](InstructPacket &packet) {
    // TODO: do something
    log_d("Receive instruction %d\n", packet.instruction);
  });

  comm.set_disconnect_callback([](uint8_t agent_id) {
    if (xSemaphoreTake(_agents_mutex, portMAX_DELAY) == pdTRUE) {
      agents[get_aidx(agent_id)].packet.state = AGENT_STATE::LOST_CONN;
      xSemaphoreGive(_agents_mutex);
    }
    set_armed(false);
    set_state(AGENT_STATE::LOST_CONN);
  });
#else
  /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](CtrlPacket &packet) {
    if (_state == AGENT_STATE::LOST_CONN) {
      // relive
      set_state(AGENT_STATE::INITED);
    } else {
      if (packet.agent_id == _agent_id) {
        _ctrl_packet = packet;
        // Core::x_servo.write(packet.eta_x);
        // Core::y_servo.write(packet.eta_y);
        // Core::esc_p1.write(packet.omega_p1);
        // Core::esc_p2.write(packet.omega_p2);
        Core::x_servo.raw_write(packet.eta_x);
        Core::y_servo.raw_write(packet.eta_y);
        Core::esc_p1.raw_write(packet.omega_p1);
        Core::esc_p2.raw_write(packet.omega_p2);
        CommClient::ctrl_health.feed_data(micros());
      }
    }
  });

  comm.set_state_callback([](StatePacket &packet) {
    // TODO: do something
  });

  comm.set_instruct_callback([](InstructPacket &packet) {
    log_v("Receive instruction %d\n", packet.instruction);
    switch (packet.instruction) {
    case InstructPacket::INSTRUCT_TYPE::ARMING:
      set_armed(true);
      break;
    case InstructPacket::INSTRUCT_TYPE::DISARMING:
    case InstructPacket::INSTRUCT_TYPE::EMERGENCY_STOP:
      set_armed(false);
      break;
    case InstructPacket::INSTRUCT_TYPE::IDLE:
    default:
      break;
    }
    if (_state == AGENT_STATE::LOST_CONN) {
      // relive
      set_armed(false);
      set_state(AGENT_STATE::INITED);
    }
  });

  comm.set_disconnect_callback([](uint8_t agent_id) {
    if (_state != AGENT_STATE::LOST_CONN) {
      set_armed(false);
      set_state(AGENT_STATE::LOST_CONN);
    }
  });

  _state_packet.agent_id = _agent_id;
#endif
  log_v("Communication callback set!");

  comm.init(_agent_id);
  log_v("Communication init!");
#endif 

  /**
   * This callback function is called when instructions are received from the external 
   * I2C bus. It directly set commands to the actuators.
   */
  instruction_handler.set_instruction_callback([](const Instruction &instruction) {
    // log_i("Receiving instructions");
    set_armed(instruction.data.armed);

#ifdef SERVER
    // log_i("Instruction: %6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t",
    //   instruction.data.control[0], instruction.data.control[1], instruction.data.control[2], instruction.data.control[3],
    //   instruction.data.control[4], instruction.data.control[5], instruction.data.control[6], instruction.data.control[7]);
    if (xSemaphoreTake(_packet_mutex, portMAX_DELAY) == pdTRUE) {
      _packet.id += 1;
      _packet.time = micros();

      for (int i = 0; i < MAX_NUM_AGENTS; i++) {
        if (instruction.data.type == Instruction::ControlTypes::MOTORS ) {
          _packet.packets[i].omega_p1 = instruction.data.control[2*i];
          _packet.packets[i].omega_p2 = instruction.data.control[2*i+1];
        } else if (instruction.data.type == Instruction::ControlTypes::SERVOS) {
          _packet.packets[i].eta_x = instruction.data.control[2*i];
          _packet.packets[i].eta_y = instruction.data.control[2*i+1];
        }
        _packet.packets[i].id = _packet.id;
        _packet.packets[i].time = _packet.time;
        _packet.packets[i].agent_id = i + 1;
        
        // TODO: send data at once
        Core::comm.send(&_packet.packets[i], sizeof(CtrlPacket));
      }
      xSemaphoreGive(_packet_mutex);
    }
#endif 
  });

  instruction_handler.init();

  // Initialize daemon tasks
  indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::LONG, INDICATOR_GREEN);

#pragma region[task hook]
  // TODO: check stack usages
  xTaskCreatePinnedToCore(websocket_loop, /* Function to implement the task */
                          "websocket_updating",   /* Name of the task */
                          7000,                   /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          5,                      /* Priority of the task */
                          &websocket_task_handle, /* Task handle. */
                          1); /* Core where the task should run */

#ifndef SERVER
  xTaskCreatePinnedToCore(state_feedback,   /* Function to implement the task */
                          "state_feedback", /* Name of the task */
                          5000,             /* Stack size in words */
                          NULL,             /* Task input parameter */
                          5,                /* Priority of the task */
                          &state_feedback_handle, /* Task handle. */
                          1); /* Core where the task should run */
#endif
  
  // System start up complete
  indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::IMPULSE, INDICATOR_BLUE);
  indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, INDICATOR_BLUE);
  log_v("Thread init!");

#pragma endregion[task hook]


  // TODO: inited is not equal to ready (maybe not connected)
  set_state(AGENT_STATE::INITED);
}

void Core::Wifi_connection_setup() {
  const IPAddress local_IP(gateway[0], gateway[1], gateway[2], _agent_id + 1);
#ifdef SERVER
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(server_IP, gateway, subnet);
  const int maximum_connection = 4;
  WiFi.softAP(WIFI_SSID, WIFI_PSWD, 1, false, maximum_connection);
  log_d("AP started with IP address: %s\n", WiFi.softAPIP().toString());
#else
  log_v("Connecting to %s\n", (String)WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  log_v("\nWiFi connected with IP address:  %s\n", WiFi.softAPIP().toString());
#endif
}

bool Core::set_armed(bool armed) {
#ifdef SERVER
  if (_state == AGENT_STATE::ARMED)
    return true;
  InstructPacket arming;
  arming.time = millis();

  if (armed) {
    if (_state == AGENT_STATE::ARMING) {
      // Waiting on all agents set
      if (check_all_agent_alive(AGENT_STATE::ARMED)) {
        indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, INDICATOR_RED);
        set_state(AGENT_STATE::ARMED);
        return true;
      } else {
        // Resend arming request
        arming.instruction = InstructPacket::INSTRUCT_TYPE::ARMING;
        comm.send(&arming, sizeof(arming));
        return false;
      }
    }

    // 1. Check all agents are available (inited)
    bool all_agent_available = check_all_agent_alive();

    // 2. Send messages to agents
    if (all_agent_available && set_state(AGENT_STATE::ARMING)) {
      arming.instruction = InstructPacket::INSTRUCT_TYPE::ARMING;
      comm.send(&arming, sizeof(arming));
      return false;
    } else {
      log_e("Trying to set arm with state %d, %s\n", _state,
            all_agent_available ? "all agents alive" : "some agents lost");
    }
  } else {
    // dis-arming
    indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, INDICATOR_BLUE);
    arming.instruction = InstructPacket::INSTRUCT_TYPE::DISARMING;
    comm.send(&arming, sizeof(arming));
    return true;
  }
  return false;
#else
  const AGENT_STATE target = armed ? AGENT_STATE::ARMED : AGENT_STATE::INITED;
  if (set_state(target)) {
    indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, 
      (armed ? INDICATOR_RED : INDICATOR_BLUE));
    Core::x_servo.set_armed(armed);
    Core::y_servo.set_armed(armed);
    Core::esc_p1.set_armed(armed);
    Core::esc_p2.set_armed(armed);
#ifdef COMM_DEBUG_PRINT
    log_d("Succeed to set %s\n", armed ? "arm" : "disarm");
#endif
    return true;
  } else {
#ifdef COMM_DEBUG_PRINT
    log_e("Trying to set %s with state %d\n", armed ? "arm" : "disarm", _state);
#endif
    return false;
  }
#endif
}

int Core::get_agent_id() {
  static bool agent_id_init = false;
  if (!agent_id_init) {
    // not read from memory yet
    EEPROM.begin(EEPROM_SIZE);
#ifdef WRITE_AGENT_ID
    EEPROM.write(EEPROM_AGENT_ID_ADDR, AGENT_ID);
    EEPROM.commit();
#endif
    _agent_id = EEPROM.read(EEPROM_AGENT_ID_ADDR);
    log_v("Agent id: %d, \n", _agent_id);
    agent_id_init = true;
  }
  return _agent_id;
}

void Core::send_ctrl(const CtrlPacketArray *const packet) {
  comm.send(packet, sizeof(CtrlPacketArray));
}

void Core::print_summary() {
#ifdef SERVER
  // agent id, client id, agent state
  uint8_t cid = 0;

  log_i("Navigator state: %d, M-Instr: %6.2f, S-Instr:%6.2f\nAid\t Cid\t State\t S-FPS\t M-Instr\t S-Instr \t Sensor",
    _state, instruction_handler.get_motor_fps(), instruction_handler.get_servo_fps());
  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    // Query agent id by client id. 
    // If cid is an active client, aid is the agent id. Otherwise, aid is -1.
    const int aid = CommServer::query_agent_id(cid);
    if (aid == -1) break;

    const uint8_t aidx = get_aidx(aid);

    // Core module save agent data indexed by agent id (aid)
    Quaternion q;
    AGENT_STATE state;
    float fps;
    if (xSemaphoreTake(_agents_mutex, portMAX_DELAY) == pdTRUE) {
      memcpy((void*)&q, (void*)&(agents[aidx].packet.s.orientation), sizeof(Quaternion));
      state = agents[aidx].packet.state;
      fps = CommServer::state_health[aidx].get_fps();
      xSemaphoreGive(_agents_mutex);  
    }
    printf("[%d]\t %d\t %d\t %6.2f\t %6.2f, \t%6.2f, \t%6.2f, \t%6.2f\n", 
      aid, cid, state, fps, q.q0, q.q1, q.q2, q.q3);
    
    // Move to next client id
    cid += 1;
  }
#else
  log_i("\nAid\t State\t C-FPS\t S-FPS\t Sensor");
  const double eta_x = _ctrl_packet.eta_x;
  const double eta_y = _ctrl_packet.eta_y;
  const double omega_p1 = _ctrl_packet.omega_p1;
  const double omega_p2 = _ctrl_packet.omega_p2;
  const float ctrl_fps = CommClient::ctrl_health.get_fps();
  const float state_fps = CommClient::state_health.get_fps();
  printf("[%d]\t %d\t %6.2f\t %6.2f, \t%6.2f, \t%6.2f, \t%6.2f, \t%6.2f\n", 
    _agent_id, _state, eta_x, eta_y, omega_p1, omega_p2, ctrl_fps, state_fps);
  
#endif

  // print stack memory usage
  printf("Free stack of indicator routine: %d\n", uxTaskGetStackHighWaterMark(indicator_task_handle));
  printf("Free stack of websocket routine: %d\n", uxTaskGetStackHighWaterMark(websocket_task_handle));

  // print heap memory usage
  printf("Free heap: %d\n", ESP.getFreeHeap());
}


void Core::print_instructions() {
  #ifdef SERVER
  log_i("\nAid\t eta_x\t eta_y\t motor1\t motor2");

  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    double eta_x, eta_y, omega_p1, omega_p2;
    if (xSemaphoreTake(_packet_mutex, portMAX_DELAY) == pdTRUE) {
      eta_x = _packet.packets[i].eta_x;
      eta_y = _packet.packets[i].eta_y;
      omega_p1 = _packet.packets[i].omega_p1;
      omega_p2 = _packet.packets[i].omega_p2;
    }
    xSemaphoreGive(_packet_mutex);
    printf("[%d: %d]\t %6.2f\t %6.2f, \t%6.2f, \t%6.2f\n", 
      i, _state, eta_x, eta_y, omega_p1, omega_p2);
  }
  // TODO: This causes crashing
  #endif
}

/**
 * set the state to the target state according to finite-state machine rule
 * @param target the target state
 * @return true if ok
 */
bool Core::set_state(AGENT_STATE target) {
  if (target == _state)
    return false;
  
  switch(target) {
    case AGENT_STATE::INITED:
      indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::IMPULSE, INDICATOR_BLUE);
      break;

    case AGENT_STATE::ARMING:
      if (_state != AGENT_STATE::INITED)
        return false;
      indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::IMPULSE, 0x00FFFF00);
      break;

    case AGENT_STATE::ARMED:
    #ifdef SERVER
      if (_state != AGENT_STATE::ARMING)
        return false;
    #else 
      if (_state != AGENT_STATE::INITED)
        return false;
    #endif 
      indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::IMPULSE, 0x00FF0000);
      break;

    case AGENT_STATE::LOST_CONN:
      indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::HASTILY, 0x00FF00FF);
      break;
  }
  _state = target;
  log_i("[STATE] Succeed to set state to %d", target);
  return true;
}
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
          CommClient::state_health.feed_data(_state_packet.id, micros(), micros());
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
#endif

    // To allow other threads have chances to join
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
    // yield();
  }
#endif
}

#ifdef SERVER
bool Core::check_all_agent_alive(AGENT_STATE target) {
  bool all_agent_available = true;
  
  if (xSemaphoreTake(_agents_mutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < MAX_NUM_AGENTS; i++) {
      all_agent_available &= (agents[i].packet.state == target);
    }
    xSemaphoreGive(_agents_mutex);
  }
  return all_agent_available;
}
#endif
