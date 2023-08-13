#include "Core.h"

uint8_t Core::_agent_id;
AGENT_STATE Core::_state;
TaskHandle_t Core::websocket_task_handle;
#define COMM_DEBUG_PRINT

#ifdef SERVER
AgentData Core::agents[MAX_NUM_AGENTS];
CommServer Core::comm = CommServer(COMM_PORT);
#else
StatePacket Core::packet;
CtrlPacket Core::_ctrl_packet;

TaskHandle_t Core::state_feedback_handle;

CommClient Core::comm = CommClient(COMM_PORT, server_IP);
Sensors Core::sensor = Sensors();

ServoMotorDriver Core::x_servo(
    ServoMotorConfigs(MotorConfigs(18, 1100, 1900, 1500, 180, 4096), 0));
ServoMotorDriver Core::y_servo(
    ServoMotorConfigs(MotorConfigs(19, 1100, 1900, 1500, 180, 4096), 0));
ESCMotorDriver Core::esc_p1(
    ESCMotorConfigs(MotorConfigs(32, 1000, 2000, 1500, 100, 4096), 95));
ESCMotorDriver Core::esc_p2(
    ESCMotorConfigs(MotorConfigs(33, 1000, 2000, 1500, 100, 4096), 95));
#endif

void Core::init() {
  get_agent_id();

  Wifi_connection_setup();

#ifndef SERVER
  if (_agent_id == 2)
    sensor.init(14, 15);
  else
    sensor.init();

  x_servo.init();
  y_servo.init();
  esc_p1.init();
  esc_p2.init();
#endif

#ifdef SERVER
  /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](CtrlPacket &packet) {
    // TODO: do something
  });

  comm.set_state_callback([](StatePacket packet) {
    // TODO: do something
    const uint8_t agent_array_idx = packet.agent_id - 1;
    assert(agent_array_idx >= 0 && agent_array_idx < MAX_NUM_AGENTS);
    agents[agent_array_idx].packet = packet;
    agents[agent_array_idx].packet.time = millis();

    CommServer::state_health[agent_array_idx].feed_data(packet.id, packet.time,
                                                        micros());

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
    if (agent_id > 0 && agent_id <= MAX_NUM_AGENTS)
      agents[agent_id - 1].packet.state = AGENT_STATE::LOST_CONN;
    set_armed(false);
    set_state(AGENT_STATE::LOST_CONN);
  });
#else
  /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](CtrlPacket packet) {
    if (_state == AGENT_STATE::LOST_CONN) {
      // relive
      set_state(AGENT_STATE::INITED);
    } else {
      _ctrl_packet = packet;
      Core::x_servo.write(packet.eta_x);
      Core::y_servo.write(packet.eta_y);
      Core::esc_p1.write(packet.omega_p1);
      Core::esc_p2.write(packet.omega_p2);
      CommClient::ctrl_health.feed_data(packet.id, packet.time, micros());
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
      set_state(AGENT_STATE::INITED);
    }
  });

  comm.set_disconnect_callback([](uint8_t agent_id) {
    if (_state != AGENT_STATE::LOST_CONN) {
      set_armed(false);
      set_state(AGENT_STATE::LOST_CONN);
    }
  });

  packet.agent_id = _agent_id;
#endif
  log_v("Communication callback set!");

  comm.init(_agent_id);
  log_v("Communication init!");

#pragma region[task hook]
  // TODO: check stack usages
  xTaskCreatePinnedToCore(websocket_loop, /* Function to implement the task */
                          "websocket_updating",   /* Name of the task */
                          15000,                  /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          1,                      /* Priority of the task */
                          &websocket_task_handle, /* Task handle. */
                          1); /* Core where the task should run */

#ifndef SERVER
  xTaskCreatePinnedToCore(state_feedback,   /* Function to implement the task */
                          "state_feedback", /* Name of the task */
                          5000,             /* Stack size in words */
                          NULL,             /* Task input parameter */
                          10,               /* Priority of the task */
                          &state_feedback_handle, /* Task handle. */
                          1); /* Core where the task should run */
#endif
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
    arming.instruction = InstructPacket::INSTRUCT_TYPE::DISARMING;
    comm.send(&arming, sizeof(arming));
    return true;
  }
  return false;
#else
  const AGENT_STATE target = armed ? AGENT_STATE::ARMED : _state;
  if (set_state(target)) {
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
  char str[200];
  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    char acc_str[40];
    agents[i].packet.print(acc_str);
    sprintf(str, "%s\n[%d]\t %d\t %d\t %6.2f\t %s", str, i,
            CommServer::agent_id_map[i], agents[i].packet.state,
            CommServer::state_health[i].get_fps(), acc_str);
  }
  char server_state[40];
  sprintf(server_state, "\nNavigator state: %d", _state);
  log_i("%s\nAid\t Cid\t State\t S-FPS\t Sensor\t, %s", server_state, str);
#else
  char str[200];
  char ctrl_str[40];
  _ctrl_packet.print(ctrl_str);
  sprintf(str, "\n[%d]\t %d\t %6.2f\t %6.2f\t %s", _agent_id, _state,
          CommClient::ctrl_health.get_fps(), CommClient::state_health.get_fps(),
          ctrl_str);
  log_i("\nAid\t State\t C-FPS\t S-FPS\t Sensor\t, %s", str);
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
  switch (_state) {
  case AGENT_STATE::STARTING:
    if (target != AGENT_STATE::INITED)
      return false;
    break;
#ifdef SERVER
  // For the navigator: INIT -> ARMING -> ARMED
  // The state is permitted to be ARMED after checking all agents are set
  case AGENT_STATE::INITED:
    if (target != AGENT_STATE::ARMING && target != AGENT_STATE::LOST_CONN)
      return false;
    break;
#else
  // For agents: INIT -> ARMED
  case AGENT_STATE::INITED:
    if (target != AGENT_STATE::ARMED && target != AGENT_STATE::LOST_CONN)
      return false;
    break;
#endif
#ifdef SERVER
  case AGENT_STATE::ARMING:
    if (target != AGENT_STATE::ARMED && target != AGENT_STATE::INITED &&
        target != AGENT_STATE::LOST_CONN)
      return false;
    break;
#endif
  case AGENT_STATE::ARMED:
    if (target != AGENT_STATE::INITED && target != AGENT_STATE::LOST_CONN)
      return false;
    break;
  case AGENT_STATE::LOST_CONN:
    if (target != AGENT_STATE::INITED)
      return false;
    break;
  default:
    return false;
    break;
  }
  _state = target;
  log_i("[STATE] Succeed to set state to %d\n", target);
  return true;
}
/**
 * Continuously sending states to the navigator
 */
#ifndef SERVER
void Core::state_feedback(void *parameter) {
  static time_t last_summary_time = 0;
  while (true) {
    if (millis() - last_summary_time >= 500) {
      // write sensor data into packet
      bool ret = sensor.state_packet_gen(&packet);
      packet.state = _state;
      packet.agent_id = _agent_id;

      // send the packet to server
      if (ret && comm.send(&packet, sizeof(packet))) {
        CommClient::state_health.feed_data(packet.id, micros(), micros());
        last_summary_time = millis();
      }
    }
    delay(10);
  }
}
#endif

void Core::websocket_loop(void *parameter) {
  for (;;) {
    comm.update();

#ifdef SERVER
    // check for connection lost
    for (int i = 0; i < MAX_NUM_AGENTS; i++) {
      if (millis() - agents[i].packet.time > MAX_TIME_OUT)
        agents[i].packet.state = AGENT_STATE::LOST_CONN;
    }

    if (_state == AGENT_STATE::ARMING) {
      // check for all agents armed
      if (check_all_agent_alive(AGENT_STATE::ARMED))
        set_state(AGENT_STATE::ARMED);
    }
#endif

    // To allow other threads have chances to join
    delay(1);
  }
}

#ifdef SERVER
bool Core::check_all_agent_alive(AGENT_STATE target) {
  bool all_agent_available = true;
  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    all_agent_available &= (agents[i].packet.state == target);
  }
  return all_agent_available;
}
#endif
