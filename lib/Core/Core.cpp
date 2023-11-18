#include "Core.h"

uint8_t Core::_agent_id;
AGENT_STATE Core::_state;
TaskHandle_t Core::websocket_task_handle;
TaskHandle_t Core::indicator_task_handle;
InstructionHandler Core::instruction_handler;

#ifdef SERVER
SemaphoreHandle_t Core::_packet_mutex;
CtrlPacketArray Core::_packet;
bool Core::_packet_ready{false};
bool Core::_armed{false};

SemaphoreHandle_t Core::_agents_mutex{NULL};
volatile AgentData Core::agents[MAX_NUM_AGENTS];

CommServer Core::comm = CommServer(COMM_PORT);
#else
SemaphoreHandle_t Core::_ctrl_mutex{NULL};
SemaphoreHandle_t Core::_state_mutex{NULL};

StatePacket Core::_state_packet;
CtrlPacket Core::_ctrl_packet;

TaskHandle_t Core::state_feedback_handle;

CommClient Core::comm = CommClient(COMM_PORT, server_IP);
Sensors Core::sensor = Sensors();

// pin, min, max, mid, value_range, denominator
ServoMotorDriver Core::x_servo(
    ServoMotorConfigs(MotorConfigs(MOTOR_X_SERVO_PIN, 1100, 1900, 1500, 180, 4096, 0), 0));
ServoMotorDriver Core::y_servo(
    ServoMotorConfigs(MotorConfigs(MOTOR_Y_SERVO_PIN, 1100, 1900, 1500, 180, 4096, 1), 0));
ESCMotorDriver Core::esc_p1(
    ESCMotorConfigs(MotorConfigs(MOTOR_ESC_P1_PIN, 1000, 2000, 1000, 100, 4096, 2), 95));
ESCMotorDriver Core::esc_p2(
    ESCMotorConfigs(MotorConfigs(MOTOR_ESC_P2_PIN, 1000, 2000, 1000, 100, 4096, 3), 95));
#endif

Indicator Core::indicator;

void Core::init() {
  #ifdef SERVER
  _packet.time = 0;
  _packet.id = 0;
  _packet_mutex = xSemaphoreCreateMutex();
  _agents_mutex = xSemaphoreCreateMutex();
  #else 
  _ctrl_mutex = xSemaphoreCreateMutex();
  _state_mutex = xSemaphoreCreateMutex();
  _state_packet.agent_id = _agent_id;
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
  comm_callback_setup();
  log_v("Communication callback set!");

  comm.init(_agent_id);
  log_v("Communication init!");
#endif 

  instruction_callback_setup();

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
  if (WiFi.status() != WL_CONNECTED) 
    set_state(AGENT_STATE::LOST_CONN);
  else 
    set_state(AGENT_STATE::INITED);

  // TODO: Move indicator update to set_state()
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
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  // }
  // log_v("\nWiFi connected with IP address:  %s\n", WiFi.softAPIP().toString());
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

#ifdef SERVER
void Core::send_ctrl(const CtrlPacketArray *const packet) {
  comm.send(packet, sizeof(CtrlPacketArray));
}
#endif

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
      aid, cid, state, fps, 
      q.q0, q.q1, q.q2, q.q3);
    
    // Move to next client id
    cid += 1;
  }
#else
  log_i("\nAid\t State\t C-FPS\t S-FPS\t eta-x\t eta-y\t motor1\t motor2");
  const double eta_x = _ctrl_packet.eta_x;
  const double eta_y = _ctrl_packet.eta_y;
  const double omega_p1 = _ctrl_packet.omega_p1;
  const double omega_p2 = _ctrl_packet.omega_p2;
  const float ctrl_fps = CommClient::ctrl_health.get_fps();
  const float state_fps = CommClient::state_health.get_fps();
  printf("[%d]\t %d\t %6.2f\t %6.2f, \t%6.2f, \t%6.2f, \t%6.2f, \t%6.2f\n", 
    _agent_id, _state, ctrl_fps, state_fps, eta_x, eta_y, omega_p1, omega_p2);
  
#endif

  // print stack memory usage
  printf("Free stack of indicator routine: %d\n", uxTaskGetStackHighWaterMark(indicator_task_handle));
  printf("Free stack of websocket routine: %d\n", uxTaskGetStackHighWaterMark(websocket_task_handle));

  // print heap memory usage
  printf("Free heap: %d\n", ESP.getFreeHeap());
}

void Core::print_instructions() {
  #ifdef SERVER
  log_i("\nAid\t eta_x\t eta_y\t motor1\t motor2\t armed: %s", _armed ? "true" : "false");

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
