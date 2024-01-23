#include "Core.h"

uint8_t Core::_agent_id;
volatile AGENT_STATE Core::_state;
volatile AGENT_STATE Core::_last_state{AGENT_STATE::STARTING};
InstructionHandler Core::instruction_handler;


#ifdef SERVER
SemaphoreHandle_t Core::_packet_mutex;
SemaphoreHandle_t Core::_packet_semphr;
CtrlPacketArray Core::_packet;
bool Core::_packet_ready{false};
bool Core::_nav_armed{false};

SemaphoreHandle_t Core::_agents_mutex{NULL};
volatile AgentData Core::agents[MAX_NUM_AGENTS];

CommServer Core::comm = CommServer(COMM_PORT);

#ifdef ENABLE_SERVER_IMU_ECHO
Sensors Core::sensor = Sensors();
ImuEchoHandler Core::imu_echo = ImuEchoHandler();
SemaphoreHandle_t Core::_state_mutex{NULL};
SemaphoreHandle_t Core::_state_semphr{NULL};
#endif

#else
SemaphoreHandle_t Core::_ctrl_mutex{NULL};
SemaphoreHandle_t Core::_state_mutex{NULL};
SemaphoreHandle_t Core::_state_semphr{NULL};

StatePacket Core::_state_packet;
CtrlPacket Core::_ctrl_packet;

CommClient Core::comm = CommClient(COMM_PORT, server_IP);
Sensors Core::sensor = Sensors();

// pin, min, max, mid, value_range, denominator
ServoMotorDriver Core::x_servo(
    ServoMotorConfigs(MotorConfigs(MOTOR_X_SERVO_PIN, 800, 2200, 1500, 180, 4096, 0), 0));
ServoMotorDriver Core::y_servo(
    ServoMotorConfigs(MotorConfigs(MOTOR_Y_SERVO_PIN, 800, 2200, 1500, 180, 4096, 1), 0));
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
  _packet_semphr = xSemaphoreCreateBinary();
  _agents_mutex = xSemaphoreCreateMutex();
  #ifdef ENABLE_SERVER_IMU_ECHO
  _state_mutex = xSemaphoreCreateMutex();
  _state_semphr = xSemaphoreCreateBinary();
  #endif
  #else 
  _ctrl_mutex = xSemaphoreCreateMutex();
  _state_mutex = xSemaphoreCreateMutex();
  _state_semphr = xSemaphoreCreateBinary();
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

// Initialize sensors
#if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
  if (sensor.init() != Sensors::SENSOR_ERROR::SENSOR_OK){
    log_e("Sensor init failed!");
    indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::FAST, INDICATOR_RED);  
    while (true) 
      ;
  };
#endif

#ifdef ENABLE_SERVER_IMU_ECHO
  imu_echo.init();
#endif

#ifndef SERVER // Initialize actuators
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

  // Initialize daemon tasks
  indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::LONG, INDICATOR_GREEN);

#pragma region[task hook]
  // TODO: check stack usages
  xTaskCreatePinnedToCore(websocket_loop,         /* Function to implement the task */
                          "websocket_updating",   /* Name of the task */
                          5000,                   /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          5,                      /* Priority of the task */
                          &websocket_task_handle, /* Task handle. */
                          1);                     /* Core where the task should run */

  xTaskCreatePinnedToCore(instruction_loop,       /* Function to implement the task */
                          "instruction_updating", /* Name of the task */
                          2000,                   /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          1,                      /* Priority of the task */
                          &instruction_handle,    /* Task handle. */
                          0);                     /* Core where the task should run */

#if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
  xTaskCreatePinnedToCore(state_feedback,         /* Function to implement the task */
                          "state_feedback",       /* Name of the task */
                          3000,                   /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          3,                      /* Priority of the task */
                          &state_feedback_handle, /* Task handle. */
                          0);                     /* Core where the task should run */
#endif

  xTaskCreatePinnedToCore(regular_update,         /* Function to implement the task */
                          "regular_update",       /* Name of the task */
                          3000,                   /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          2,                      /* Priority of the task */
                          &regular_task_handle,   /* Task handle. */
                          1);                     /* Core where the task should run */
  
  instruction_handler.init();

  // System start up complete
  indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::IMPULSE, INDICATOR_BLUE);
  indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, INDICATOR_BLUE);
  log_v("Thread init!");

#pragma endregion[task hook]


  // TODO: inited is not equal to ready (maybe not connected)
  if (WiFi.status() != WL_CONNECTED) 
    set_state(AGENT_STATE::LOST_CONN, WIFI_LOST);
  else 
    set_state(AGENT_STATE::INITED, NONE);

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

  #ifdef ENABLE_SERVER_IMU_ECHO
  log_i("\nNavigator state: %d, M-Instr: %6.2f, S-Instr:%6.2f, IMU-fps: %6.2f\nAid\t Cid\t State\t S-FPS\t M-Instr\t S-Instr \t Sensor",
    _state, instruction_handler.get_motor_fps(), instruction_handler.get_servo_fps(), sensor.imu_health.get_fps());
  #else 
  log_i("\nNavigator state: %d, M-Instr: %6.2f, S-Instr:%6.2f\nAid\t Cid\t State\t S-FPS\t M-Instr\t S-Instr \t Sensor",
    _state, instruction_handler.get_motor_fps(), instruction_handler.get_servo_fps());
  #endif
  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    // Query agent id by client id. 
    // If cid is an active client, aid is the agent id. Otherwise, aid is -1.
    const int aid = CommServer::query_agent_id(cid);
    if (aid == -1) break;

    const uint8_t aidx = get_aidx(aid);

    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    // Core module save agent data indexed by agent id (aid)
    Quaternion q = {0, 0, 0, 0};
    AGENT_STATE state = AGENT_STATE::LOST_CONN;
    float fps = 0.0f;
    if (xSemaphoreTake(_agents_mutex, 0) == pdTRUE) {
      memcpy((void*)&q, (void*)&(agents[aidx].packet.s.orientation), sizeof(Quaternion));
      state = agents[aidx].packet.state;
      fps = CommServer::state_health[aidx].get_fps();
      xSemaphoreGive(_agents_mutex);  
    }
    printf("[%d]\t %d\t %d\t %6.2f\t %6.2f, \t%6.2f, \t%6.2f, \t%6.2f\n", 
      aid, cid, state, fps, 
      q.q0, q.q1, q.q2, q.q3);
    #elif ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    float fps = CommServer::state_health[aidx].get_fps();
    printf("[%d]: %f\t", aid, fps);
    #endif
    
    // Move to next client id
    cid += 1;
  }
  printf("\n");
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
  printf("Free stack of instruction routine: %d\n", uxTaskGetStackHighWaterMark(instruction_handle));
  printf("Free stack of regular routine: %d\n", uxTaskGetStackHighWaterMark(regular_task_handle));
  #if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
  printf("Free stack of state feedback routine: %d\n", uxTaskGetStackHighWaterMark(state_feedback_handle));
  #endif

  // print heap memory usage
  printf("Free heap: %d\n", ESP.getFreeHeap());
}

void Core::print_instructions() {
  #ifdef SERVER
  log_i("\nAid\t eta_x\t eta_y\t motor1\t motor2\t armed: %s", _nav_armed ? "true" : "false");

  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    double eta_x=0.0f, eta_y=0.0f, omega_p1=0.0f, omega_p2=0.0f;
    if (xSemaphoreTake(_packet_mutex, 0) == pdTRUE) {
      eta_x = _packet.packets[i].eta_x;
      eta_y = _packet.packets[i].eta_y;
      omega_p1 = _packet.packets[i].omega_p1;
      omega_p2 = _packet.packets[i].omega_p2;
      xSemaphoreGive(_packet_mutex);
    }
    printf("[%d: %d]\t %6.2f\t %6.2f, \t%6.2f, \t%6.2f\n", 
      i, _state, eta_x, eta_y, omega_p1, omega_p2);
  }
  // TODO: This causes crashing
  #endif
}
