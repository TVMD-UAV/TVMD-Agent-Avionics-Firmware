#include "Core.h"

#ifdef SERVER
CommServer Core::comm(COMM_PORT);
#else
Core::DRONE_STATE Core::_state = STARTING;
CommClient Core::comm(COMM_PORT, server_IP);

ServoMotorDriver Core::x_servo(ServoMotorConfigs(MotorConfigs(18, 1100, 1900, 1500, 180, 4096), 0));
ServoMotorDriver Core::y_servo(ServoMotorConfigs(MotorConfigs(19, 1100, 1900, 1500, 180, 4096), 0));
ESCMotorDriver Core::esc_p1(ESCMotorConfigs(MotorConfigs(32, 1000, 2000, 1500, 100, 4096), 95));
ESCMotorDriver Core::esc_p2(ESCMotorConfigs(MotorConfigs(33, 1000, 2000, 1500, 100, 4096), 95));
#endif 


Core::Core(){}

void Core::init(){
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
  comm.set_ctrl_callback([](CtrlPacket &packet){
    // TODO: do something
  });

  comm.set_state_callback([](StatePacket &packet){
    // TODO: do something
    log_i("[Acc]%2.2f, %2.2f, %2.2f from %d\n", 
      packet.acc.x, packet.acc.y, packet.acc.z, packet.agent_id);
  });

  comm.set_instruct_callback([](InstructPacket &packet){

  });
#else
  /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](CtrlPacket &packet){
    Core::x_servo.write(packet.eta_x);
    Core::y_servo.write(packet.eta_y);
    Core::esc_p1.write(packet.omega_p1);
    Core::esc_p2.write(packet.omega_p2);
  });

  comm.set_state_callback([](StatePacket &packet){
    // TODO: do something
  });

  comm.set_instruct_callback([](InstructPacket &packet){

  });
#endif 

  comm.init();

  //TODO: check stack usages
  xTaskCreatePinnedToCore(websocket_loop, /* Function to implement the task */
                          "websocket_updating",   /* Name of the task */
                          15000,                  /* Stack size in words */
                          NULL,                   /* Task input parameter */
                          1,                      /* Priority of the task */
                          &websocket_task_handle, /* Task handle. */
                          1); /* Core where the task should run */
  
#ifndef SERVER
  xTaskCreatePinnedToCore(state_feedback,   /* Function to implement the task
                                             */
                          "state_feedback", /* Name of the task */
                          5000,             /* Stack size in words */
                          NULL,             /* Task input parameter */
                          10,               /* Priority of the task */
                          &state_feedback_handle, /* Task handle. */
                          1); /* Core where the task should run */
#endif
  _state = INITED;
}

void Core::Wifi_connection_setup() {
  const IPAddress local_IP(gateway[0], gateway[1], gateway[2], _agent_id + 1);
#ifdef SERVER
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(server_IP, gateway, subnet);
  const int maximum_connection = 4;
  WiFi.softAP(WIFI_SSID, WIFI_PSWD, 1, false, maximum_connection);
  log_i("AP started with IP address: %s\n", WiFi.softAPIP().toString());
#else
  log_i("Connecting to %s\n", (String)WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  log_i("\nWiFi connected with IP address:  %s\n", WiFi.softAPIP().toString());
#endif
}

void Core::set_armed(bool armed){
  bool target_state = armed;
  if (armed){
    // Trying to set arm
    if (_state == INITED){
      _state = ARMED;
      log_i("Succeed to set arm\n");
    }
    else{
      target_state = false;
      log_e("Trying to set arm with state %d\n", _state);
    }
  }
  else{
    _state = INITED;
  }
  Core::x_servo.set_armed(target_state);
  Core::y_servo.set_armed(target_state);
  Core::esc_p1.set_armed(target_state);
  Core::esc_p2.set_armed(target_state);
}

int Core::get_agent_id(){
  static bool agent_id_init = false;
  if (!agent_id_init){
    // not read from memory yet
    EEPROM.begin(EEPROM_SIZE);
#ifdef WRITE_AGENT_ID
    EEPROM.write(EEPROM_AGENT_ID_ADDR, AGENT_ID);
    EEPROM.commit();
#endif
    _agent_id = EEPROM.read(EEPROM_AGENT_ID_ADDR);
    log_i("Agent id: %d, \n", _agent_id);
  }
  return _agent_id;
}

#ifdef SERVER
#include "esp_wifi.h"
void display_connected_devices() {
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

  if (adapter_sta_list.num > 0)
    Serial.println("-----------");
  for (uint8_t i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    Serial.print((String) "[+] Device " + i + " | MAC : ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", station.mac[0],
                  station.mac[1], station.mac[2], station.mac[3],
                  station.mac[4], station.mac[5]);
    // Serial.println((String) " | IP " + ip4addr_ntoa(&(station.ip)));

    Serial.printf("   IP: %s\n", IPAddress(station.ip.addr).toString());
  }
}
#endif
