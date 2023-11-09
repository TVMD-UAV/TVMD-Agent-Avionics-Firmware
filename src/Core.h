#ifndef CORE_H
#define CORE_H

#include "CommProtocol.h"
#include "configs.h"
#include <Arduino.h>

#include <EEPROM.h>

#ifdef SERVER

#else
#include "SensorDriver.h"
#include <MotorDriver.h>
#endif

#include "Indicator.h"


class Core {
public:
  static Indicator indicator;
  
#ifdef SERVER
  static CommServer comm;
#else
  static CommClient comm;
#endif

#ifndef SERVER
  static Sensors sensor;
  static ServoMotorDriver x_servo;
  static ServoMotorDriver y_servo;
  static ESCMotorDriver esc_p1;
  static ESCMotorDriver esc_p2;
#endif

  static void init();

  static bool set_armed(bool armed);

  static void send_ctrl(const CtrlPacketArray *const packet);

  static void print_summary();

  static bool set_state(AGENT_STATE target);

  static AGENT_STATE get_current_state() { return _state; };

protected:
  static uint8_t _agent_id;

  static AGENT_STATE _state;

#ifdef SERVER
  // agent data
  static AgentData agents[MAX_NUM_AGENTS];
#else
  // sensor data
  static StatePacket packet;
  static CtrlPacket _ctrl_packet;
#endif

  // for initialization
  static void Wifi_connection_setup();

  // Loading agent id from EEPROM
  static int get_agent_id();

#ifdef SERVER
  static bool check_all_agent_alive(AGENT_STATE target = AGENT_STATE::INITED);
#endif

private:
  static TaskHandle_t websocket_task_handle;
  static void websocket_loop(void *parameter);

#ifndef SERVER
  static TaskHandle_t state_feedback_handle;
  static void state_feedback(void *parameter);
#endif

  static TaskHandle_t indicator_task_handle;
  static void indicator_update(void *parameter){
    for (;;) {
      indicator.update();
      yield();
    }
  };
};

#endif