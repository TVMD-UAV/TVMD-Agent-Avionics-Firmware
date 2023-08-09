#ifndef CORE_H
#define CORE_H

#include "CommProtocol.h"
#include "configs.h"
#include <Arduino.h>

#include <EEPROM.h>

#ifdef SERVER
CtrlPacket packet;
#else
#include "SensorDriver.h"
#include <MotorDriver.h>
#endif

#endif 

class Core{
public:
  enum DRONE_STATE {STARTING=0, INITED, LOST_CONN, ARMED};

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

  Core();

  static void init();

  static void Wifi_connection_setup();

  static void set_armed(bool armed);

  static int get_agent_id();

private:
  static DRONE_STATE _state;
  static uint8_t _agent_id;

  static TaskHandle_t websocket_task_handle;
  static void websocket_loop(void *parameter) {
    while (true) {
      comm.update();
      // To allow other threads have chances to join
      delay(1);
    }
  }

#ifndef SERVER
  static TaskHandle_t state_feedback_handle;
  static void state_feedback(void *parameter) {
    static unsigned long last_summary_time = 0;
    while (true) {
      if (millis() - last_summary_time >= 500) {
        const StatePacket *s_packet_ptr = sensor.state_packet_gen(_state);
        if (comm.send(CommProtocol::PACKET_TYPE::STATE_AGN, s_packet_ptr))
          log_i("State feedback sent!");
        last_summary_time = millis();
      }
      delay(10);
    }
  }
#endif
};
