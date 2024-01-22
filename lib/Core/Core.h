#ifndef CORE_H
#define CORE_H

#include "CommProtocol.h"
#include "configs.h"
#include <Arduino.h>

#include <EEPROM.h>
#include "freertos/semphr.h"

#define COMM_SETUP
#ifdef SERVER
#ifdef ENABLE_SERVER_IMU_ECHO
#include "ImuEchoHandler.h"
#include "SensorDriver.h"
#endif 
#else
#include "SensorDriver.h"
#include <MotorDriver.h>
#endif

#include "Indicator.h"
#include "InstructionHandler.h"

class Core {
public:
    static Indicator indicator;

#ifdef SERVER
    static CommServer comm;
#else
    static CommClient comm;
#endif

    static InstructionHandler instruction_handler;

#ifdef SERVER
    static void send_ctrl(const CtrlPacketArray *const packet);
    
#ifdef ENABLE_SERVER_IMU_ECHO
    static Sensors sensor;
    static ImuEchoHandler imu_echo;
#endif

#else
    static Sensors sensor;
    static ServoMotorDriver x_servo;
    static ServoMotorDriver y_servo;
    static ESCMotorDriver esc_p1;
    static ESCMotorDriver esc_p2;
#endif

    static void init();

    static bool set_armed(bool armed);

    static bool set_state(AGENT_STATE target);

    static AGENT_STATE get_current_state() { return _state; };
    
    static void print_summary();

    static void print_instructions();

protected:
#ifdef SERVER
    // A server is responsible for listening control commands from the 
    // navigator and transport them to all agents.
    static SemaphoreHandle_t _packet_mutex;
    static SemaphoreHandle_t _packet_semphr;
    static CtrlPacketArray _packet;
    static bool _packet_ready;

    // The armed state from the navigator
    static bool _nav_armed;
    
    // Agents' feedbacks including states and sensor data
    static SemaphoreHandle_t _agents_mutex;
    static volatile AgentData agents[MAX_NUM_AGENTS];

    #ifdef ENABLE_SERVER_IMU_ECHO
    static SemaphoreHandle_t _state_mutex;
    static SemaphoreHandle_t _state_semphr;
    #endif
#else
    // The state packet to the navigator (maintained by the sensor callback).
    // When the sensors are ready, the state packet is updated and sent to the
    // navigator.
    static SemaphoreHandle_t _state_mutex;
    static SemaphoreHandle_t _state_semphr;
    static StatePacket _state_packet;

    // The control packet from the navigator.
    static SemaphoreHandle_t _ctrl_mutex;
    static CtrlPacket _ctrl_packet;
#endif

    // The agent id of this agent, a server's id must be 0.
    static uint8_t _agent_id;

    static volatile AGENT_STATE _state;
    static volatile AGENT_STATE _last_state;
    
    // Loading agent id from EEPROM
    static int get_agent_id();

    // Perform range check on agent id and shift an agent id to an array index
    static uint8_t get_aidx(uint8_t agent_id) {
        assert(agent_id > 0 && agent_id <= MAX_NUM_AGENTS);
        return agent_id - 1;
    };

    static bool check_comm_alive();

private:
    // Wifi initialization
    static void Wifi_connection_setup();

    // Callback functions setup
    static void instruction_callback_setup();
    static void comm_callback_setup();

    static TaskHandle_t instruction_handle;
    static void instruction_loop(void *parameter);

    // Routines for websocket services
    static TaskHandle_t websocket_task_handle;
    static void websocket_loop(void *parameter);

#if !defined(SERVER) || defined(ENABLE_SERVER_IMU_ECHO)
    // Routines for state feedback (from an agent to a navigator)
    static TaskHandle_t state_feedback_handle;
    static void state_feedback(void *parameter);
#endif

    static TaskHandle_t indicator_task_handle;
    static void indicator_update(void *parameter);

    static TaskHandle_t regular_task_handle;
    static void regular_update(void *parameter);

    static void state_machine_update();
};

#endif
