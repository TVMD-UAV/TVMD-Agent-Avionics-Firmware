#include "Core.h"

#ifdef SERVER
void Core::comm_callback_setup() {
    /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](const CtrlPacket &packet) {
    // TODO: do something
  });

  comm.set_state_callback([](const StatePacket &packet) {
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

  comm.set_instruct_callback([](const InstructPacket &packet) {
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
}

/**
 * This callback function is called when instructions are received from the external 
 * I2C bus. It directly set commands to the actuators.
 */
void Core::instruction_callback_setup() {   
  instruction_handler.set_instruction_callback([](const Instruction &instruction) {
    // log_i("Receiving instructions");
    // TODO: limit maximum update interval
    // set_armed(instruction.data.armed);

    // log_i("Instruction: %6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t%6.2f\t",
    //   instruction.data.control[0], instruction.data.control[1], instruction.data.control[2], instruction.data.control[3],
    //   instruction.data.control[4], instruction.data.control[5], instruction.data.control[6], instruction.data.control[7]);
    if (xSemaphoreTake(_packet_mutex, portMAX_DELAY) == pdTRUE) {
      _packet.id += 1;
      _packet.time = micros();
      _armed = instruction.data.armed;

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
      }
      xSemaphoreGive(_packet_mutex);

      if (instruction.data.type == Instruction::ControlTypes::MOTORS )
        _packet_ready = true;
    }
  });
}

#else 

void Core::comm_callback_setup() {
    /* Setup ctrl callback functions */
  comm.set_ctrl_callback([](const CtrlPacket &packet) {
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

  comm.set_state_callback([](const StatePacket &packet) {
    // TODO: do something
  });

  comm.set_instruct_callback([](const InstructPacket &packet) {
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
}

void Core::instruction_callback_setup() {};

#endif

void Core::indicator_update(void *parameter){
    for (;;) {
        indicator.update();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
};
