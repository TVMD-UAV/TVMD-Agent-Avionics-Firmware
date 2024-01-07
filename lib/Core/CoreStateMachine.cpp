#include "Core.h"

bool Core::set_armed(bool armed) {
#ifdef SERVER
  InstructPacket arming;
  arming.time = millis();

  if (armed) {
    if (_state == AGENT_STATE::ARMED)
      return true;
    else if (_state == AGENT_STATE::ARMING) {
      // Waiting on all agents set
      if (check_all_agent_alive(AGENT_STATE::ARMED)) {
        indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, INDICATOR_RED);
        set_state(AGENT_STATE::ARMED);
        return true;
      } else {
        // Resend arming request
        arming.instruction = InstructPacket::INSTRUCT_TYPE::ARMING;
        comm.send(&arming, sizeof(arming));
        // set_state(AGENT_STATE::ARMED);
        return false;
      }
    }
    else {
      // INITED, LOST_CONN
      // Other states, check before arming
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
  const AGENT_STATE target = (armed ? AGENT_STATE::ARMED : AGENT_STATE::INITED);
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

#ifdef SERVER
bool Core::check_all_agent_alive(AGENT_STATE target) {
  bool all_agent_available = true;
  
  if (xSemaphoreTake(_agents_mutex, 0) == pdTRUE) {
    for (int i = 0; i < MAX_NUM_AGENTS; i++) {
      all_agent_available &= (agents[i].packet.state == target);
    }
    xSemaphoreGive(_agents_mutex);
    return all_agent_available;
  } else {
    return false;
  }
}
#endif
