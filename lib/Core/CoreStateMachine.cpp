#include "Core.h"

#ifdef SERVER
bool Core::set_armed(bool armed) { 
  if (armed) {
    // Already armed
    if (_state == AGENT_STATE::ARMED)
      return true;

    // Try to arm
    if (_nav_armed && (_state == AGENT_STATE::INITED)) {
      return set_state(AGENT_STATE::ARMING, CMD_TRY_ARMING);
    }
  } else {
    // force disarming
    if (_state != AGENT_STATE::LOST_CONN) {
      return set_state(AGENT_STATE::INITED, CMD_DISARMING);
    }
  }
  return false;
}
#else 
bool Core::set_armed(bool armed) { 
  Core::esc_p1.set_armed(armed);
  Core::esc_p2.set_armed(armed);
  Core::x_servo.set_armed(armed);
  Core::y_servo.set_armed(armed);
  return true;
}
#endif

/**
 * set the state to the target state according to finite-state machine rule
 * @param target the target state
 * @return true if ok
 */
bool Core::set_state(AGENT_STATE target, StateChangeReason reason) {
  if (target == _state)
    return false;
  
  switch(target) {
    case AGENT_STATE::INITED:
      indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::IMPULSE, INDICATOR_BLUE);
      indicator.set_led_state(Indicator::LED_ID::DATA, Indicator::SINE_WAVE, INDICATOR_BLUE);
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
      indicator.set_led_state(Indicator::LED_ID::BOTH, Indicator::IMPULSE, 0x00FF0000);
      break;

    case AGENT_STATE::LOST_CONN:
      indicator.set_led_state(Indicator::LED_ID::COMM, Indicator::HASTILY, 0x00FF00FF);
      break;

    default:
      return false;
  }
  log_i("[STATE] set to %d from %d due to %d", target, _state, reason);
  _state = target;
  return true;
}

bool Core::check_comm_alive() {
  bool any_agent_timeout = false;
#ifdef SERVER
  // check all agents are alive
  for (int i = 0; i < MAX_NUM_AGENTS; i++) {
    any_agent_timeout |= CommServer::state_health[i].is_timeout();
  }
#else
  any_agent_timeout |= CommClient::ctrl_health.is_timeout();
#endif
  return !any_agent_timeout;
}

void Core::state_machine_update() {
  const bool comm_alive = check_comm_alive();
  if (_last_state != _state) {
    #ifdef SERVER
    // if state is arming, try to arm
    if ((_state == AGENT_STATE::ARMING) && comm_alive) {
      set_state(AGENT_STATE::ARMED, ARMING_PASSED);
    }
    #else 
    bool armed = _state == AGENT_STATE::ARMED;
    set_armed(armed);
    #endif

    _last_state = _state;
  }

  // if any agent timeout, set state to LOST_CONN
  if (!comm_alive) {
    set_state(AGENT_STATE::LOST_CONN, LOST_CONN);
  } else {
    if (_state == AGENT_STATE::LOST_CONN) {
      set_state(AGENT_STATE::INITED, CONN_RELIVED);
    }
  }
}