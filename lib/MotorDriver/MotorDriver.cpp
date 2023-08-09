#include "MotorDriver.h"

#ifdef HAS_MOTOR

MotorDriver::MotorDriver() : armed(false){};

void MotorDriver::set_armed(bool _arm) {
  armed = _arm;
  if (!armed)
    init();
};

ServoMotorDriver::ServoMotorDriver(const ServoMotorConfigs config)
    : _config(config) {}

void ServoMotorDriver::init() {
  // TODO: config check is required

  Servo::attach(_config.pin, _config.pmin, _config.pmax);

  // set angles to middle
  Servo::write((_config.pmin + _config.pmax) / 2);
}

/** Drive the servo to the target angle
 * @param angle the target angle in degree
 */
void ServoMotorDriver::write(const float angle) {
  if (armed) {
    // unit mapping
    const int32_t value =
        angle * (_config.pmax - _config.pmin) / _config.vrange + _config.pmid;

    // range constrain
    const uint16_t v = CONSTRAIN_VALUE(value, _config.pmin, _config.pmax);

    Servo::write(v);
    log_i("Servo: %d\n", v);
  }
};

ESCMotorDriver::ESCMotorDriver(const ESCMotorConfigs config)
    : _config(config) {}

void ESCMotorDriver::init() {
  Servo::attach(_config.pin, _config.pmin, _config.pmax);

  // set angles to middle
  Servo::write(_config.pmin);
}

/** Drive the esc (the propeller) to the target throttle
 * @param throttle the target throttle (in percentage, 0~100)
 */
void ESCMotorDriver::write(const float throttle) {
  if (armed) {
    // unit mapping
    const int32_t value =
        throttle * (_config.pmax - _config.pmin) / _config.vrange +
        _config.pmin;

    // range constrain
    const uint16_t v = CONSTRAIN_VALUE(value, _config.pmin, _config.pmax);

    Servo::write(v);
  }
};
#endif // HAS MOTOR
