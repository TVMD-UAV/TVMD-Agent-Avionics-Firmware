#include "MotorDriver.h"

#ifdef HAS_MOTOR

MotorDriver::MotorDriver(const MotorConfigs config) : 
  _config(config){};

void MotorDriver::init() {
  // TODO: config check is required

  Servo::attach(_config.pin, _config.pmin, _config.pmax);

  // set angles to middle
  Servo::write(_idle_value);
}

void MotorDriver::raw_write(const uint16_t value) {
  if (armed) {
    // range constrain
    const uint16_t w = CONSTRAIN_VALUE(value, _config.pmin, _config.pmax);

    Servo::write(w);
  }
};


ServoMotorDriver::ServoMotorDriver(const ServoMotorConfigs config): 
  MotorDriver(config) 
{
  gear_ratio = config.gear_ratio;
  _idle_value = (_config.pmin + _config.pmax) / 2;
}

/** Drive the servo to the target angle
 * @param angle the target angle in degree
 */
void ServoMotorDriver::write(const float angle) {
  // unit mapping
  const int32_t value =
      angle * (_config.pmax - _config.pmin) / _config.vrange + _config.pmid;

  raw_write(value);
};


ESCMotorDriver::ESCMotorDriver(const ESCMotorConfigs config): 
  MotorDriver(config) 
{
  max_throttle = config.max_throttle;
  _idle_value = _config.pmin;
}

/** Drive the esc (the propeller) to the target throttle
 * @param throttle the target throttle (in percentage, 0~100)
 */
void ESCMotorDriver::write(const float throttle) {
  // unit mapping
  const int32_t value =
      throttle * (_config.pmax - _config.pmin) / _config.vrange + _config.pmin;

  raw_write(value);
};
#endif // HAS MOTOR
