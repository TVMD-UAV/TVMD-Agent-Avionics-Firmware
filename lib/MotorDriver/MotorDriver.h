#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#ifdef ESP32
#include "configs.h"

#ifdef HAS_MOTOR
#include "driver/ledc.h"
#include "esp32-hal-ledc.h"

#define MOTOR_PWM_FREQ (400)
#define MOTOR_PWM_DUTY_RES LEDC_TIMER_13_BIT
#define MOTOR_PWM_DUTY_US (1000000 / MOTOR_PWM_FREQ)  

#define CONSTRAIN_VALUE(X, MIN, MAX)                                           \
  (((X) > (MAX)) ? (MAX) : (((X) < (MIN)) ? (MIN) : (X)))

struct MotorConfigs {
public:
  MotorConfigs(const uint16_t _pin, const uint16_t _min, const uint16_t _max,
               const uint16_t _mid, const uint16_t _vrange, const uint16_t _den, const uint8_t _channel)
      : pin(_pin), pmin(_min), pmax(_max), pmid(_mid), vrange(_vrange),
        denominator(_den), channel(_channel){};

  virtual ~MotorConfigs() = default;
  const uint16_t pin;
  const uint16_t pmin;
  const uint16_t pmax;
  const uint16_t pmid;
  const uint16_t vrange;
  const uint16_t denominator;
  const uint8_t channel;
};

struct ServoMotorConfigs : public MotorConfigs {
public:
  ServoMotorConfigs(const MotorConfigs _m, const uint16_t _gear)
      : MotorConfigs(_m), gear_ratio(_gear){};

  const uint16_t gear_ratio;
};

struct ESCMotorConfigs : public MotorConfigs {
public:
  ESCMotorConfigs(const MotorConfigs _m, const uint16_t _max_throttle)
      : MotorConfigs(_m), max_throttle(_max_throttle){};

  const uint16_t max_throttle;
};

/*
 * Providing high level closed-loop control for servo objects
 */
class MotorDriver {
public:
  MotorDriver(const MotorConfigs);

  virtual void init();

  virtual void write(const float) = 0;

  virtual void raw_write(const uint16_t);

  void set_armed(bool _arm) {
    if (!_calibrating) armed = _arm;
  };

protected:
  bool armed{false};

  // This flag is used to indicate whether the motor is in calibration mode.
  // If it is, the output will be enabled regardless of the armed flag.
  // And it will not be able to be armed.
  bool _calibrating{false};

  MotorConfigs _config;

  uint16_t _idle_value{0};

  uint32_t inline usToTicks(uint32_t usec) {
    // if usec < 2^14 and the maximum resolution is 2^13, then the result will
    // be smaller than 2^27. So it is safe to calculate using uint32_t.
    return (((uint32_t)usec * (uint32_t)(1 << MOTOR_PWM_DUTY_RES) ) / MOTOR_PWM_DUTY_US);
  };
};

/*
 * Driver for Servos
 */
class ServoMotorDriver : public MotorDriver {
public:
  ServoMotorDriver(const ServoMotorConfigs config);

  virtual void write(const float angle);

protected:
  // ServoMotorConfigs _config;
  uint16_t gear_ratio;
};

/*
 * Driver for ESC
 */
class ESCMotorDriver : public MotorDriver {
public:
  ESCMotorDriver(const ESCMotorConfigs config);

  void write(const float throttle);

  void calibrate(int stage);

protected:
  // ESCMotorConfigs _config;
  uint16_t max_throttle;
};

#endif // HAS MOTOR
#endif // defined ESP32
#endif
