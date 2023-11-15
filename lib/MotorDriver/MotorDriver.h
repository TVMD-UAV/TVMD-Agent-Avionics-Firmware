#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#ifdef ESP32
#include "configs.h"

#ifdef HAS_MOTOR
#include <ESP32Servo.h>

#define CONSTRAIN_VALUE(X, MIN, MAX)                                           \
  (((X) > (MAX)) ? (MAX) : (((X) < (MIN)) ? (MIN) : (X)))

class MotorConfigs {
public:
  MotorConfigs(const uint16_t _pin, const uint16_t _min, const uint16_t _max,
               const uint16_t _mid, const uint16_t _vrange, const uint16_t _den)
      : pin(_pin), pmin(_min), pmax(_max), pmid(_mid), vrange(_vrange),
        denominator(_den){};

  virtual ~MotorConfigs() = default;
  const uint16_t pin;
  const uint16_t pmin;
  const uint16_t pmax;
  const uint16_t pmid;
  const uint16_t vrange;
  const uint16_t denominator;
};

class ServoMotorConfigs : public MotorConfigs {
public:
  ServoMotorConfigs(const MotorConfigs _m, const uint16_t _gear)
      : MotorConfigs(_m), gear_ratio(_gear){};

  const uint16_t gear_ratio;
};

class ESCMotorConfigs : public MotorConfigs {
public:
  ESCMotorConfigs(const MotorConfigs _m, const uint16_t _max_throttle)
      : MotorConfigs(_m), max_throttle(_max_throttle){};

  const uint16_t max_throttle;
};

/*
 * Providing high level closed-loop control for servo objects
 */
class MotorDriver : protected Servo {
public:
  MotorDriver(const MotorConfigs);

  virtual void init();

  virtual void write(const float) = 0;

  virtual void raw_write(const uint16_t);

  void set_armed(bool _arm) {armed = _arm;};

  //   virtual void calibration();

protected:
  bool armed{false};

  MotorConfigs _config;

  uint16_t _idle_value{0};
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

protected:
  // ESCMotorConfigs _config;
  uint16_t max_throttle;
};

#endif // HAS MOTOR
#endif // defined ESP32
#endif
