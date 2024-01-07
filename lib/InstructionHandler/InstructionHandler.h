#ifndef INSTRUCTION_HANDLER_H
#define INSTRUCTION_HANDLER_H

#include "configs.h"
#include <functional>
#include <Wire.h>
// #include <esp_err.h>
// #include <esp32-hal-i2c.h>
// #include <esp32-hal-i2c-slave.h>
#include "benchmark.h"
#include "freertos/semphr.h"

union Instruction {
    enum ControlTypes {UNKNOWN=0, MOTORS, SERVOS};
    struct {
        uint8_t armed;
        uint8_t type;
        float control[8];
    } data;
    uint8_t raw[sizeof(data)];
};

typedef std::function<void(const Instruction &)> InstructionCallbackFunc;

class InstructionHandler {
public:
    InstructionHandler() {
        _instr_mutex = xSemaphoreCreateMutex();
    };

    void init();

    void set_instruction_callback(InstructionCallbackFunc func) {
        _instruct_callback = func;
    };

    double get_motor_fps() { return _motors_instr_health.get_fps(); };

    double get_servo_fps() { return _servos_instr_health.get_fps(); };

    void update();

private:
    static uint8_t _recv_trigger;
    static Instruction _instruction;
    static SemaphoreHandle_t _instr_mutex;

    static InstructionCallbackFunc _instruct_callback;

    static void onReceive(int numBytes);

    static void onRequest();

    static Benchmark _motors_instr_health;
    static Benchmark _servos_instr_health;
};

#endif