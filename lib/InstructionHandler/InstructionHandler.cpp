#include "InstructionHandler.h"

InstructionCallbackFunc InstructionHandler::_instruct_callback = NULL;
Benchmark InstructionHandler::_motors_instr_health;
Benchmark InstructionHandler::_servos_instr_health;

void InstructionHandler::init() {
    assert(_instruct_callback != NULL);

    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    // uint8_t slaveAddr, int sda, int scl, uint32_t frequency
    Wire.begin(INSTR_I2C_SLAVE_ADDR, EXT_I2C_SDA_PIN, EXT_I2C_SCL_PIN, INSTR_I2C_CLOCK_FREQ);

    _motors_instr_health = Benchmark(NUM_HEALTH_CHECK);
    _servos_instr_health = Benchmark(NUM_HEALTH_CHECK);

    // if (i2cInit(0, EXT_I2C_SDA_PIN, EXT_I2C_SCL_PIN, INSTR_I2C_CLOCK_FREQ) != ESP_OK) {
        
    //     Serial.println("I2C init failed");
    //     return;
    // }
}

void InstructionHandler::onReceive(int numBytes) {
    // log_i("Receiving %d bytes", numBytes);
    switch (numBytes) {
        case sizeof(Instruction): {
            Instruction instruction;
            Wire.readBytes(instruction.raw, numBytes);
            if (instruction.data.type == Instruction::ControlTypes::MOTORS) {
                _motors_instr_health.feed_data(micros());
            } else if (instruction.data.type == Instruction::ControlTypes::SERVOS) {
                _servos_instr_health.feed_data(micros());
            } else {
                log_e("Received invalid instruction type: %d", instruction.data.type);
                return;
            }
            // log_i("Received instruction: %d, %d", instruction.data.armed, instruction.data.type);
            _instruct_callback(instruction);
        } break;

        default: {
            log_e("Received invalid instruction size: %d", numBytes);
            // clear the buffer
            while (Wire.available()) {
                Wire.read();
            }
        } break;
    }
}

void InstructionHandler::onRequest() {
    log_i("Requesting");
}