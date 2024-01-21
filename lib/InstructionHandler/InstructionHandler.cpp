#include "InstructionHandler.h"

InstructionCallbackFunc InstructionHandler::_instruct_callback = NULL;
Perf InstructionHandler::_motors_instr_health;
Perf InstructionHandler::_servos_instr_health;

uint8_t InstructionHandler::_recv_trigger;
SemaphoreHandle_t InstructionHandler::_instr_mutex{nullptr};
Instruction InstructionHandler::_instruction;

void InstructionHandler::init() {
    assert(_instruct_callback != NULL);

    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Wire.setBufferSize(INSTR_I2C_BUFFER_SIZE);

    // uint8_t slaveAddr, int sda, int scl, uint32_t frequency
    Wire.begin(INSTR_I2C_SLAVE_ADDR, EXT_I2C_SDA_PIN, EXT_I2C_SCL_PIN, INSTR_I2C_CLOCK_FREQ);

    _motors_instr_health = Perf(NUM_HEALTH_CHECK);
    _servos_instr_health = Perf(NUM_HEALTH_CHECK);
}

void InstructionHandler::onReceive(int numBytes) {
    // log_i("Receiving %d bytes", numBytes);
    switch (numBytes) {
        case sizeof(Instruction): {
            if (xSemaphoreTake(_instr_mutex, 0) == pdTRUE) {
                Wire.readBytes(_instruction.raw, numBytes);
                _recv_trigger = true;
                xSemaphoreGive(_instr_mutex);
                
                if (_instruction.data.type == Instruction::ControlTypes::MOTORS) {
                    _motors_instr_health.feed_data(micros());
                } else if (_instruction.data.type == Instruction::ControlTypes::SERVOS) {
                    _servos_instr_health.feed_data(micros());
                } else {
                    // log_e("Received invalid instruction type: %d", instruction.data.type);
                    Wire.flush();
                    return;
                }
            }
        } break;

        default: {
            // log_e("Received invalid instruction size: %d", numBytes);
            // clear the buffer
            // Instruction instruction;
            // Wire.readBytes(instruction.raw, numBytes);
            Wire.flush();
        } break;
    }
    // Wire.flush();
}

void InstructionHandler::onRequest() {
    // log_i("Requesting");
}

void InstructionHandler::update() { 
    if (_recv_trigger) {
        // pulling
        if (xSemaphoreTake(_instr_mutex, 0) == pdTRUE) {
            
            // log_i("Received instruction: %d, %d", instruction.data.armed, instruction.data.type);
            _instruct_callback(_instruction);    
            _recv_trigger = false;
            xSemaphoreGive(_instr_mutex);
        }
    }
}