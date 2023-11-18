#include <Indicator.h>

const uint8_t Indicator::LED_BIT_MAP[LED_STATE_NUM] = {
    0b00000000, // TURN_OFF
    0b11111111, // FLASHING
    0b10000000, // IMPULSE
    0b10100000, // DOUBLE_IMPULSE
    0b10001000, // SHORT
    0b11000000, // PEACE
    0b11001100, // FAST
    0b11110000, // LONG
    0b01010101, // HASTILY
};

void Indicator::update() {
    static time_t last_update_time = millis();

    if (millis() - last_update_time > UPDATE_INTERVAL) {
        for (int i=0; i<LED_COUNT; ++i) {
            uint32_t color = 0;

            if (xSemaphoreTake(_state_mutex, portMAX_DELAY) == pdTRUE) {
                if (_state[i] < LED_STATE_NUM){
                    if (LED_BIT_MAP[_state[i]] & (1 << _phase[i])) {
                        color = _color[i];
                    }
                }
                else {
                    switch (_state[i])
                    {
                    case SINE_WAVE: {
                        const uint8_t brightness = pixels.sine8((uint8_t)(millis() / 4)); // period ~ 4s
                        color = ((_color[i] & INDICATOR_RED)   * brightness) & INDICATOR_RED
                            | ((_color[i] & INDICATOR_GREEN) * brightness) & INDICATOR_GREEN
                            | ((_color[i] & INDICATOR_BLUE)  * brightness) & INDICATOR_BLUE;
                    } break;
                        
                    case COLOR_SINE_WAVE: {
                        const uint16_t hue = (uint16_t)(millis() * 16); // period ~ 4s
                        color = pixels.ColorHSV(hue, 255, 255);
                    } break;
                    
                    default:
                        // turn off led
                        break;
                    }
                }
                _phase[i] = (_phase[i] + 1) % 8;
                xSemaphoreGive(_state_mutex);
            }

            pixels.setPixelColor(i, color);
            last_update_time = millis();
        }
        pixels.show();
    }
};
