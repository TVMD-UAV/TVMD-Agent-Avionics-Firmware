#ifndef INDICATOR_H
#define INDICATOR_H

#include "configs.h"
#include <Adafruit_NeoPixel.h>
#include "freertos/semphr.h"

#define INDICATOR_RED 0x00FF0000
#define INDICATOR_GREEN 0x0000FF00
#define INDICATOR_BLUE 0x0000FF

class Indicator {
public:
    enum LED_STATE {
        TURN_OFF = 0,
        FLASHING,
        IMPULSE,
        DOUBLE_IMPULSE,
        SHORT,
        PEACE,
        FAST,
        LONG,
        HASTILY,
        LED_STATE_NUM,
        SINE_WAVE,
        COLOR_SINE_WAVE,
    };

    enum LED_ID {
        COMM = 0b00000001,
        DATA = 0b00000010,
        BOTH = 0b00000011,
    };

    static const uint8_t LED_BIT_MAP[LED_STATE::LED_STATE_NUM];
    static const uint8_t BIT_TIME = 100; // ms
    static const uint8_t UPDATE_INTERVAL = 50; // ms

    Indicator(): pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800) {
        _state_mutex = xSemaphoreCreateMutex();
        pixels.begin();
        pixels.clear();
        pixels.setBrightness(128);
    };

    void set_color(LED_ID idx, uint8_t r, uint8_t g, uint8_t b) {
        pixels.setPixelColor(idx, pixels.Color(r, g, b));
        pixels.show();
    };

    void set_led_state(LED_ID led_id, LED_STATE state, uint32_t color) {
        for (uint8_t i=0; (i < LED_COUNT) && (i < LED_ID::BOTH-1); ++i) {
            if ((led_id >> i) & 1 ) { 
                if (xSemaphoreTake(_state_mutex, portMAX_DELAY) == pdTRUE) {
                    _state[i] = state;
                    _phase[i] = 0;
                    _color[i] = color;
                    xSemaphoreGive(_state_mutex);
                }
            }
        }
    };

    void update();

protected:
    SemaphoreHandle_t _state_mutex = NULL;
    Adafruit_NeoPixel pixels;
    LED_STATE _state[LED_COUNT];

    // Recording the step of each LED (the maximum must be 8, i.e., uint8_t)
    uint8_t _phase[LED_COUNT];

    uint32_t _color[LED_COUNT];

private:
};

#endif
