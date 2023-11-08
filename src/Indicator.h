#ifndef INDICATOR_H
#define INDICATOR_H

#include "configs.h"
#include <Adafruit_NeoPixel.h>

class Indicator {
public:
    Indicator(): pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800) {
        pixels.begin();
    };

    void set_color(int idx, uint8_t r, uint8_t g, uint8_t b) {
        // for (int i = 0; i < LED_COUNT; i++) {
        // pixels.setPixelColor(idx, r, g, b);
        pixels.setPixelColor(idx, pixels.Color(r, g, b));
        // }
        pixels.show();
    };

protected:
    Adafruit_NeoPixel pixels;
private:
};


#endif
