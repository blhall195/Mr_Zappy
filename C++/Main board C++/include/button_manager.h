#pragma once

#include <Arduino.h>
#include "config.h"

constexpr uint8_t NUM_BUTTONS = 5;

// Button indices — match Python's "Button 1" through "Fire Button"
enum class Button : uint8_t {
    MEASURE  = 0,   // Button 1 — A3
    DISCO    = 1,   // Button 2 — A4
    CALIB    = 2,   // Button 3 — A0
    SHUTDOWN = 3,   // Button 4 — A1
    FIRE     = 4,   // Fire Button — D4
};

class ButtonManager {
public:
    void begin();
    void update();

    bool isPressed(Button btn) const;
    bool wasPressed(Button btn) const;
    bool wasReleased(Button btn) const;

    static const char* name(Button btn);

private:
    struct State {
        uint8_t  pin;
        bool     debounced;       // true = pressed (active-low inverted)
        bool     previous;        // debounced state from last update()
        bool     raw;             // last raw reading (true = pressed)
        uint32_t lastChangeTime;  // millis() when raw last changed
        bool     fell;            // edge: just pressed this tick
        bool     rose;            // edge: just released this tick
    };

    State btn_[NUM_BUTTONS];
};
