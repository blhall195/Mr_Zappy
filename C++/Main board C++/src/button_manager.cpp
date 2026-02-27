#include "button_manager.h"

// Pin table — order must match Button enum
static const uint8_t pinTable[NUM_BUTTONS] = {
    PIN_BTN_MEASURE,   // Button::MEASURE
    PIN_BTN_DISCO,     // Button::DISCO
    PIN_BTN_CALIB,     // Button::CALIB
    PIN_BTN_SHUTDOWN,  // Button::SHUTDOWN
    PIN_BTN_FIRE,      // Button::FIRE
};

static const char* const nameTable[NUM_BUTTONS] = {
    "Measure",
    "Disco",
    "Calib",
    "Shutdown",
    "Fire",
};

void ButtonManager::begin() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        btn_[i].pin            = pinTable[i];
        btn_[i].debounced      = false;
        btn_[i].previous       = false;
        btn_[i].raw            = false;
        btn_[i].lastChangeTime = now;
        btn_[i].fell           = false;
        btn_[i].rose           = false;

        // Pins are already configured in initPins(), but read initial state
        bool pressed = (digitalRead(btn_[i].pin) == LOW);
        btn_[i].debounced = pressed;
        btn_[i].previous  = pressed;
        btn_[i].raw       = pressed;
    }
}

void ButtonManager::update() {
    uint32_t now = millis();

    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        State& b = btn_[i];

        // Save previous debounced state for edge detection
        b.previous = b.debounced;

        // Read pin (active LOW — invert so true = pressed)
        bool reading = (digitalRead(b.pin) == LOW);

        // If raw state changed, reset the debounce timer
        if (reading != b.raw) {
            b.raw = reading;
            b.lastChangeTime = now;
        }

        // If stable for the debounce interval, accept as new state
        if ((now - b.lastChangeTime) >= Timing::BUTTON_DEBOUNCE_MS) {
            b.debounced = b.raw;
        }

        // Edge flags
        b.fell = ( b.debounced && !b.previous);  // just pressed
        b.rose = (!b.debounced &&  b.previous);   // just released
    }
}

bool ButtonManager::isPressed(Button btn) const {
    return btn_[static_cast<uint8_t>(btn)].debounced;
}

bool ButtonManager::wasPressed(Button btn) const {
    return btn_[static_cast<uint8_t>(btn)].fell;
}

bool ButtonManager::wasReleased(Button btn) const {
    return btn_[static_cast<uint8_t>(btn)].rose;
}

const char* ButtonManager::name(Button btn) {
    return nameTable[static_cast<uint8_t>(btn)];
}
