#pragma once
// Disco Manager — NeoPixel LED effects (non-blocking state machine)
// Session 9: static colors, disco rainbow (with gyro-driven wild mode), purple pulse.

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

/// Active LED effect
enum class DiscoEffect : uint8_t {
    OFF,
    STATIC,       // solid colour, no animation
    DISCO,        // HSV rainbow cycling, shake → wild mode
    PURPLE_PULSE  // sine-wave brightness modulation
};

class DiscoManager {
public:
    /// Call once in setup() after pin init
    void begin();

    /// Call every loop() iteration — drives animations non-blockingly
    void update(float accelX, float accelY, float accelZ);

    // ── Static colours ──────────────────────────────────────────────
    void setRed();
    void setGreen();
    void setBlue();
    void setWhite();
    void turnOff();

    // ── Animated effects ────────────────────────────────────────────
    void startDisco();
    void setPurple();       // starts purple pulse

    // ── State queries ───────────────────────────────────────────────
    bool isDiscoActive()  const { return effect_ == DiscoEffect::DISCO; }
    bool isPulseActive()  const { return effect_ == DiscoEffect::PURPLE_PULSE; }
    bool isActive()       const { return effect_ != DiscoEffect::OFF; }
    DiscoEffect effect()  const { return effect_; }

private:
    Adafruit_NeoPixel pixel_{NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800};
    DiscoEffect effect_ = DiscoEffect::OFF;

    // ── Brightness ──────────────────────────────────────────────────
    float brightness_ = Disco::BASE_BRIGHTNESS;

    // ── Disco state ─────────────────────────────────────────────────
    uint16_t hueStep_       = 0;        // rotating hue (0-359)
    bool     wildLatched_   = false;
    uint32_t lastShakeTime_ = 0;
    uint32_t lastFrameTime_ = 0;

    // ── Purple pulse state ──────────────────────────────────────────
    float    pulsePhase_     = 0.0f;
    uint32_t lastPulseTime_  = 0;

    // ── Helpers ─────────────────────────────────────────────────────
    void setPixel(uint8_t r, uint8_t g, uint8_t b);
    void stopAll();

    /// HSV → RGB (h 0.0-1.0, s 0.0-1.0, v 0.0-1.0) → 0-255 per channel
    static void hsvToRgb(float h, float s, float v,
                         uint8_t& r, uint8_t& g, uint8_t& b);
};
