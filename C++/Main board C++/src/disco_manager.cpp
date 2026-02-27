#include "disco_manager.h"
#include <math.h>

// ── Lifecycle ────────────────────────────────────────────────────────

void DiscoManager::begin() {
    pixel_.begin();
    pixel_.setBrightness(255);   // HW brightness max; we software-scale
    pixel_.show();
}

// ── Main update — call every loop() iteration ───────────────────────

void DiscoManager::update(float accelX, float accelY, float accelZ) {
    uint32_t now = millis();

    if (effect_ == DiscoEffect::DISCO) {
        // --- Shake detection for wild mode ---
        if (fabsf(accelX) > Disco::SHAKE_THRESHOLD ||
            fabsf(accelY) > Disco::SHAKE_THRESHOLD ||
            fabsf(accelZ) > Disco::SHAKE_THRESHOLD) {
            wildLatched_ = true;
            lastShakeTime_ = now;
            brightness_ = Disco::WILD_BRIGHTNESS;
        }

        // Return to dim after timeout
        if (wildLatched_ && (now - lastShakeTime_ > Disco::SHAKE_TIMEOUT_MS)) {
            wildLatched_ = false;
            brightness_ = Disco::BASE_BRIGHTNESS;
        }

        bool isWild = (brightness_ == Disco::WILD_BRIGHTNESS);
        uint32_t interval = isWild ? Disco::WILD_SLEEP_MS : Disco::BASE_SLEEP_MS;

        if (now - lastFrameTime_ >= interval) {
            lastFrameTime_ = now;

            float hue;
            if (isWild) {
                // Random hue in wild mode
                hue = random(360) / 360.0f;
            } else {
                hue = hueStep_ / 360.0f;
                hueStep_ = (hueStep_ + Disco::COLOR_STEP_BASE) % 360;
            }

            uint8_t r, g, b;
            hsvToRgb(hue, 1.0f, 1.0f, r, g, b);
            setPixel(r, g, b);
        }

    } else if (effect_ == DiscoEffect::PURPLE_PULSE) {
        // ~33 Hz update (30 ms)
        if (now - lastPulseTime_ >= 30) {
            lastPulseTime_ = now;

            float pulseBright = 0.55f + 0.45f * sinf(pulsePhase_);
            pulsePhase_ += 0.3f;

            uint8_t r = (uint8_t)(255.0f * pulseBright);
            uint8_t g = 0;
            uint8_t b = (uint8_t)(255.0f * pulseBright);
            pixel_.setPixelColor(0, pixel_.Color(r, g, b));
            pixel_.show();
        }
    }
    // STATIC and OFF need no per-frame work
}

// ── Static colours ──────────────────────────────────────────────────

void DiscoManager::setRed() {
    stopAll();
    effect_ = DiscoEffect::STATIC;
    setPixel(255, 0, 0);
}

void DiscoManager::setGreen() {
    stopAll();
    effect_ = DiscoEffect::STATIC;
    setPixel(0, 255, 0);
}

void DiscoManager::setBlue() {
    stopAll();
    effect_ = DiscoEffect::STATIC;
    setPixel(0, 0, 255);
}

void DiscoManager::setWhite() {
    stopAll();
    effect_ = DiscoEffect::STATIC;
    setPixel(255, 255, 255);
}

void DiscoManager::turnOff() {
    stopAll();
}

// ── Animated effects ────────────────────────────────────────────────

void DiscoManager::startDisco() {
    if (effect_ == DiscoEffect::DISCO) return;   // already running
    stopAll();
    effect_        = DiscoEffect::DISCO;
    brightness_    = Disco::BASE_BRIGHTNESS;
    wildLatched_   = false;
    lastShakeTime_ = 0;
    hueStep_       = 0;
    lastFrameTime_ = millis();
}

void DiscoManager::setPurple() {
    stopAll();
    effect_        = DiscoEffect::PURPLE_PULSE;
    pulsePhase_    = 0.0f;
    lastPulseTime_ = millis();
}

// ── Internal helpers ────────────────────────────────────────────────

void DiscoManager::stopAll() {
    effect_ = DiscoEffect::OFF;
    wildLatched_ = false;
    brightness_ = Disco::BASE_BRIGHTNESS;
    pixel_.setPixelColor(0, 0);
    pixel_.show();
}

void DiscoManager::setPixel(uint8_t r, uint8_t g, uint8_t b) {
    // Apply software brightness scaling
    uint8_t rr = (uint8_t)(r * brightness_);
    uint8_t gg = (uint8_t)(g * brightness_);
    uint8_t bb = (uint8_t)(b * brightness_);
    pixel_.setPixelColor(0, pixel_.Color(rr, gg, bb));
    pixel_.show();
}

void DiscoManager::hsvToRgb(float h, float s, float v,
                             uint8_t& r, uint8_t& g, uint8_t& b) {
    // Identical algorithm to Python's DiscoMode.hsv_to_rgb
    int i = (int)(h * 6.0f);
    float f = h * 6.0f - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);
    i = i % 6;
    float rv, gv, bv;
    switch (i) {
        case 0: rv = v; gv = t; bv = p; break;
        case 1: rv = q; gv = v; bv = p; break;
        case 2: rv = p; gv = v; bv = t; break;
        case 3: rv = p; gv = q; bv = v; break;
        case 4: rv = t; gv = p; bv = v; break;
        default: rv = v; gv = p; bv = q; break;   // case 5
    }
    r = (uint8_t)(rv * 255.0f);
    g = (uint8_t)(gv * 255.0f);
    b = (uint8_t)(bv * 255.0f);
}
