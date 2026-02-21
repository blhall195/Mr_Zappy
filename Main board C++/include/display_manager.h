#pragma once

#include <Arduino.h>
#include <Adafruit_SH110X.h>
#include "config.h"

class DisplayManager {
public:
    /// Initialize the SH1107 display. Returns true on success.
    bool begin();

    /// Draw the main measurement screen from cached values
    void initScreen();

    /// Update all three reading labels at once
    void updateSensorReadings(float distance, float azimuth, float inclination);

    /// Update distance label (meters). 0 = blank.
    void updateDistance(float distance);

    /// Update distance label with arbitrary text (e.g. anomaly message)
    void updateDistanceText(const char* text);

    /// Update azimuth label (degrees)
    void updateAzimuth(float azimuth);

    /// Update inclination label (degrees)
    void updateInclination(float inclination);

    /// Update battery bar (0-100%)
    void updateBattery(float percentage);

    /// Show/hide BT label
    void updateBTLabel(bool connected);

    /// Show pending BLE reading count (0 = hidden)
    void updateBTNumber(uint16_t pending);

    /// Clear display to black
    void blankScreen();

    /// Show "Starting Menu" message
    void showStartingMenu();

    /// Show "Device Initialising" message
    void showInitialisingMessage();

    /// Push buffer to OLED (call after updates)
    void refresh();

    /// Direct access to the underlying display (used by MenuManager)
    Adafruit_SH1107& getDisplay() { return _display; }

private:
    // Keep I2C at 400 kHz before AND after display transactions
    Adafruit_SH1107 _display{SH1107_WIDTH, SH1107_HEIGHT, &Wire,
                              -1, 400000, 400000};
    bool _initialized = false;

    // Cached display state (set by updateXxx, drawn by refresh)
    float    _distance    = 0.0f;
    char     _distText[16] = "";
    bool     _distIsText  = false;
    float    _azimuth     = 0.0f;
    float    _inclination = 0.0f;
    float    _battery     = 0.0f;
    bool     _btConnected = false;
    uint16_t _btPending   = 0;

    void drawMainScreen();
    void drawBattery(float pct);
};
