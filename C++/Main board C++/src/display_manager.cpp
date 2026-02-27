#include "display_manager.h"

// ── Layout constants (match Python display_manager.py positions) ────
// Battery icon — top-right
static constexpr int16_t BAT_X      = 90;
static constexpr int16_t BAT_Y      = 0;
static constexpr int16_t BAT_W      = 32;
static constexpr int16_t BAT_H      = 15;
static constexpr int16_t BAT_TIP_W  = 3;
static constexpr int16_t BAT_TIP_H  = 6;
static constexpr int16_t BAT_FILL_X = BAT_X + 1;
static constexpr int16_t BAT_FILL_Y = BAT_Y + 1;
static constexpr int16_t BAT_FILL_W = 30;
static constexpr int16_t BAT_FILL_H = 13;

// BT label — top-left (text size 2)
static constexpr int16_t BT_X       = 0;
static constexpr int16_t BT_Y       = 0;
static constexpr int16_t BT_NUM_X   = 30;

// Data lines — text size 3
static constexpr int16_t DIST_Y     = 34;
static constexpr int16_t AZ_Y       = 68;
static constexpr int16_t INC_Y      = 100;

// Degree symbol drawn as a small circle (looks better than CP437 '\xF8' at size 3)
static constexpr int16_t DEG_RADIUS  = 3;
static constexpr int16_t DEG_OFFSET_X = 2;   // gap after text
static constexpr int16_t DEG_OFFSET_Y = 3;   // down from top of text line

// ── Public API ─────────────────────────────────────────────────────

bool DisplayManager::begin() {
    if (!_display.begin(SH1107_ADDR, true)) {
        return false;
    }
    _display.setRotation(2);          // 180 deg to match Python layout
    _display.clearDisplay();
    _display.display();
    _initialized = true;
    return true;
}

void DisplayManager::initScreen() {
    if (!_initialized) return;
    drawMainScreen();
    _display.display();
}

void DisplayManager::updateSensorReadings(float distance, float azimuth,
                                           float inclination) {
    _distance    = distance;
    _distIsText  = false;
    _azimuth     = azimuth;
    _inclination = inclination;
}

void DisplayManager::updateDistance(float distance) {
    _distance   = distance;
    _distIsText = false;
}

void DisplayManager::updateDistanceText(const char* text) {
    strncpy(_distText, text, sizeof(_distText) - 1);
    _distText[sizeof(_distText) - 1] = '\0';
    _distIsText = true;
}

void DisplayManager::updateAzimuth(float azimuth) {
    _azimuth = azimuth;
}

void DisplayManager::updateInclination(float inclination) {
    _inclination = inclination;
}

void DisplayManager::updateBattery(float percentage) {
    _battery = constrain(percentage, 0.0f, 100.0f);
}

void DisplayManager::updateBTLabel(bool connected) {
    _btConnected = connected;
}

void DisplayManager::updateBTNumber(uint16_t pending) {
    _btPending = pending;
}

void DisplayManager::blankScreen() {
    if (!_initialized) return;
    _display.clearDisplay();
    _display.display();
}

void DisplayManager::showStartingMenu() {
    if (!_initialized) return;
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SH110X_WHITE);
    _display.setCursor(0, 10);
    _display.println(F("Starting"));
    _display.println(F("Menu"));
    _display.println();
    _display.println(F("If this takes longer"));
    _display.println(F(" than 10 seconds"));
    _display.println(F(" turn the device"));
    _display.println(F(" on/off again"));
    _display.println(F(" and reattempt"));
    _display.display();
}

void DisplayManager::showInitialisingMessage() {
    if (!_initialized) return;
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SH110X_WHITE);
    _display.setCursor(0, 10);
    _display.println(F("Device Initialising"));
    _display.println(F("Please wait..."));
    _display.display();
}

void DisplayManager::refresh() {
    if (!_initialized) return;
    drawMainScreen();
    _display.display();
}

// ── Private drawing helpers ────────────────────────────────────────

void DisplayManager::drawMainScreen() {
    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);

    // ── BT label (top-left, size 2) ─────────────────────────────
    _display.setTextSize(2);
    if (_btConnected) {
        _display.setCursor(BT_X, BT_Y);
        _display.print(F("BT"));
    }

    // ── BT pending count ────────────────────────────────────────
    if (_btPending > 0) {
        _display.setCursor(BT_NUM_X, BT_Y);
        _display.print(_btPending);
    }

    // ── Battery bar (top-right) ─────────────────────────────────
    drawBattery(_battery);

    // ── Distance (size 3) ───────────────────────────────────────
    _display.setTextSize(3);
    _display.setCursor(0, DIST_Y);
    if (_distIsText) {
        _display.print(_distText);
    } else if (_distance != 0.0f) {
        _display.print(_distance, 2);
        _display.print('m');
    }

    // ── Azimuth (size 3) ────────────────────────────────────────
    _display.setCursor(0, AZ_Y);
    _display.print(_azimuth, 1);
    drawDegreeSymbol(AZ_Y);

    // ── Inclination (size 3) ────────────────────────────────────
    _display.setCursor(0, INC_Y);
    _display.print(_inclination, 1);
    drawDegreeSymbol(INC_Y);
}

void DisplayManager::drawDegreeSymbol(int16_t y) {
    int16_t cx = _display.getCursorX() + DEG_OFFSET_X + DEG_RADIUS;
    int16_t cy = y + DEG_OFFSET_Y + DEG_RADIUS;
    _display.drawCircle(cx, cy, DEG_RADIUS,     SH110X_WHITE);
    _display.drawCircle(cx, cy, DEG_RADIUS - 1, SH110X_WHITE);
}

void DisplayManager::drawBattery(float pct) {
    // Outline
    _display.drawRect(BAT_X, BAT_Y, BAT_W, BAT_H, SH110X_WHITE);

    // Positive terminal tip
    _display.fillRect(BAT_X + BAT_W, BAT_Y + (BAT_H - BAT_TIP_H) / 2,
                      BAT_TIP_W, BAT_TIP_H, SH110X_WHITE);

    // Fill proportional to charge level
    int16_t fillW = (int16_t)((pct / 100.0f) * BAT_FILL_W);
    if (fillW > 0) {
        _display.fillRect(BAT_FILL_X, BAT_FILL_Y,
                          fillW, BAT_FILL_H, SH110X_WHITE);
    }
}
