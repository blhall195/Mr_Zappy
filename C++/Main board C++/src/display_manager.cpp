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

void DisplayManager::showSplash(float progress) {
    if (!_initialized) return;

    _display.clearDisplay();
    _display.setTextColor(SH110X_WHITE);

    // ── "DiscoX" title — text size 3, centred ───────────────────
    _display.setTextSize(3);
    _display.setCursor(10, 8);
    _display.print(F("DiscoX"));

    // ── Device silhouette (logo) ────────────────────────────────
    // The logo has concave TOP and BOTTOM edges (dipping inward toward
    // the centre) with relatively straight vertical sides and rounded
    // corners.  We trace the outline by computing the top-edge Y and
    // bottom-edge Y as functions of X.
    static constexpr int16_t BODY_CX  = 46;   // centre X of body
    static constexpr int16_t BODY_CY  = 64;   // centre Y of body
    static constexpr int16_t BODY_HW  = 40;   // half-width
    static constexpr int16_t BODY_HH  = 20;   // half-height at the sides
    static constexpr int16_t SCOOP    = 6;     // how far top/bottom edges dip inward
    static constexpr int16_t CORNER_R = 8;     // visual corner rounding zone

    // Given an X position, return the half-height of the body at that X.
    // Two smooth dips with sharp pointed cusps where the curves meet
    // (at left edge, centre, and right edge).
    auto bodyHH = [&](int16_t x) -> float {
        float t = (float)(x - (BODY_CX - BODY_HW)) / (float)(2 * BODY_HW); // 0..1
        return BODY_HH - SCOOP * fabsf(sinf(2.0f * 3.14159f * t));
    };

    // Trace top and bottom edges column by column (2-pass for thickness)
    for (int pass = 0; pass < 2; pass++) {
        float inset = (float)pass;
        int16_t prevTy = -1, prevBy = -1;
        for (int16_t x = BODY_CX - BODY_HW; x <= BODY_CX + BODY_HW; x++) {
            float hh = bodyHH(x) - inset;
            int16_t ty = BODY_CY - (int16_t)hh;   // top edge Y
            int16_t by = BODY_CY + (int16_t)hh;   // bottom edge Y

            // Draw top and bottom edge pixels
            _display.drawPixel(x, ty, SH110X_WHITE);
            _display.drawPixel(x, by, SH110X_WHITE);

            // Left and right extremes: draw full vertical span
            if (x == BODY_CX - BODY_HW + pass || x == BODY_CX + BODY_HW - pass) {
                _display.drawLine(x, ty, x, by, SH110X_WHITE);
            }

            // Connect to previous column to fill gaps in the curve
            if (prevTy >= 0) {
                if (ty != prevTy) _display.drawLine(x-1, min(ty,prevTy), x, max(ty,prevTy), SH110X_WHITE);
                if (by != prevBy) _display.drawLine(x-1, min(by,prevBy), x, max(by,prevBy), SH110X_WHITE);
            }
            prevTy = ty;
            prevBy = by;
        }
    }

    // ── 4 button squares (no surrounding frame) ──────────────────
    static constexpr int16_t BTN_SIZE = 8;
    static constexpr int16_t BTN_GAP  = 2;
    static constexpr int16_t BTN_X0   = BODY_CX - BODY_HW + 8;
    static constexpr int16_t BTN_Y0   = BODY_CY - BTN_SIZE / 2;

    for (int i = 0; i < 4; i++) {
        int16_t bx = BTN_X0 + i * (BTN_SIZE + BTN_GAP);
        _display.drawRoundRect(bx, BTN_Y0, BTN_SIZE, BTN_SIZE, 2, SH110X_WHITE);
    }

    // ── Disco square — slightly shorter, centred on right side ──────
    static constexpr int16_t DSQ_W = 22;
    static constexpr int16_t DSQ_H = 18;
    static constexpr int16_t DSQ_X = BTN_X0 + 4 * (BTN_SIZE + BTN_GAP) + 4;
    static constexpr int16_t DSQ_Y = BODY_CY - DSQ_H / 2;

    _display.drawRoundRect(DSQ_X, DSQ_Y, DSQ_W, DSQ_H, 4, SH110X_WHITE);

    // ── Laser beam — flashing on/off as the device loads ──────────
    static constexpr int16_t BEAM_Y   = BODY_CY;
    static constexpr int16_t BEAM_X0  = BODY_CX + BODY_HW + 2;
    static constexpr int16_t BEAM_X1  = 112;

    bool laserOn = ((millis() / 400) % 2) == 0;  // toggle every 400ms

    if (laserOn) {
        // Main beam (2px thick)
        _display.drawLine(BEAM_X0, BEAM_Y,     BEAM_X1, BEAM_Y,     SH110X_WHITE);
        _display.drawLine(BEAM_X0, BEAM_Y - 1, BEAM_X1, BEAM_Y - 1, SH110X_WHITE);

        // Starburst at the tip
        static constexpr int16_t STAR_CX = BEAM_X1 + 2;
        static constexpr int16_t STAR_CY = BEAM_Y;
        static constexpr int16_t RAY_LEN = 8;

        static constexpr float RAY_ANGLES[] = {
            0.0f, 0.7854f, 1.5708f, 2.3562f,
            3.1416f, 3.9270f, 4.7124f, 5.4978f
        };
        for (float a : RAY_ANGLES) {
            int16_t ex = STAR_CX + (int16_t)(cosf(a) * RAY_LEN);
            int16_t ey = STAR_CY + (int16_t)(sinf(a) * RAY_LEN);
            _display.drawLine(STAR_CX, STAR_CY, ex, ey, SH110X_WHITE);
        }
        _display.fillCircle(STAR_CX, STAR_CY, 2, SH110X_WHITE);
    }

    // ── Progress bar ────────────────────────────────────────────
    static constexpr int16_t BAR_X = 10;
    static constexpr int16_t BAR_Y = 112;
    static constexpr int16_t BAR_W = 108;
    static constexpr int16_t BAR_H = 10;

    _display.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, SH110X_WHITE);

    float clamped = constrain(progress, 0.0f, 1.0f);
    int16_t fillW = (int16_t)(clamped * (BAR_W - 2));
    if (fillW > 0) {
        _display.fillRect(BAR_X + 1, BAR_Y + 1, fillW, BAR_H - 2, SH110X_WHITE);
    }

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
