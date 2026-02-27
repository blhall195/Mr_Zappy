#pragma once
// Snake Game — C++ port of Main board/lib/snake.py
// 16x16 grid on 128x128 SH1107 OLED, 8-pixel cells.

#include <Arduino.h>
#include <Adafruit_SH110X.h>
#include "button_manager.h"
#include "disco_manager.h"
#include "config.h"

class SnakeGame {
public:
    /// Initialise and take over the display.  Call once from setup().
    void begin(Adafruit_SH1107& display, ButtonManager& buttons,
               DiscoManager& disco);

    /// Drive one iteration — call from loop().
    /// Returns true when the game is completely finished (post-game-over
    /// delay elapsed) and the caller should reset the MCU.
    bool update();

    /// True while the game is running or showing the game-over screen.
    bool isActive() const { return active_; }

private:
    // ── Constants ────────────────────────────────────────────────────
    static constexpr uint8_t CELL       = 8;
    static constexpr uint8_t GRID_W     = 16;
    static constexpr uint8_t GRID_H     = 16;
    static constexpr uint16_t MOVE_MS   = 200;
    static constexpr uint16_t FLASH_MS  = 300;
    static constexpr uint16_t GAP_MS    = 200;
    static constexpr uint8_t  FLASHES   = 3;
    static constexpr uint16_t END_WAIT  = 3000;

    // Maximum snake length (entire grid)
    static constexpr uint16_t MAX_LEN   = GRID_W * GRID_H;

    // ── Pixel font (5x7, same as Python) ────────────────────────────
    static const char  FONT_CHARS[];
    static const uint8_t FONT_DATA[];

    // ── Hardware refs ───────────────────────────────────────────────
    Adafruit_SH1107* disp_    = nullptr;
    ButtonManager*   buttons_ = nullptr;
    DiscoManager*    disco_   = nullptr;

    // ── Game state ──────────────────────────────────────────────────
    bool     active_    = false;
    bool     gameOver_  = false;
    bool     justAte_   = false;
    uint16_t score_     = 0;

    // Snake body: flat x,y pairs.  body_[0..1] = head.
    uint8_t  body_[MAX_LEN * 2];
    uint16_t bodyLen_   = 0;        // number of segments (pairs)

    int8_t   dirX_      = 1;
    int8_t   dirY_      = 0;
    uint8_t  foodX_     = 0;
    uint8_t  foodY_     = 0;

    // ── Timing ──────────────────────────────────────────────────────
    uint32_t lastMove_      = 0;
    uint32_t gameOverTime_  = 0;

    // Game-over animation state
    uint8_t  flashCount_    = 0;
    bool     flashOn_       = false;
    uint32_t lastFlash_     = 0;
    bool     splashDrawn_   = false;
    bool     flashesDone_   = false;

    // ── Helpers ─────────────────────────────────────────────────────
    bool inSnake(uint8_t x, uint8_t y, bool skipLast = false) const;
    void spawnFood();
    void moveSnake();
    void drawCell(uint8_t cx, uint8_t cy, uint16_t color);
    void drawInitial();
    void showGameOver();
    void drawChar(char ch, int16_t x, int16_t y, uint8_t scale);
    void drawText(const char* text, int16_t yPos, uint8_t scale);
};
