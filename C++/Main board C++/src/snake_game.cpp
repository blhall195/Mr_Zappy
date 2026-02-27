#include "snake_game.h"

// ── Pixel font data (5×7, same bitmaps as Python snake.py) ──────────
const char SnakeGame::FONT_CHARS[] = "GAMEOVRSC:0123456789 ";

const uint8_t SnakeGame::FONT_DATA[] = {
    0x0e,0x11,0x01,0x1d,0x11,0x11,0x0e,  // G
    0x0e,0x11,0x11,0x1f,0x11,0x11,0x11,  // A
    0x11,0x1b,0x15,0x11,0x11,0x11,0x11,  // M
    0x1f,0x01,0x01,0x0f,0x01,0x01,0x1f,  // E
    0x0e,0x11,0x11,0x11,0x11,0x11,0x0e,  // O
    0x11,0x11,0x11,0x11,0x0a,0x0a,0x04,  // V
    0x0f,0x11,0x11,0x0f,0x05,0x09,0x11,  // R
    0x0e,0x11,0x01,0x0e,0x10,0x11,0x0e,  // S
    0x0e,0x11,0x01,0x01,0x01,0x11,0x0e,  // C
    0x00,0x04,0x04,0x00,0x04,0x04,0x00,  // :
    0x0e,0x11,0x19,0x15,0x13,0x11,0x0e,  // 0
    0x04,0x06,0x04,0x04,0x04,0x04,0x0e,  // 1
    0x0e,0x11,0x10,0x0e,0x01,0x01,0x1f,  // 2
    0x0e,0x11,0x10,0x0c,0x10,0x11,0x0e,  // 3
    0x08,0x0c,0x0a,0x09,0x1f,0x08,0x08,  // 4
    0x1f,0x01,0x0f,0x10,0x10,0x11,0x0e,  // 5
    0x0e,0x01,0x01,0x0f,0x11,0x11,0x0e,  // 6
    0x1f,0x10,0x08,0x04,0x02,0x02,0x02,  // 7
    0x0e,0x11,0x11,0x0e,0x11,0x11,0x0e,  // 8
    0x0e,0x11,0x11,0x1e,0x10,0x10,0x0e,  // 9
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // (space)
};

// ═════════════════════════════════════════════════════════════════════
// ── Public API ──────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════

void SnakeGame::begin(Adafruit_SH1107& display, ButtonManager& buttons,
                      DiscoManager& disco) {
    disp_    = &display;
    buttons_ = &buttons;
    disco_   = &disco;
    active_  = true;

    // Initial snake: 3 segments at (8,8), (7,8), (6,8)
    body_[0] = 8;  body_[1] = 8;   // head
    body_[2] = 7;  body_[3] = 8;
    body_[4] = 6;  body_[5] = 8;
    bodyLen_ = 3;

    dirX_     = 1;
    dirY_     = 0;
    score_    = 0;
    gameOver_ = false;
    justAte_  = false;

    // Game-over animation state
    flashCount_  = 0;
    flashOn_     = false;
    splashDrawn_ = false;
    flashesDone_ = false;

    randomSeed(analogRead(A5) ^ micros());
    spawnFood();
    drawInitial();

    lastMove_ = millis();

    Serial.println(F("SNAKE GAME"));
}

bool SnakeGame::update() {
    if (!active_) return false;

    uint32_t now = millis();
    // Note: buttons_->update() is called by main loop before update(),
    // so edge flags (wasPressed) are already set — don't call it again.

    // ── Game-over sequence ──────────────────────────────────────────
    if (gameOver_) {
        // Step 1: draw splash once
        if (!splashDrawn_) {
            showGameOver();
            splashDrawn_ = true;
            lastFlash_   = now;
            flashOn_     = false;
            flashCount_  = 0;
        }

        // Step 2: red flash animation
        if (!flashesDone_) {
            if (flashOn_) {
                if (now - lastFlash_ >= FLASH_MS) {
                    disco_->turnOff();
                    flashOn_   = false;
                    lastFlash_ = now;
                    flashCount_++;
                    if (flashCount_ >= FLASHES) {
                        flashesDone_  = true;
                        gameOverTime_ = now;
                    }
                }
            } else {
                if (now - lastFlash_ >= GAP_MS) {
                    disco_->setRed();
                    flashOn_   = true;
                    lastFlash_ = now;
                }
            }
            return false;
        }

        // Step 3: wait for user to see screen, then signal done
        if (now - gameOverTime_ >= END_WAIT) {
            active_ = false;
            return true;   // caller should NVIC_SystemReset()
        }
        return false;
    }

    // ── Button handling (during play) ───────────────────────────────
    if (buttons_->wasPressed(Button::MEASURE)) {   // Button 1 — turn left
        int8_t tmp = dirX_;
        dirX_ = dirY_;
        dirY_ = -tmp;
    }
    if (buttons_->wasPressed(Button::DISCO)) {      // Button 2 — turn right
        int8_t tmp = dirX_;
        dirX_ = -dirY_;
        dirY_ = tmp;
    }
    if (buttons_->wasPressed(Button::CALIB)) {      // Button 3 — exit
        Serial.println(F("Exiting Snake Game..."));
        disco_->turnOff();
        active_ = false;
        return true;
    }
    if (buttons_->wasPressed(Button::SHUTDOWN)) {   // Button 4 — power off
        Serial.println(F("Power off requested during snake game"));
        digitalWrite(PIN_POWER, LOW);
    }

    // ── Move snake at fixed interval ────────────────────────────────
    if (now - lastMove_ >= MOVE_MS) {
        moveSnake();
        lastMove_ = now;

        if (justAte_) {
            disco_->setGreen();
        } else {
            disco_->turnOff();
        }
    }

    return false;
}

// ═════════════════════════════════════════════════════════════════════
// ── Private helpers ─────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════

bool SnakeGame::inSnake(uint8_t x, uint8_t y, bool skipLast) const {
    uint16_t end = bodyLen_ - (skipLast ? 1 : 0);
    for (uint16_t i = 0; i < end; i++) {
        if (body_[i * 2] == x && body_[i * 2 + 1] == y)
            return true;
    }
    return false;
}

void SnakeGame::spawnFood() {
    do {
        foodX_ = random(0, GRID_W);
        foodY_ = random(0, GRID_H);
    } while (inSnake(foodX_, foodY_));
    drawCell(foodX_, foodY_, SH110X_WHITE);
    disp_->display();
}

void SnakeGame::moveSnake() {
    if (gameOver_) return;

    justAte_ = false;

    uint8_t headX = body_[0];
    uint8_t headY = body_[1];
    uint8_t newX  = (headX + dirX_ + GRID_W) % GRID_W;
    uint8_t newY  = (headY + dirY_ + GRID_H) % GRID_H;

    // Collision with body (exclude tail — it will move away)
    if (inSnake(newX, newY, true)) {
        gameOver_ = true;
        Serial.print(F("Game Over! Score: "));
        Serial.println(score_);
        return;
    }

    // Shift body array right by 2 to make room for new head
    if (bodyLen_ < MAX_LEN) {
        // Move everything right
        memmove(&body_[2], &body_[0], bodyLen_ * 2);
    }
    body_[0] = newX;
    body_[1] = newY;
    bodyLen_++;

    drawCell(newX, newY, SH110X_WHITE);

    if (newX == foodX_ && newY == foodY_) {
        score_++;
        justAte_ = true;
        spawnFood();
    } else {
        // Remove tail
        uint8_t tailX = body_[(bodyLen_ - 1) * 2];
        uint8_t tailY = body_[(bodyLen_ - 1) * 2 + 1];
        bodyLen_--;
        drawCell(tailX, tailY, SH110X_BLACK);
    }

    disp_->display();
}

void SnakeGame::drawCell(uint8_t cx, uint8_t cy, uint16_t color) {
    disp_->fillRect(cx * CELL, cy * CELL, CELL, CELL, color);
}

void SnakeGame::drawInitial() {
    disp_->clearDisplay();

    for (uint16_t i = 0; i < bodyLen_; i++) {
        drawCell(body_[i * 2], body_[i * 2 + 1], SH110X_WHITE);
    }
    drawCell(foodX_, foodY_, SH110X_WHITE);

    disp_->display();
}

// ── Game Over splash (pixel-art font, same layout as Python) ────────

void SnakeGame::drawChar(char ch, int16_t x, int16_t y, uint8_t scale) {
    // Find character in font index
    const char* p = strchr(FONT_CHARS, ch);
    if (!p) return;
    uint8_t idx = (uint8_t)(p - FONT_CHARS);

    uint16_t offset = idx * 7;
    for (uint8_t row = 0; row < 7; row++) {
        uint8_t bits = FONT_DATA[offset + row];
        for (uint8_t col = 0; col < 5; col++) {
            if (bits & (1 << col)) {
                disp_->fillRect(x + col * scale, y + row * scale,
                                scale, scale, SH110X_WHITE);
            }
        }
    }
}

void SnakeGame::drawText(const char* text, int16_t yPos, uint8_t scale) {
    uint8_t len = strlen(text);
    int16_t charW  = 5 * scale + scale;  // char width + spacing
    int16_t totalW = len * charW - scale;
    int16_t startX = (128 - totalW) / 2;
    for (uint8_t i = 0; i < len; i++) {
        drawChar(text[i], startX + i * charW, yPos, scale);
    }
}

void SnakeGame::showGameOver() {
    disp_->clearDisplay();

    // "GAME" and "OVER" at scale 3 (same Y positions as Python)
    drawText("GAME", 15, 3);
    drawText("OVER", 45, 3);

    // "SCORE:<n>" at scale 2
    char buf[16];
    snprintf(buf, sizeof(buf), "SCORE:%u", score_);
    drawText(buf, 95, 2);

    disp_->display();
}
