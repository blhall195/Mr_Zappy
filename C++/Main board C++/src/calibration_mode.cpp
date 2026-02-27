#include "calibration_mode.h"
#include <math.h>

// ── Initialization ──────────────────────────────────────────────────

void CalibrationMode::begin(
    ButtonManager& btns, DisplayManager& disp, DiscoManager& disco,
    LaserEgismos& laser, RM3100& magSensor, Adafruit_ISM330DHCX& imu,
    ConfigManager& cfgMgr, MagCal::Calibration& cal,
    bool shortCal)
{
    btns_      = &btns;
    disp_      = &disp;
    disco_     = &disco;
    laser_     = &laser;
    magSensor_ = &magSensor;
    imu_       = &imu;
    cfgMgr_    = &cfgMgr;
    cal_       = &cal;

    // Clear data
    magArray_.clear();
    gravArray_.clear();
    bufferCount_ = 0;
    bufferIdx_ = 0;
    waitingForStable_ = false;
    iteration_ = 0;
    holdCounter_ = 0.0f;
    beepActive_ = false;
    resultMagAcc_ = 0.0f;
    resultGravAcc_ = 0.0f;
    resultAccuracy_ = 0.0f;

    memset(coverageZones_, 0, sizeof(coverageZones_));

    // Turn on laser for calibration
    laser_->setLaser(true);

    isShortCal_ = shortCal;

    if (shortCal) {
        Serial.println(F("Calibration mode: Short Calibration"));
        state_ = CalibState::INTRO_ALIGNMENT;
        showAlignmentIntro();
    } else {
        Serial.println(F("Calibration mode: Long Calibration"));
        state_ = CalibState::INTRO_ELLIPSOID;
        showEllipsoidIntro();
    }
}

// ── Main update loop ────────────────────────────────────────────────

bool CalibrationMode::update() {
    if (state_ == CalibState::INACTIVE || state_ == CalibState::DONE) {
        return true;
    }

    // Note: btns_->update() is called by loop() before calMode.update(),
    // so we must NOT call it again here — that would clear edge flags.
    updateBeep();

    // Shutdown button always works
    if (btns_->wasPressed(Button::SHUTDOWN)) {
        Serial.println(F("Calibration: shutdown requested"));
        digitalWrite(PIN_POWER, LOW);
        return false;
    }

    switch (state_) {
        case CalibState::INTRO_ELLIPSOID:
        case CalibState::INTRO_ALIGNMENT:     updateIntro(); break;
        case CalibState::COLLECTING_ELLIPSOID:
        case CalibState::COLLECTING_ALIGNMENT: updateCollecting(); break;
        case CalibState::CALCULATING_ELLIPSOID:  updateCalculatingEllipsoid(); break;
        case CalibState::CALCULATING_ALIGNMENT:  updateCalculatingAlignment(); break;
        case CalibState::CALCULATING_SHORT:      updateCalculatingShort(); break;
        case CalibState::SHOW_RESULTS:         updateShowResults(); break;
        case CalibState::SAVING:               updateSaving(); break;
        default: break;
    }

    return (state_ == CalibState::DONE);
}

// ── State: INTRO (ellipsoid or alignment) ───────────────────────────

void CalibrationMode::updateIntro() {
    // Dismiss intro screen on any button press
    if (btns_->wasPressed(Button::MEASURE) ||
        btns_->wasPressed(Button::DISCO)   ||
        btns_->wasPressed(Button::CALIB)   ||
        btns_->wasPressed(Button::FIRE)) {

        if (state_ == CalibState::INTRO_ELLIPSOID) {
            // Start ellipsoid collection
            targetCount_ = 56;
            iteration_ = 0;
            magArray_.clear();
            gravArray_.clear();
            magArray_.reserve(56);
            gravArray_.reserve(56);
            bufferCount_ = 0;
            bufferIdx_ = 0;
            waitingForStable_ = false;

            state_ = CalibState::COLLECTING_ELLIPSOID;
            showEllipsoidScreen();
            Serial.println(F("Starting ellipsoid collection (56 points)..."));
        } else {
            // Start alignment collection
            targetCount_ = 24;
            iteration_ = 0;
            magArray_.clear();
            gravArray_.clear();
            magArray_.reserve(24);
            gravArray_.reserve(24);
            bufferCount_ = 0;
            bufferIdx_ = 0;
            waitingForStable_ = false;

            state_ = CalibState::COLLECTING_ALIGNMENT;
            showAlignmentProgress();
            Serial.println(F("Starting alignment collection (24 points: 3 dirs x 8 rotations)..."));
        }
    }
}

// ── State: COLLECTING ───────────────────────────────────────────────

void CalibrationMode::updateCollecting() {
    // Sample sensors at ~100 Hz
    uint32_t now = millis();
    if (now - lastSampleTime_ < 10) return;
    lastSampleTime_ = now;

    // Read raw sensor data
    Eigen::Vector3f magReading, gravReading;
    readSensors(magReading, gravReading);

    // Push into rolling circular buffer
    magBuffer_[bufferIdx_] = magReading;
    gravBuffer_[bufferIdx_] = gravReading;
    bufferIdx_ = (bufferIdx_ + 1) % BUFFER_SIZE;
    if (bufferCount_ < BUFFER_SIZE) bufferCount_++;

    // MEASURE button starts a capture attempt
    if (btns_->wasPressed(Button::MEASURE) && !waitingForStable_) {
        waitingForStable_ = true;
        disco_->setRed();
    }

    // Check stability when waiting and buffer is full
    if (waitingForStable_ && bufferCount_ >= BUFFER_SIZE) {
        bool magConsistent = isConsistent(magBuffer_, BUFFER_SIZE, 0.3f);
        bool gravConsistent = isConsistent(gravBuffer_, BUFFER_SIZE, 0.1f);

        if (magConsistent && gravConsistent) {
            // Record averaged point
            Eigen::Vector3f avgMag = average(magBuffer_, BUFFER_SIZE);
            Eigen::Vector3f avgGrav = average(gravBuffer_, BUFFER_SIZE);

            recordPoint(avgMag, avgGrav);
            disco_->setGreen();
            beep();

            iteration_++;
            waitingForStable_ = false;
            bufferCount_ = 0;
            bufferIdx_ = 0;

            // Update display
            if (state_ == CalibState::COLLECTING_ELLIPSOID) {
                updateCoverageBar(avgGrav);
                showEllipsoidScreen();
            } else {
                showAlignmentProgress();

                // Triple beep at direction changes (every 8 points)
                if (iteration_ % 8 == 0 && iteration_ < targetCount_) {
                    beepTriple();
                    Serial.println(F("Change direction..."));
                }
            }

            Serial.print(F("Point "));
            Serial.print(iteration_);
            Serial.print(F("/"));
            Serial.println(targetCount_);

            // Brief green flash then turn off LED
            delay(200);
            disco_->turnOff();

            // Check if collection is complete
            if (iteration_ >= targetCount_) {
                Serial.println(F("Collection complete. Calculating..."));
                if (state_ == CalibState::COLLECTING_ELLIPSOID) {
                    state_ = CalibState::CALCULATING_ELLIPSOID;
                } else if (isShortCal_) {
                    state_ = CalibState::CALCULATING_SHORT;
                } else {
                    state_ = CalibState::CALCULATING_ALIGNMENT;
                }
            }
        }
    }
}

// ── State: CALCULATING_ELLIPSOID ────────────────────────────────────

void CalibrationMode::updateCalculatingEllipsoid() {
    // Show calculating message
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    calculateEllipsoid();

    // Transition to alignment phase
    Serial.println(F("Ellipsoid done. Moving to alignment phase."));
    state_ = CalibState::INTRO_ALIGNMENT;
    showAlignmentIntro();
}

// ── State: CALCULATING_ALIGNMENT ────────────────────────────────────

void CalibrationMode::updateCalculatingAlignment() {
    // Show calculating message
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    calculateAlignment();

    state_ = CalibState::SHOW_RESULTS;
    showResultsScreen();
}

// ── State: CALCULATING_SHORT ────────────────────────────────────────

void CalibrationMode::updateCalculatingShort() {
    // Show calculating message
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    // Step 1: Update ellipsoid with the 24 alignment points
    Serial.println(F("Short cal: updating ellipsoid with 24 points..."));
    calculateEllipsoid();

    // Step 2: Run alignment on the same 24 points
    Serial.println(F("Short cal: running alignment..."));
    calculateAlignment();

    state_ = CalibState::SHOW_RESULTS;
    showResultsScreen();
}

// ── State: SHOW_RESULTS ─────────────────────────────────────────────

void CalibrationMode::updateShowResults() {
    // Hold MEASURE+DISCO to save, hold DISCO alone to discard
    bool b1 = btns_->isPressed(Button::MEASURE);
    bool b2 = btns_->isPressed(Button::DISCO);

    if (b1 && b2) {
        holdCounter_ += 0.01f;
        if (holdCounter_ >= HOLD_TIME) {
            Serial.println(F("Saving calibration..."));
            state_ = CalibState::SAVING;
            showSavingScreen();
        }
    } else if (!b1 && b2) {
        holdCounter_ += 0.01f;
        if (holdCounter_ >= HOLD_TIME) {
            Serial.println(F("Calibration discarded."));
            state_ = CalibState::DONE;
        }
    } else {
        holdCounter_ = 0.0f;
    }
}

// ── State: SAVING ───────────────────────────────────────────────────

void CalibrationMode::updateSaving() {
    auto& d = disp_->getDisplay();
    if (saveCalibration()) {
        Serial.println(F("Calibration saved to flash."));

        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(20, 50);
        d.print(F("Saved!"));
        d.display();
        delay(1500);
    } else {
        Serial.println(F("Failed to save calibration!"));

        d.clearDisplay();
        d.setTextColor(SH110X_WHITE);
        d.setTextSize(2);
        d.setCursor(10, 50);
        d.print(F("Save FAIL"));
        d.display();
        delay(1500);
    }
    state_ = CalibState::DONE;
}

// ── Sensor reading ──────────────────────────────────────────────────

void CalibrationMode::readSensors(Eigen::Vector3f& mag, Eigen::Vector3f& grav) {
    // Read magnetometer
    RM3100::Reading mr = magSensor_->readSingle();
    float mx, my, mz;
    magSensor_->toMicroTesla(mr, mx, my, mz);
    mag = Eigen::Vector3f(mx, my, mz);

    // Read accelerometer
    sensors_event_t accel, gyro, temp;
    imu_->getEvent(&accel, &gyro, &temp);
    grav = Eigen::Vector3f(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
}

// ── Consistency check ───────────────────────────────────────────────

bool CalibrationMode::isConsistent(const Eigen::Vector3f* buffer, int count,
                                   float threshold) const {
    // Port of Python _is_consistent(): all samples within threshold of first
    for (int i = 1; i < count; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabsf(buffer[i][j] - buffer[0][j]) > threshold) {
                return false;
            }
        }
    }
    return true;
}

Eigen::Vector3f CalibrationMode::average(const Eigen::Vector3f* buffer, int count) const {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (int i = 0; i < count; i++) sum += buffer[i];
    return sum / (float)count;
}

void CalibrationMode::recordPoint(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav) {
    magArray_.push_back(mag);
    gravArray_.push_back(grav);
}

// ── Coverage bar ────────────────────────────────────────────────────

void CalibrationMode::updateCoverageBar(const Eigen::Vector3f& grav) {
    // Map gravity vector to coverage zone
    float magnitude = grav.norm();
    if (magnitude < 0.001f) return;

    float nz = grav[2] / magnitude;
    float elevation = asinf(fmaxf(-1.0f, fminf(1.0f, nz))) * (180.0f / M_PI);
    float azimuth = fmodf(atan2f(grav[1], grav[0]) * (180.0f / M_PI) + 360.0f, 360.0f);

    int row = (int)((elevation + 90.0f) / 45.0f);
    row = max(0, min(COV_ROWS - 1, row));
    int col = (int)(azimuth / 45.0f);
    col = max(0, min(COV_COLS - 1, col));

    coverageZones_[row][col] = true;
}

// ── Display helpers ─────────────────────────────────────────────────
// All calibration screens draw directly to the OLED via getDisplay()
// to avoid the measurement-layout template from DisplayManager::refresh().

void CalibrationMode::showEllipsoidIntro() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Calibrate"));
    d.println(F("Phase 1"));

    // Instructions
    d.setTextSize(1);
    d.setCursor(0, 44);
    d.println(F("Take 56 readings in a"));
    d.println(F("diverse range of"));
    d.println(F("directions and"));
    d.println(F("orientations."));

    // Dismiss prompt
    d.setCursor(0, 110);
    d.print(F("Press any button..."));

    d.display();
}

void CalibrationMode::showAlignmentIntro() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Calibrate"));
    d.println(F("Phase 2"));

    // Instructions
    d.setTextSize(1);
    d.setCursor(0, 44);
    d.println(F("Take 3 sets of 8"));
    d.println(F("readings, barrel roll"));
    d.println(F("the laser around a"));
    d.println(F("single point for each"));
    d.println(F("set of 8."));

    // Dismiss prompt
    d.setCursor(0, 110);
    d.print(F("Press any button..."));

    d.display();
}

void CalibrationMode::showEllipsoidScreen() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Ellipsoid"));

    // Counter
    char buf[16];
    snprintf(buf, sizeof(buf), "%d/56", iteration_);
    d.setTextSize(3);
    d.setCursor(0, 30);
    d.print(buf);

    // Instruction
    d.setTextSize(1);
    d.setCursor(0, 68);
    d.print(F("Button 1 to record"));

    // Coverage bar
    showCoverageBar();

    d.display();
}

void CalibrationMode::showCoverageBar() {
    // Draw an 8-column coverage bar at bottom of display
    // Each column fills from bottom based on how many of 4 rows are covered
    // Bar area: x=8..119, y=84..111 (28 pixels tall, 4px per row + outline)
    auto& d = disp_->getDisplay();

    const int barX = 8, barY = 84, barW = 112, barH = 28;
    const int colW = barW / COV_COLS;   // 14px per column
    const int rowH = 6;                 // height per coverage row

    // Outline
    d.drawRect(barX, barY, barW, barH, SH110X_WHITE);

    // Fill columns based on coverage
    for (int c = 0; c < COV_COLS; c++) {
        int filled = 0;
        for (int r = 0; r < COV_ROWS; r++)
            if (coverageZones_[r][c]) filled++;

        if (filled > 0) {
            int fillH = filled * rowH;
            int fillY = barY + barH - 1 - fillH;
            d.fillRect(barX + 1 + c * colW, fillY, colW - 1, fillH, SH110X_WHITE);
        }

        // Column dividers
        if (c > 0) {
            d.drawFastVLine(barX + c * colW, barY, barH, SH110X_WHITE);
        }
    }

    // Print coverage to serial periodically
    if (iteration_ > 0 && iteration_ % 8 == 0) {
        int total = 0;
        for (int r = 0; r < COV_ROWS; r++)
            for (int cc = 0; cc < COV_COLS; cc++)
                if (coverageZones_[r][cc]) total++;
        Serial.print(F("Coverage: "));
        Serial.print(total);
        Serial.println(F("/32 zones"));
    }
}

void CalibrationMode::showAlignmentProgress() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Alignment"));

    int stage = (iteration_ > 0) ? ((iteration_ - 1) / 8 + 1) : 1;
    int inStage = (iteration_ > 0) ? (((iteration_ - 1) % 8) + 1) : 0;

    // Stage label
    char buf[16];
    snprintf(buf, sizeof(buf), "Stage %d", stage);
    d.setTextSize(2);
    d.setCursor(0, 30);
    d.print(buf);

    // Progress counter
    snprintf(buf, sizeof(buf), "%d/8", inStage);
    d.setTextSize(3);
    d.setCursor(0, 56);
    d.print(buf);

    // Instruction
    d.setTextSize(1);
    d.setCursor(0, 100);
    d.print(F("Button 1 to record"));

    d.display();
}

void CalibrationMode::showResultsScreen() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Results"));

    char buf[24];

    // Show both ellipsoid uniformity and alignment accuracy
    d.setTextSize(1);
    d.setCursor(0, 28);
    snprintf(buf, sizeof(buf), "Mag:  %.4f", (double)resultMagAcc_);
    d.println(buf);
    d.setCursor(0, 40);
    snprintf(buf, sizeof(buf), "Grav: %.4f", (double)resultGravAcc_);
    d.println(buf);
    d.setCursor(0, 52);
    snprintf(buf, sizeof(buf), "Acc:  %.2f deg", (double)resultAccuracy_);
    d.println(buf);

    Serial.print(F("Results — Mag: "));
    Serial.print(resultMagAcc_, 4);
    Serial.print(F("  Grav: "));
    Serial.print(resultGravAcc_, 4);
    Serial.print(F("  Accuracy: "));
    Serial.print(resultAccuracy_, 3);
    Serial.println(F(" deg"));

    // Save/discard instructions
    d.setCursor(0, 72);
    d.println(F("Hold B1+B2: Save"));
    d.setCursor(0, 84);
    d.println(F("Hold B2: Discard"));
    d.setCursor(0, 100);
    d.println(F("Lower = Better"));

    d.display();

    Serial.println(F("Hold MEASURE+DISCO to SAVE"));
    Serial.println(F("Hold DISCO to DISCARD"));
}

void CalibrationMode::showSavingScreen() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(10, 50);
    d.print(F("Saving..."));
    d.display();
}

// ── Beep control ────────────────────────────────────────────────────

void CalibrationMode::beep() {
    laser_->setBuzzer(true);
    beepActive_ = true;
    beepEndTime_ = millis() + 200;
}

void CalibrationMode::beepTriple() {
    for (int i = 0; i < 3; i++) {
        laser_->setBuzzer(true);
        delay(100);
    }
}

void CalibrationMode::updateBeep() {
    if (beepActive_ && millis() >= beepEndTime_) {
        beepActive_ = false;
        // Buzzer auto-stops on Egismos, but explicit off is harmless
    }
}

// ── Calculation ─────────────────────────────────────────────────────

void CalibrationMode::calculateEllipsoid() {
    Serial.println(F("Running ellipsoid fit..."));
    uint32_t t0 = millis();

    // Create fresh calibration with correct axis mappings
    *cal_ = MagCal::Calibration(MAG_AXES, GRAV_AXES);

    auto result = cal_->fitEllipsoid(magArray_, gravArray_);
    resultMagAcc_ = result.first;
    resultGravAcc_ = result.second;

    // Set field characteristics for anomaly detection
    cal_->setFieldCharacteristics(magArray_, gravArray_);

    uint32_t dt = millis() - t0;
    Serial.print(F("Ellipsoid fit done in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
    Serial.print(F("  Mag uniformity:  "));
    Serial.println(resultMagAcc_, 4);
    Serial.print(F("  Grav uniformity: "));
    Serial.println(resultGravAcc_, 4);
}

void CalibrationMode::calculateAlignment() {
    Serial.println(F("Running alignment fit..."));
    uint32_t t0 = millis();

    // Alignment refines existing calibration — cal_ should already be loaded

    // Find similar shots (runs of aligned readings)
    auto runs = cal_->findSimilarShots(magArray_, gravArray_);
    Serial.print(F("  Found "));
    Serial.print(runs.size());
    Serial.println(F(" aligned run(s)"));

    if (runs.empty()) {
        Serial.println(F("  ERROR: No aligned runs found!"));
        resultAccuracy_ = 999.0f;
        return;
    }

    // Build paired data from runs
    MagCal::PairedData pairedData;
    for (const auto& run : runs) {
        std::vector<Eigen::Vector3f> magSub(
            magArray_.begin() + run.first, magArray_.begin() + run.second);
        std::vector<Eigen::Vector3f> gravSub(
            gravArray_.begin() + run.first, gravArray_.begin() + run.second);
        pairedData.push_back({magSub, gravSub});
    }

    // Step 1: Fit to axis
    float accAfterAxis = cal_->fitToAxis(pairedData, 'Y');
    Serial.print(F("  After axis fit: "));
    Serial.print(accAfterAxis, 3);
    Serial.println(F(" deg"));

    // Step 2: Non-linear quick (5 params, magnetometer only)
    float accAfterNL = cal_->fitNonLinearQuick(pairedData, 5);
    Serial.print(F("  After non-linear: "));
    Serial.print(accAfterNL, 3);
    Serial.println(F(" deg"));

    // Step 3: Roll alignment
    cal_->alignSensorRoll(magArray_, gravArray_);

    // Final accuracy
    resultAccuracy_ = cal_->accuracy(pairedData);
    Serial.print(F("  Final accuracy: "));
    Serial.print(resultAccuracy_, 3);
    Serial.println(F(" deg"));

    // Update field characteristics
    cal_->setFieldCharacteristics(magArray_, gravArray_);

    uint32_t dt = millis() - t0;
    Serial.print(F("Alignment fit done in "));
    Serial.print(dt);
    Serial.println(F(" ms"));
}

bool CalibrationMode::saveCalibration() {
    // Serialize calibration to JSON
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    cal_->toJson(root);

    // Pre-check required size before serializing — serializeJson silently
    // truncates when the buffer is too small and returns sizeof(buf)-1, so
    // the naive "len >= sizeof(buf)" check never fires on overflow.
    size_t required = measureJson(doc);
    if (required == 0) {
        Serial.println(F("JSON serialization failed (empty)"));
        return false;
    }

    static constexpr size_t JSON_BUF_SIZE = 2048;
    if (required >= JSON_BUF_SIZE) {
        Serial.print(F("JSON too large: "));
        Serial.print(required);
        Serial.println(F(" bytes"));
        return false;
    }

    char jsonBuf[JSON_BUF_SIZE];
    size_t len = serializeJson(doc, jsonBuf, JSON_BUF_SIZE);
    if (len != required) {
        Serial.println(F("JSON serialization error"));
        return false;
    }

    // Save to flash
    return cfgMgr_->saveCalibrationJson(jsonBuf, len);
}
