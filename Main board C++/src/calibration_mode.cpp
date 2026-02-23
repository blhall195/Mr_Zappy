#include "calibration_mode.h"
#include <math.h>

// ── Initialization ──────────────────────────────────────────────────

void CalibrationMode::begin(
    ButtonManager& btns, DisplayManager& disp, DiscoManager& disco,
    LaserEgismos& laser, RM3100& magSensor, Adafruit_ISM330DHCX& imu,
    ConfigManager& cfgMgr, MagCal::Calibration& cal)
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

    memset(zoneCounts_, 0, sizeof(zoneCounts_));

    // Turn on laser for calibration
    laser_->setLaser(true);

    state_ = CalibState::CHOOSING;
    showChoiceScreen();

    Serial.println(F("Calibration mode entered."));
    Serial.println(F("  MEASURE = Ellipsoid (auto)"));
    Serial.println(F("  DISCO   = Alignment (24 pts)"));
    Serial.println(F("  SHUTDOWN = power off"));
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
        case CalibState::CHOOSING:            updateChoosing(); break;
        case CalibState::COLLECTING_ELLIPSOID:
        case CalibState::COLLECTING_ALIGNMENT: updateCollecting(); break;
        case CalibState::CALCULATING:         updateCalculating(); break;
        case CalibState::SHOW_RESULTS:        updateShowResults(); break;
        case CalibState::SAVING:              updateSaving(); break;
        default: break;
    }

    return (state_ == CalibState::DONE);
}

// ── State: CHOOSING ─────────────────────────────────────────────────

void CalibrationMode::updateChoosing() {
    if (btns_->wasPressed(Button::MEASURE)) {
        // Ellipsoid calibration — continuous auto-collection
        isEllipsoidMode_ = true;
        iteration_ = 0;
        magArray_.clear();
        gravArray_.clear();
        magArray_.reserve(maxCount_);
        gravArray_.reserve(maxCount_);
        bufferCount_ = 0;
        bufferIdx_ = 0;
        memset(zoneCounts_, 0, sizeof(zoneCounts_));

        state_ = CalibState::COLLECTING_ELLIPSOID;
        showEllipsoidScreen();
        Serial.println(F("Starting ellipsoid calibration (continuous collection)..."));
    }
    else if (btns_->wasPressed(Button::DISCO)) {
        // Alignment calibration
        isEllipsoidMode_ = false;
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
        Serial.println(F("Starting alignment calibration (24 points: 3 dirs x 8 rotations)..."));
    }
}

// ── State: COLLECTING ───────────────────────────────────────────────

void CalibrationMode::updateCollecting() {
    if (state_ == CalibState::COLLECTING_ELLIPSOID) {
        updateCollectingEllipsoid();
    } else {
        updateCollectingAlignment();
    }
}

// ── Ellipsoid: continuous auto-collection ───────────────────────────

void CalibrationMode::updateCollectingEllipsoid() {
    // Sample at ~30 Hz (matches RM3100 cycle time of ~14ms at CC=400)
    uint32_t now = millis();
    if (now - lastSampleTime_ < 30) return;
    lastSampleTime_ = now;

    // Read raw sensor data
    Eigen::Vector3f magReading, gravReading;
    readSensors(magReading, gravReading);

    // Push into rolling circular buffer
    magBuffer_[bufferIdx_] = magReading;
    gravBuffer_[bufferIdx_] = gravReading;
    bufferIdx_ = (bufferIdx_ + 1) % BUFFER_SIZE;
    if (bufferCount_ < BUFFER_SIZE) bufferCount_++;

    // Button 1 = early finish (only if minimum coverage met)
    if (btns_->wasPressed(Button::MEASURE)) {
        int covered = coveredZoneCount();
        if (covered >= EARLY_FINISH_ZONES && iteration_ >= EARLY_FINISH_POINTS) {
            Serial.print(F("Early finish: "));
            Serial.print(iteration_);
            Serial.print(F(" pts, "));
            Serial.print(covered);
            Serial.println(F("/32 zones"));
            beep();
            state_ = CalibState::CALCULATING;
            return;
        }
        // Not enough coverage — flash red to indicate "keep going"
        disco_->setRed();
    }

    // Need full buffer for stability check
    if (bufferCount_ < BUFFER_SIZE) return;

    // Stability gate (relaxed thresholds for slow rotation)
    bool magStable = isConsistent(magBuffer_, BUFFER_SIZE, ELLIPSOID_MAG_THRESH);
    bool gravStable = isConsistent(gravBuffer_, BUFFER_SIZE, ELLIPSOID_GRAV_THRESH);

    // While purple flash is active, don't change LED
    bool flashActive = (now < purpleFlashEnd_);

    if (!magStable || !gravStable) {
        // Moving too fast — red (unless mid-flash)
        if (!flashActive) disco_->setRed();
        return;
    }

    // Average the buffer
    Eigen::Vector3f avgMag = average(magBuffer_, BUFFER_SIZE);
    Eigen::Vector3f avgGrav = average(gravBuffer_, BUFFER_SIZE);

    // Novelty gate — only record if this orientation is new
    if (!isNovel(avgMag)) {
        // Stable but already-covered region — green (unless mid-flash)
        if (!flashActive) disco_->setGreen();
        return;
    }

    // Record this point
    recordPoint(avgMag, avgGrav);
    updateCoverageZone(avgGrav);
    iteration_++;

    // Novel reading recorded — purple flash for 300ms
    disco_->setPurple();
    purpleFlashEnd_ = now + 300;

    // Update display every point (counter + coverage bar)
    showEllipsoidScreen();

    // Check auto-complete
    int covered = coveredZoneCount();
    if ((covered >= AUTO_COMPLETE_ZONES && iteration_ >= AUTO_COMPLETE_POINTS)
        || iteration_ >= maxCount_) {
        Serial.print(F("Auto-complete: "));
        Serial.print(iteration_);
        Serial.print(F(" pts, "));
        Serial.print(covered);
        Serial.println(F("/32 zones"));
        beep();
        state_ = CalibState::CALCULATING;
        return;
    }

    // Log every 10 points
    if (iteration_ % 10 == 0) {
        Serial.print(F("Ellipsoid: "));
        Serial.print(iteration_);
        Serial.print(F(" pts, "));
        Serial.print(covered);
        Serial.println(F("/32 zones"));
    }
}

// ── Alignment: manual button-press collection (unchanged) ───────────

void CalibrationMode::updateCollectingAlignment() {
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
        bool magConsistent = isConsistent(magBuffer_, BUFFER_SIZE, 0.4f);
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

            showAlignmentProgress();

            // Triple beep at direction changes (every 8 points)
            if (iteration_ % 8 == 0 && iteration_ < targetCount_) {
                beepTriple();
                Serial.println(F("Change direction..."));
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
                state_ = CalibState::CALCULATING;
            }
        }
    }
}

// ── State: CALCULATING ──────────────────────────────────────────────

void CalibrationMode::updateCalculating() {
    // Show calculating message
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);
    d.setTextSize(2);
    d.setCursor(0, 50);
    d.print(F("Calculating..."));
    d.display();

    if (isEllipsoidMode_) {
        calculateEllipsoid();
    } else {
        calculateAlignment();
    }

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

void CalibrationMode::updateCoverageZone(const Eigen::Vector3f& grav) {
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

    zoneCounts_[row][col]++;
}

bool CalibrationMode::isNovel(const Eigen::Vector3f& mag) const {
    // Check if this magnetometer reading is sufficiently different from all
    // previously recorded points (angular distance in degrees)
    float magNorm = mag.norm();
    if (magNorm < 0.001f) return false;

    Eigen::Vector3f magUnit = mag / magNorm;

    for (const auto& existing : magArray_) {
        float existNorm = existing.norm();
        if (existNorm < 0.001f) continue;
        Eigen::Vector3f existUnit = existing / existNorm;
        float dot = magUnit.dot(existUnit);
        dot = fmaxf(-1.0f, fminf(1.0f, dot));
        float angleDeg = acosf(dot) * (180.0f / M_PI);
        if (angleDeg < MIN_ANGULAR_DIST) {
            return false;
        }
    }
    return true;
}

int CalibrationMode::coveredZoneCount() const {
    int count = 0;
    for (int r = 0; r < COV_ROWS; r++) {
        for (int c = 0; c < COV_COLS; c++) {
            if (zoneCounts_[r][c] >= ZONE_COVERED_THRESHOLD) {
                count++;
            }
        }
    }
    return count;
}

// ── Display helpers ─────────────────────────────────────────────────
// All calibration screens draw directly to the OLED via getDisplay()
// to avoid the measurement-layout template from DisplayManager::refresh().

void CalibrationMode::showChoiceScreen() {
    auto& d = disp_->getDisplay();
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    // Title
    d.setTextSize(2);
    d.setCursor(0, 0);
    d.println(F("Calibrate"));

    // Option 1
    d.setCursor(0, 30);
    d.setTextSize(1);
    d.print(F("Button 1:"));
    d.setCursor(0, 44);
    d.setTextSize(2);
    d.print(F("Ellipsoid"));

    // Option 2
    d.setCursor(0, 72);
    d.setTextSize(1);
    d.print(F("Button 2:"));
    d.setCursor(0, 86);
    d.setTextSize(2);
    d.print(F("Alignment"));

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
    snprintf(buf, sizeof(buf), "%d pts", iteration_);
    d.setTextSize(3);
    d.setCursor(0, 30);
    d.print(buf);

    // Instruction
    d.setTextSize(1);
    d.setCursor(0, 68);
    d.print(F("Rotate slowly..."));

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
            if (zoneCounts_[r][c]) filled++;

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
                if (zoneCounts_[r][cc]) total++;
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
    if (isEllipsoidMode_) {
        d.setTextSize(1);
        d.setCursor(0, 28);
        snprintf(buf, sizeof(buf), "Mag:  %.4f", (double)resultMagAcc_);
        d.println(buf);
        d.setCursor(0, 40);
        snprintf(buf, sizeof(buf), "Grav: %.4f", (double)resultGravAcc_);
        d.println(buf);

        Serial.print(F("Ellipsoid result — Mag: "));
        Serial.print(resultMagAcc_, 4);
        Serial.print(F("  Grav: "));
        Serial.println(resultGravAcc_, 4);
    } else {
        d.setTextSize(1);
        d.setCursor(0, 28);
        d.print(F("Accuracy:"));
        d.setTextSize(2);
        d.setCursor(0, 42);
        snprintf(buf, sizeof(buf), "%.2f deg", (double)resultAccuracy_);
        d.print(buf);

        Serial.print(F("Alignment accuracy: "));
        Serial.print(resultAccuracy_, 3);
        Serial.println(F(" deg (target < 1.0)"));
    }

    // Save/discard instructions
    d.setTextSize(1);
    d.setCursor(0, 80);
    d.println(F("Hold B1+B2: Save"));
    d.setCursor(0, 92);
    d.println(F("Hold B2: Discard"));
    d.setCursor(0, 108);
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

    // Serialize to string
    char jsonBuf[1024];
    size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (len == 0 || len >= sizeof(jsonBuf)) {
        Serial.println(F("JSON serialization failed"));
        return false;
    }

    // Save to flash
    return cfgMgr_->saveCalibrationJson(jsonBuf, len);
}
