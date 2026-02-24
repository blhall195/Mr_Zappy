#pragma once
// Calibration Mode — on-device calibration data collection and processing
// Session 11: port of Python calibration_manager.py + calibration_mode.py

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <Adafruit_ISM330DHCX.h>
#include <vector>
#include "config.h"
#include "button_manager.h"
#include "display_manager.h"
#include "disco_manager.h"
#include "laser_egismos.h"
#include "rm3100.h"
#include "config_manager.h"
#include "mag_cal/calibration.h"

/// Calibration state machine states
enum class CalibState : uint8_t {
    INACTIVE,               // not in calibration mode
    CHOOSING,               // waiting for user to pick ellipsoid or alignment
    COLLECTING_ELLIPSOID,   // 56-point ellipsoid collection
    COLLECTING_ALIGNMENT,   // 24-point alignment collection (3 stages × 8)
    CALCULATING,            // running fitting math
    SHOW_RESULTS,           // displaying accuracy, waiting for save/discard
    SAVING,                 // writing calibration to flash
    DONE                    // finished, caller should exit calibration mode
};

class CalibrationMode {
public:
    /// Initialize with references to all required peripherals.
    void begin(ButtonManager& btns, DisplayManager& disp, DiscoManager& disco,
               LaserEgismos& laser, RM3100& magSensor, Adafruit_ISM330DHCX& imu,
               ConfigManager& cfgMgr, MagCal::Calibration& cal);

    /// Call every loop iteration. Returns true when calibration is complete.
    bool update();

    bool isActive() const { return state_ != CalibState::INACTIVE && state_ != CalibState::DONE; }
    CalibState state() const { return state_; }

private:
    // ── Peripheral references (not owned) ──
    ButtonManager*       btns_     = nullptr;
    DisplayManager*      disp_     = nullptr;
    DiscoManager*        disco_    = nullptr;
    LaserEgismos*        laser_    = nullptr;
    RM3100*              magSensor_= nullptr;
    Adafruit_ISM330DHCX* imu_      = nullptr;
    ConfigManager*       cfgMgr_   = nullptr;
    MagCal::Calibration* cal_      = nullptr;

    // ── State machine ──
    CalibState state_ = CalibState::INACTIVE;

    // ── Data collection ──
    std::vector<Eigen::Vector3f> magArray_;
    std::vector<Eigen::Vector3f> gravArray_;

    // Rolling 8-sample consistency buffer
    static constexpr int BUFFER_SIZE = 8;
    Eigen::Vector3f magBuffer_[BUFFER_SIZE];
    Eigen::Vector3f gravBuffer_[BUFFER_SIZE];
    int bufferCount_ = 0;
    int bufferIdx_   = 0;   // circular write index

    bool waitingForStable_ = false;
    int iteration_ = 0;
    int targetCount_ = 56;  // 56 for ellipsoid, 24 for alignment

    // ── Coverage bar (ellipsoid only) ──
    static constexpr int COV_COLS = 8;
    static constexpr int COV_ROWS = 4;
    bool coverageZones_[COV_ROWS][COV_COLS];

    // ── Save/discard hold detection ──
    float holdCounter_ = 0.0f;
    static constexpr float HOLD_TIME = 0.5f;  // seconds to hold for save/discard

    // ── Results ──
    float resultMagAcc_  = 0.0f;
    float resultGravAcc_ = 0.0f;
    float resultAccuracy_ = 0.0f;
    bool isEllipsoidMode_ = true;

    // ── Timing ──
    uint32_t lastSampleTime_ = 0;
    uint32_t beepEndTime_ = 0;
    bool beepActive_ = false;

    // ── State handlers ──
    void updateChoosing();
    void updateCollecting();
    void updateCalculating();
    void updateShowResults();
    void updateSaving();

    // ── Helpers ──
    void readSensors(Eigen::Vector3f& mag, Eigen::Vector3f& grav);
    bool isConsistent(const Eigen::Vector3f* buffer, int count, float threshold) const;
    Eigen::Vector3f average(const Eigen::Vector3f* buffer, int count) const;
    void recordPoint(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav);
    void updateCoverageBar(const Eigen::Vector3f& grav);

    // ── Display helpers ──
    void showChoiceScreen();
    void showEllipsoidScreen();
    void showAlignmentProgress();
    void showCoverageBar();
    void showResultsScreen();
    void showSavingScreen();

    // ── Beep control ──
    void beep();
    void beepTriple();
    void updateBeep();

    // ── Calculation ──
    void calculateEllipsoid();
    void calculateAlignment();
    bool saveCalibration();
};
