#pragma once
// Calibration Mode — on-device calibration data collection and processing
// Session 11: port of Python calibration_manager.py + calibration_mode.py
// Session 13: combined ellipsoid + alignment into single "long calibration" flow

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <Adafruit_ISM330DHCX.h>
#include <vector>
#include "config.h"
#include "device_context.h"
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
    INTRO_ELLIPSOID,        // showing ellipsoid instruction screen
    COLLECTING_ELLIPSOID,   // 56-point ellipsoid collection
    CALCULATING_ELLIPSOID,  // running ellipsoid fitting math
    INTRO_ALIGNMENT,        // showing alignment instruction screen
    COLLECTING_ALIGNMENT,   // 24-point alignment collection (3 stages × 8)
    CALCULATING_ALIGNMENT,  // running alignment fitting math (long cal)
    CALCULATING_SHORT,      // running ellipsoid + alignment on same 24 pts (short cal)
    SHOW_RESULTS,           // displaying accuracy, waiting for save/discard
    SAVING,                 // writing calibration to flash
    FB_INTRO,               // F/B check: showing instructions
    FB_WAIT_FORESIGHT,      // F/B check: waiting for user to take foresight shot
    FB_WAIT_BACKSIGHT,      // F/B check: waiting for user to take backsight shot
    FB_PAIR_RESULT,         // F/B check: showing pair error, waiting for button press
    FB_CALCULATING,         // F/B check: running sinusoidal fit
    FB_RESULTS,             // F/B check: showing residual amplitude, save/discard
    FB_SAVING,              // F/B check: saving corrected calibration
    DONE                    // finished, caller should exit calibration mode
};

class CalibrationMode {
public:
    /// Initialize with references to all required peripherals.
    /// shortCal=true skips directly to alignment-only collection (24 points).
    void begin(ButtonManager& btns, DisplayManager& disp, DiscoManager& disco,
               LaserEgismos& laser, RM3100& magSensor, Adafruit_ISM330DHCX& imu,
               ConfigManager& cfgMgr, MagCal::Calibration& cal,
               const Config& config, bool shortCal = false);

    /// Enter foresight/backsight field check mode.
    /// Requires an existing calibration. Collects F/B pairs to correct residual
    /// hard-iron offset from the calibration environment.
    void beginFBCheck(ButtonManager& btns, DisplayManager& disp, DiscoManager& disco,
                      LaserEgismos& laser, RM3100& magSensor, Adafruit_ISM330DHCX& imu,
                      ConfigManager& cfgMgr, MagCal::Calibration& cal,
                      const Config& config);

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

    // Rolling consistency buffer (runtime-sized, max 8)
    static constexpr int MAX_BUFFER_SIZE = 8;
    Eigen::Vector3f magBuffer_[MAX_BUFFER_SIZE];
    Eigen::Vector3f gravBuffer_[MAX_BUFFER_SIZE];
    int bufferLen_ = Defaults::calBufferLength;  // runtime length from config
    float magThreshold_  = Defaults::calMagConsistency;
    float gravThreshold_ = Defaults::calGravConsistency;
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

    // ── Calibration mode ──
    bool isShortCal_ = false;

    // ── Results ──
    float resultMagAcc_  = 0.0f;
    float resultGravAcc_ = 0.0f;
    float resultAccuracy_ = 0.0f;

    // ── Timing ──
    uint32_t lastSampleTime_ = 0;
    uint32_t beepEndTime_ = 0;
    bool beepActive_ = false;

    // ── F/B check data ──
    static constexpr int FB_MAX_PAIRS = 10;
    float fbFwd_[FB_MAX_PAIRS];         // foresight bearings (degrees)
    float fbBwd_[FB_MAX_PAIRS];         // backsight bearings (degrees)
    float fbFwdSpread_[FB_MAX_PAIRS];   // max-min spread of 3 foresight legs (degrees)
    float fbBwdSpread_[FB_MAX_PAIRS];   // max-min spread of 3 backsight legs (degrees)
    int   fbCount_ = 0;                 // number of completed pairs
    bool  fbHasForesight_ = false;      // true if foresight recorded for current pair
    float fbCurrentFwd_ = 0.0f;         // current foresight bearing
    float fbCurrentFwdSpread_ = 0.0f;   // spread of current foresight legs
    float fbAmplitude_ = 0.0f;          // result: correction amplitude (degrees)

    // Bearing stability buffer (replicates SensorManager pattern)
    static constexpr int FB_STAB_LEN = 3;
    float fbStabBuf_[FB_STAB_LEN];
    int fbStabHead_ = 0;
    int fbStabCount_ = 0;

    // Leg consistency buffer (3 shots must agree to accept a bearing)
    static constexpr int FB_LEG_LEN = 3;
    float fbLegBuf_[FB_LEG_LEN];
    int fbLegCount_ = 0;

    bool fbTakingShot_ = false;         // true = red LED, waiting for stability
    bool fbLaserOn_ = true;             // tracks laser state during FB collection

    // Config values for FB stability/leg checks
    float fbStabilityTol_ = 0.4f;
    float fbLegAngleTol_ = 1.7f;

    // Display refresh timing
    uint32_t fbLastDisplayTime_ = 0;
    float fbCurrentBearing_ = 0.0f;     // latest computed bearing for live display

    // ── State handlers ──
    void updateIntro();
    void updateCollecting();
    void updateCalculatingEllipsoid();
    void updateCalculatingAlignment();
    void updateCalculatingShort();
    void updateShowResults();
    void updateSaving();
    void updateFBIntro();
    void updateFBWaitShot();
    void updateFBPairResult();
    void updateFBCalculating();
    void updateFBResults();
    void updateFBSaving();

    // ── Helpers ──
    void readSensors(Eigen::Vector3f& mag, Eigen::Vector3f& grav);
    bool isConsistent(const Eigen::Vector3f* buffer, int count, float threshold) const;
    Eigen::Vector3f average(const Eigen::Vector3f* buffer, int count) const;
    void recordPoint(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav);
    void updateCoverageBar(const Eigen::Vector3f& grav);
    float getBearing(const Eigen::Vector3f& mag, const Eigen::Vector3f& grav);
    bool fbBearingStable(float tolerance) const;
    float fbCircularAverage(const float* buf, int count) const;
    static float circularDiff(float a, float b);

    // ── Display helpers ──
    void showEllipsoidIntro();
    void showAlignmentIntro();
    void showEllipsoidScreen();
    void showAlignmentProgress();
    void showCoverageBar();
    void showResultsScreen();
    void showSavingScreen();
    void showFBIntroScreen();
    void showFBLiveScreen();
    void showFBPairResult(float error, float bearing);
    void showFBResultsScreen();

    // ── Beep control ──
    void beep();
    void beepTriple();
    void updateBeep();

    // ── Calculation ──
    void calculateEllipsoid();
    void calculateAlignment();
    bool saveCalibration();
};
