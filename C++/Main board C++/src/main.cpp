#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_MAX1704X.h>
#include "config.h"
#include "device_context.h"
#include "button_manager.h"
#include "rm3100.h"
#include "display_manager.h"
#include "laser_egismos.h"
#include "ble_manager.h"
#include "mag_cal/calibration.h"
#include "sensor_manager.h"
#include "config_manager.h"
#include "disco_manager.h"
#include "menu_manager.h"
#include "calibration_mode.h"
#include "snake_game.h"
#include <Adafruit_TinyUSB.h>
#include <Adafruit_SPIFlash.h>

// ── Embedded calibration data ──────────────────────────────────────
// From Main board/calibration_dict.json — loaded at startup
static const char CALIBRATION_JSON[] PROGMEM = R"({
  "mag": {
    "axes": "-X-Y-Z",
    "transform": [[0.0225258, -0.000348105, -0.000709316],
                   [0.000171691, 0.0221358, -0.000679377],
                   [0.000447533, -0.000145672, 0.0225338]],
    "centre": [-0.205928, -1.49322, 3.39001],
    "rbfs": [[[0.003298], [-0.000433647], [0.000818552], [-0.00208113], [0.00252497]],
             [[0.0], [0.0], [0.0], [0.0], [0.0]],
             [[0.00067137], [0.000540657], [0.000939099], [0.00137892], [0.00192191]]],
    "field_avg": 44.6264,
    "field_std": 0.310694
  },
  "dip_avg": 68.9221,
  "grav": {
    "axes": "-Y-X+Z",
    "transform": [[0.101936, -0.00167875, 0.000143624],
                   [0.0015862, 0.10164, 0.000604115],
                   [0.000153489, -0.000228592, 0.102199]],
    "centre": [0.235023, 0.0249322, 0.100026],
    "rbfs": [],
    "field_avg": 9.81022,
    "field_std": 0.00509215
  }
})";

// ── MAX17048 subclass: skip reset() to preserve ModelGauge state ────
// The stock begin() calls reset() which wipes the IC's learned battery
// model, forcing a voltage-only "first guess" every boot.  By skipping
// the reset the IC keeps tracking SOC across reboots (it still PORs on
// its own if battery power is lost).
class MAX17048_Persistent : public Adafruit_MAX17048 {
public:
    bool begin(TwoWire *wire = &Wire) {
        if (i2c_dev) { delete i2c_dev; delete status_reg; }
        i2c_dev = new Adafruit_I2CDevice(MAX17048_I2CADDR_DEFAULT, wire);
        if (!i2c_dev->begin())   return false;
        if (!isDeviceReady())    return false;
        status_reg = new Adafruit_BusIO_Register(i2c_dev, MAX1704X_STATUS_REG);
        // No reset() — preserve ModelGauge tracking state
        enableSleep(false);
        sleep(false);
        wake();   // exit hibernation if the IC entered it
        return true;
    }
};

// ── Global state ────────────────────────────────────────────────────
DeviceContext       ctx;
ButtonManager       buttons;
RM3100              mag;
Adafruit_ISM330DHCX imu;
MAX17048_Persistent battery;
DisplayManager      display;
LaserEgismos        laser;
BleManager          ble;
MagCal::Calibration calibration;
SensorManager       sensorMgr;
ConfigManager       configMgr;
DiscoManager        disco;
MenuManager         menuMgr;
CalibrationMode     calMode;
SnakeGame           snakeGame;
Adafruit_USBD_MSC   usbMsc;

bool magOk   = false;
bool imuOk   = false;
bool batOk   = false;
bool dispOk  = false;
bool laserOk = false;
bool bleOk   = false;
bool calOk   = false;
bool flashOk = false;
static bool enterMenuMode  = false;
static bool enterCalibMode = false;
static bool enterSnakeMode = false;

// Latest sensor readings
static float lastMagX = 0, lastMagY = 0, lastMagZ = 0;
static float lastAccX = 0, lastAccY = 0, lastAccZ = 0;

static float lastDistance = 0;

// ── Display deadband (prevents ±0.1 flicker when stationary) ────────
static float dispAz   = 0.0f;   // currently displayed azimuth
static float dispInc  = 0.0f;   // currently displayed inclination
static constexpr float DEADBAND_ANGLE = 0.15f;  // degrees

// ── Timing statics ──────────────────────────────────────────────────
static uint32_t lastSensorUpdate    = 0;
static uint32_t lastBatRead         = 0;
static float    lastBatPct          = 0.0f;
static uint32_t lastDisplayRefresh  = 0;
static uint32_t lastBleCheck        = 0;
static uint32_t lastBleUartPoll     = 0;
static uint32_t lastAutoShutCheck   = 0;
static uint32_t lastLaserTimeCheck  = 0;

// ── BLE startup name sync retry ────────────────────────────────────
static uint32_t bleNameSyncStartMs      = 0;
static uint32_t bleNameSyncLastSendMs   = 0;
static uint8_t  bleNameSyncAttempts     = 0;
static bool     bleNameSyncDone         = false;
static bool     bleReadySeen            = false;
static bool     bleNameSentAfterReady   = false;

// ── Button hold detection ───────────────────────────────────────────
static uint32_t discoHoldStart   = 0;
static bool     discoHolding     = false;
static bool     discoTriggered   = false;  // true once disco toggled during this hold
static uint32_t calibHoldStart = 0;
static bool     calibHolding   = false;

// ── Measurement state ───────────────────────────────────────────────
static bool measRedLedSet = false;

// ── BLE state ───────────────────────────────────────────────────────
static bool lastBleConnected = false;

// ── Forward declarations — init ────────────────────────────────────
static void initPins();
static void scanI2C();
// initSensors() inlined into setup()
static void initDisplay();
static void initLaser();
static void initBle();
static void initFlash();
static void initCalibration();
static void initDisco();

// ── Forward declarations — polling ──────────────────────────────────
static void readSensorsUpdate(uint32_t now);
static void pollButtons(uint32_t now);
static void pollMeasurement(uint32_t now);
static void pollBLEPin(uint32_t now);
static void pollBLECommands(uint32_t now);
static void pollBLEStartupNameSync(uint32_t now);
static void pollBattery(uint32_t now);
static void checkAutoShutoff(uint32_t now);
static void checkLaserTimeout(uint32_t now);
static void updateDisplay(uint32_t now);

// ── Forward declarations — helpers ──────────────────────────────────
static void handleMeasurementSuccess();
static void alertError(const char* errCode);
static void resetLaser();
static void doShutdown();
static float circularDiff(float a, float b);
static bool bearingsWithinTol(const float* vals, uint8_t n, float tol);
static bool linearWithinTol(const float* vals, uint8_t n, float tol);
static void onFlushReading(float az, float inc, float dist);
static void enterUsbDriveMode();  // USB MSC loop (does not return)

// ── USB MSC block-level callbacks ───────────────────────────────────
// Defined here (before setup) so the early USB drive mode check can use them.
static Adafruit_SPIFlash* s_mscFlash = nullptr;

static int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
    return s_mscFlash->readBlocks(lba, (uint8_t*)buffer, bufsize / 512)
           ? (int32_t)bufsize : -1;
}

static int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
    return s_mscFlash->writeBlocks(lba, buffer, bufsize / 512)
           ? (int32_t)bufsize : -1;
}

static void msc_flush_cb() {
    s_mscFlash->syncBlocks();
}

// ═══════════════════════════════════════════════════════════════════
// ── Setup ─────────────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════
void setup() {
    // Kill NeoPixel immediately to prevent green flash on boot
    pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
    digitalWrite(PIN_NEOPIXEL_POWER, LOW);

    // Get display up ASAP — before serial, which can block on USB enumeration
    Wire.begin();
    Wire.setClock(400000);
    initDisplay();

    // Configure button pull-up BEFORE reading it
    pinMode(PIN_BTN_MEASURE, INPUT_PULLUP);

    // ── Early USB drive mode check ─────────────────────────────────
    // Must happen BEFORE Serial.begin() so MSC is part of the initial
    // USB enumeration — otherwise the host won't see the mass storage
    // device without a cable replug.
    // Check for usb_drive flag BEFORE showing splash so the user sees
    // "Entering USB mode..." instead of the normal splash screen.
    {
        bool earlyFlash = configMgr.begin();  // safe before Serial — prints are no-ops
        if (earlyFlash && configMgr.hasFlag("usb_drive")) {
            configMgr.clearFlag("usb_drive");

            // Show transitional message while USB stack initialises
            if (dispOk) {
                auto& d = display.getDisplay();
                d.clearDisplay();
                d.setTextSize(1);
                d.setTextColor(SH110X_WHITE);
                d.setCursor(0, 56);
                d.println(F("Entering USB mode..."));
                d.display();
            }

            // Register MSC interface BEFORE USB stack enumerates
            s_mscFlash = configMgr.getFlash();
            if (s_mscFlash) {
                usbMsc.setID("Mr_Zappy", "Settings", "1.0");
                usbMsc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
                usbMsc.setCapacity(s_mscFlash->size() / 512, 512);
                usbMsc.setUnitReady(true);
                usbMsc.begin();
            }

            Serial.begin(115200);
            Serial.println(F("=== USB Drive Mode (early init) ==="));

            // Show final instructions
            if (dispOk) {
                auto& d = display.getDisplay();
                d.clearDisplay();
                d.setTextSize(1);
                d.setTextColor(SH110X_WHITE);
                d.setCursor(0, 10);
                d.println(F("USB Drive Mode"));
                d.println();
                d.println(F("Edit config.json"));
                d.println(F("on the USB drive."));
                d.println();
                d.println(F("Eject drive, then"));
                d.println(F("hold power button"));
                d.println(F("to restart."));
                d.display();
            }

            // Spin forever — device stays in USB drive mode until power cycled
            while (true) { delay(100); }
            // Does not return
        }
    }

    // Normal boot — show splash screen now that we know we're not in USB mode
    uint32_t splashStart = millis();
    display.showSplash(false);

    Serial.begin(115200);
    if (digitalRead(PIN_BTN_MEASURE) == LOW) {   // hold MEASURE for serial debug
        while (!Serial && millis() < 3000) { }
        Serial.println(F("I2C bus scan:"));
        scanI2C();
        Serial.println();
    }

    Serial.println(F("=== Mr_Zappy Main Board C++ ==="));
    Serial.println(F("Session 12 — Full Integration"));
    Serial.println();

    initPins();
    buttons.begin();

    // ── Sensors ──
    magOk = mag.begin(Wire, RM3100_I2C_ADDR, RM3100_CYCLE_COUNT, PIN_MAG_DRDY);
    Serial.print(F("RM3100:      "));
    Serial.println(magOk ? F("OK") : F("FAILED"));

    imuOk = imu.begin_I2C(ISM330DHCX_ADDR, &Wire);
    if (imuOk) {
        imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
        imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
        imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
        imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    }
    Serial.print(F("ISM330DHCX:  "));
    Serial.println(imuOk ? F("OK") : F("FAILED"));

    batOk = battery.begin(&Wire);
    if (batOk) {
        delay(100);  // let SOC register update after wake
        lastBatPct = battery.cellPercent();
        lastBatRead = millis();
    }
    Serial.print(F("MAX17048:    "));
    Serial.println(batOk ? F("OK") : F("FAILED"));
    if (batOk) {
        Serial.print(F("  Voltage:   ")); Serial.print(battery.cellVoltage(), 3); Serial.println(F(" V"));
        Serial.print(F("  SOC:       ")); Serial.print(lastBatPct, 1); Serial.println(F(" %"));
        Serial.print(F("  Hibernate: ")); Serial.println(battery.isHibernating() ? F("yes") : F("no"));
    }
    Serial.println();

    initLaser();
    initBle();
    initFlash();
    initCalibration();
    initDisco();

    // ── Splash sequence: off(200) → on(300) → off(200) → on(750) = 1450ms
    {
        auto splashLaser = [&]() -> bool {
            uint32_t t = millis() - splashStart;
            if (t < 200)  return false;  // off  200ms
            if (t < 500)  return true;   // on   300ms
            if (t < 700)  return false;  // off  200ms
            return true;                 // on   750ms (final hold)
        };
        // Extract suffix after '_' from BLE name for splash display
        const char* nameSuffix = strchr(ctx.config.bleName, '_');
        if (nameSuffix) nameSuffix++;   // skip the '_'

        while (millis() - splashStart < 1450) {
            display.showSplash(splashLaser(), nameSuffix);
        }
    }

    // ── Switch display from splash to main screen ────────────────
    if (dispOk) {
        display.updateBattery(lastBatPct);
        display.initScreen();
    }

    // ── Startup: turn laser on with beep ───────────────────────────
    if (laserOk) {
        laser.setLaser(true);
        laser.setBuzzer(true);   // startup beep
        laser.setBuzzer(false);
        ctx.laserEnabled = true;
    }

    // ── Low battery check on boot ────────────────────────────────
    // Skip if 0% — that means no LiPo connected (USB-only power)
    if (batOk && lastBatPct > 0.5f && lastBatPct <= Timing::BATTERY_SHUTDOWN_PCT) {
        Serial.println(F("LOW BATTERY — shutting down"));
        if (dispOk) {
            display.updateDistanceText("LOW");
            display.updateAzimuth(0);
            display.updateInclination(0);
            display.refresh();
        }
        disco.setRed();
        delay(3000);
        doShutdown();
    }

    // ── Pending readings display ─────────────────────────────────
    if (dispOk) {
        display.updateBTNumber(ctx.bleDisconnectionCounter);
    }

    // ── BLE device name sync is retried from loop() to avoid boot races ──
    bleNameSyncStartMs = millis();

    // ── Check for boot mode overrides ────────────────────────────
    buttons.update();
    if (buttons.isPressed(Button::CALIB)) {
        Serial.println(F("CALIB held at boot — entering menu mode"));
        enterMenuMode = true;
    }

    // Enter menu mode if flag was set or CALIB held
    if (enterMenuMode && dispOk) {
        display.showStartingMenu();
        delay(500);
        menuMgr.begin(display.getDisplay(), ctx, configMgr);
    }

    // Enter calibration mode if flag was set
    if (enterCalibMode && magOk && imuOk && dispOk && laserOk) {
        calMode.begin(buttons, display, disco, laser, mag, imu,
                      configMgr, calibration);
    }

    // Enter snake mode if flag was set
    if (enterSnakeMode && dispOk) {
        if (laserOk) laser.setLaser(false);
        snakeGame.begin(display.getDisplay(), buttons, disco);
    }

    ctx.lastActivityTime = millis();

    if (snakeGame.isActive()) {
        Serial.println(F("Running. Snake mode."));
    } else if (calMode.isActive()) {
        Serial.println(F("Running. Calibration mode."));
    } else if (menuMgr.isActive()) {
        Serial.println(F("Running. Menu mode."));
    } else {
        Serial.println(F("Running. Normal mode."));
    }
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// ── Main Loop ─────────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    buttons.update();

    // Retries NAME sync after boot so BLE board has time to finish setup.
    pollBLEStartupNameSync(now);

    // ── Snake mode: SnakeGame owns the loop ──
    if (snakeGame.isActive()) {
        if (snakeGame.update()) {
            // Game finished — return to normal operation
            Serial.println(F("Snake game ended — returning to normal mode"));
            disco.turnOff();
            display.initScreen();
            if (laserOk) {
                laser.setLaser(true);
                delay(25);
                laser.setBuzzer(true);
                delay(100);
                laser.setBuzzer(false);
                delay(25);
            }
            ctx.laserEnabled = true;
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // ── Menu mode: hand off to MenuManager, skip everything else ──
    if (menuMgr.isActive()) {
        menuMgr.update(buttons);
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // ── Handle menu exit transitions (one-shot) ──
    if (menuMgr.exitAction() == MenuExitAction::ENTER_LONG_CALIB ||
        menuMgr.exitAction() == MenuExitAction::ENTER_SHORT_CALIB) {
        bool shortCal = (menuMgr.exitAction() == MenuExitAction::ENTER_SHORT_CALIB);
        menuMgr.clearExitAction();
        if (magOk && imuOk && dispOk && laserOk) {
            Serial.println(F("Transitioning: menu → calibration"));
            calMode.begin(buttons, display, disco, laser, mag, imu,
                          configMgr, calibration, shortCal);
        } else {
            Serial.println(F("Cannot enter calibration — sensors not ready"));
            display.initScreen();
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }
    if (menuMgr.exitAction() == MenuExitAction::ENTER_SNAKE) {
        menuMgr.clearExitAction();
        if (dispOk) {
            Serial.println(F("Transitioning: menu → snake game"));
            if (laserOk) laser.setLaser(false);
            ctx.laserEnabled = false;
            snakeGame.begin(display.getDisplay(), buttons, disco);
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }
    if (menuMgr.exitAction() == MenuExitAction::ENTER_BOOTLOADER) {
        menuMgr.clearExitAction();
        Serial.println(F("Entering UF2 bootloader..."));
        if (dispOk) {
            auto& d = display.getDisplay();
            d.clearDisplay();
            d.setTextSize(1);
            d.setTextColor(SH110X_WHITE);
            d.setCursor(0, 10);
            d.println(F("Entered bootloader"));
            d.println();
            d.println(F("Plug device into PC"));
            d.println(F("and flash new"));
            d.println(F("firmware (.uf2)"));
            d.println();
            d.println(F("Load new firmware or"));
            d.println(F("hold power button"));
            d.println(F("to exit."));
            d.display();
            delay(3000);
        }
        // Write UF2 bootloader magic to end of RAM and reset
        // SAME51J19A: 192KB RAM at 0x20000000, so end-4 = 0x2002FFFC
        #define BOOT_DOUBLE_TAP_ADDRESS  (0x2002FFFCul)
        *((volatile uint32_t *)BOOT_DOUBLE_TAP_ADDRESS) = 0xf01669efUL;
        NVIC_SystemReset();
        // Does not return
    }
    if (menuMgr.exitAction() == MenuExitAction::ENTER_USB_DRIVE) {
        menuMgr.clearExitAction();
        Serial.println(F("Entering USB drive mode..."));
        if (dispOk) {
            auto& d = display.getDisplay();
            d.clearDisplay();
            d.setTextSize(1);
            d.setTextColor(SH110X_WHITE);
            d.setCursor(0, 56);
            d.println(F("Entering USB mode..."));
            d.display();
        }
        // Set flag file so USB MSC mode starts after reboot
        configMgr.writeFlag("usb_drive");
        delay(1500);
        NVIC_SystemReset();
        // Does not return
    }
    if (menuMgr.exitAction() == MenuExitAction::RETURN_NORMAL) {
        menuMgr.clearExitAction();
        Serial.println(F("Returning to normal mode"));
        display.initScreen();
        if (laserOk) {
            laser.setLaser(true);
            delay(25);
            laser.setBuzzer(true);
            delay(100);
            laser.setBuzzer(false);
            delay(25);
        }
        ctx.laserEnabled = true;
        ctx.lastActivityTime = millis();
    }

    // ── Calibration mode: CalibrationMode owns the loop ──
    if (calMode.isActive()) {
        bool done = calMode.update();
        if (done) {
            Serial.println(F("Calibration mode finished."));
            // Drain buttons held during save/discard (DISCO or MEASURE+DISCO)
            // to prevent pollButtons() treating the release as a disco toggle.
            while (buttons.isPressed(Button::DISCO) ||
                   buttons.isPressed(Button::MEASURE)) {
                buttons.update();
                delay(Timing::LOOP_INTERVAL_MS);
            }
            calOk = calibration.isCalibrated();
            if (calOk) {
                sensorMgr.init(&calibration, ctx.config.emaAlpha,
                               ctx.config.stabilityBufferLength);
            }
            display.initScreen();
            if (laserOk) {
                laser.setLaser(true);
                delay(25);
                laser.setBuzzer(true);
                delay(100);
                laser.setBuzzer(false);
                delay(25);
            }
            ctx.laserEnabled = true;
            ctx.lastActivityTime = millis();
        }
        delay(Timing::LOOP_INTERVAL_MS);
        return;
    }

    // ── Normal operation: cooperative polling ─────────────────────
    readSensorsUpdate(now);
    pollButtons(now);
    pollMeasurement(now);
    pollBLEPin(now);
    pollBLECommands(now);
    pollBattery(now);
    checkAutoShutoff(now);
    checkLaserTimeout(now);
    updateDisplay(now);
    disco.update(lastAccX, lastAccY, lastAccZ);

    // ── Deferred flash write: sync RAM-buffered readings when idle ──
    if (ctx.currentState == SystemState::IDLE && flashOk &&
        configMgr.hasPendingToSync()) {
        configMgr.syncPendingToFlash();
    }

    delay(Timing::LOOP_INTERVAL_MS);
}

// ═══════════════════════════════════════════════════════════════════
// ── Polling Functions ─────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

// ── Read sensors + update fusion at 50 Hz ───────────────────────
static void readSensorsUpdate(uint32_t now) {
    if (now - lastSensorUpdate < Timing::SENSOR_POLL_MS) return;

    lastSensorUpdate = now;

    if (magOk) {
        RM3100::Reading mr = mag.readSingle();
        mag.toMicroTesla(mr, lastMagX, lastMagY, lastMagZ);
    }

    if (imuOk) {
        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);
        lastAccX = accel.acceleration.x;
        lastAccY = accel.acceleration.y;
        lastAccZ = accel.acceleration.z;
    }

    if (calOk && magOk && imuOk) {
        Eigen::Vector3f rawMag(lastMagX, lastMagY, lastMagZ);
        Eigen::Vector3f rawGrav(lastAccX, lastAccY, lastAccZ);
        sensorMgr.update(rawMag, rawGrav);
    }
}

// ── Button event handling ───────────────────────────────────────
static void pollButtons(uint32_t now) {
    // ── Button 1 (MEASURE) — take measurement or wake laser ──
    if (buttons.wasPressed(Button::MEASURE)) {
        ctx.purpleLatched = false;
        ctx.lastActivityTime = now;

        if (ctx.displayFrozen && !ctx.laserEnabled) {
            // Leg complete — laser off, just unfreeze + turn laser on
            ctx.displayFrozen = false;
            if (laserOk) {
                laser.setBuzzer(true);
                delay(25);
                laser.setLaser(true);
                delay(200);
                laser.setBuzzer(false);
                delay(25);
            }
            ctx.laserEnabled = true;
            lastDistance = 0;
            disco.turnOff();
            Serial.println(F("MEASURE: unfreezing after leg, live readings"));
        } else if (ctx.displayFrozen && ctx.laserEnabled) {
            // Normal shot freeze — laser is on, go straight to measurement
            ctx.displayFrozen = false;
            ctx.currentState = SystemState::TAKING_MEASUREMENT;
            ctx.measurementTaken = false;
            measRedLedSet = false;
            lastDistance = 0;
            Serial.println(F("MEASURE: taking measurement (from frozen)"));
        } else if (ctx.laserEnabled) {
            // Laser already on, live display — take measurement
            ctx.currentState = SystemState::TAKING_MEASUREMENT;
            ctx.measurementTaken = false;
            measRedLedSet = false;
            lastDistance = 0;
            Serial.println(F("MEASURE: taking measurement"));
        } else if (ctx.currentState == SystemState::IDLE) {
            // Laser was off (timeout etc) — beep + turn on
            if (laserOk) {
                laser.setBuzzer(true);
                delay(25);
                laser.setLaser(true);
                delay(200);
                laser.setBuzzer(false);
                delay(25);
            }
            ctx.laserEnabled = true;
            lastDistance = 0;
            Serial.println(F("MEASURE: laser re-enabled"));
        } else {
            Serial.println(F("MEASURE: system busy, ignored"));
            ctx.currentState = SystemState::IDLE;
        }
    }

    // ── Button 2 (DISCO) — short press: quick shot, hold 2s: toggle disco ──
    if (buttons.isPressed(Button::DISCO)) {
        ctx.lastActivityTime = now;
        if (!discoHolding) {
            discoHoldStart = now;
            discoHolding   = true;
            discoTriggered = false;
        } else if (!discoTriggered) {
            uint32_t held = now - discoHoldStart;
            if (held >= Timing::DISCO_HOLD_MS) {
                // 2s hold reached — toggle disco now
                discoTriggered = true;
                if (ctx.discoOn) {
                    disco.turnOff();
                    ctx.discoOn = false;
                    if (laserOk) laser.setLaser(false);
                    ctx.laserEnabled = false;
                    Serial.println(F("DISCO: off"));
                } else {
                    disco.startDisco();
                    ctx.discoOn = true;
                    if (laserOk) laser.setLaser(false);
                    ctx.laserEnabled = false;
                    Serial.println(F("DISCO: on"));
                }
            }
        }
    } else {
        // Released — only fire quick shot if disco wasn't triggered
        if (discoHolding && !discoTriggered) {
                // Short press — quick shot (wider stability tolerance)
                ctx.purpleLatched = false;

                if (ctx.displayFrozen && !ctx.laserEnabled) {
                    // Leg complete — just unfreeze + turn laser on
                    ctx.displayFrozen = false;
                    if (laserOk) {
                        laser.setBuzzer(true);
                        delay(25);
                        laser.setLaser(true);
                        delay(200);
                        laser.setBuzzer(false);
                        delay(25);
                    }
                    ctx.laserEnabled = true;
                    lastDistance = 0;
                    disco.turnOff();
                    Serial.println(F("QUICK: unfreezing after leg, live readings"));
                } else if (ctx.displayFrozen && ctx.laserEnabled) {
                    ctx.displayFrozen = false;
                    ctx.quickShot = true;
                    ctx.currentState = SystemState::TAKING_MEASUREMENT;
                    ctx.measurementTaken = false;
                    measRedLedSet = false;
                    lastDistance = 0;
                    Serial.println(F("QUICK: taking measurement (from frozen)"));
                } else if (ctx.laserEnabled) {
                    ctx.quickShot = true;
                    ctx.currentState = SystemState::TAKING_MEASUREMENT;
                    ctx.measurementTaken = false;
                    measRedLedSet = false;
                    lastDistance = 0;
                    Serial.println(F("QUICK: taking measurement"));
                } else if (ctx.currentState == SystemState::IDLE) {
                    // Laser was off — re-enable
                    if (laserOk) {
                        laser.setBuzzer(true);
                        delay(25);
                        laser.setLaser(true);
                        delay(200);
                        laser.setBuzzer(false);
                        delay(25);
                    }
                    ctx.laserEnabled = true;
                    lastDistance = 0;
                    Serial.println(F("QUICK: laser re-enabled"));
                }
        }
        discoHolding = false;
    }

    // ── Button 3 (CALIB) — hold 2s: enter menu mode ──
    if (buttons.isPressed(Button::CALIB)) {
        ctx.lastActivityTime = now;
        if (!calibHolding) {
            calibHoldStart = now;
            calibHolding = true;
        } else {
            uint32_t held = now - calibHoldStart;
            if (held >= Timing::CALIB_HOLD_MS &&
                ctx.currentState == SystemState::IDLE) {
                Serial.println(F("CALIB held 2s — entering menu mode"));
                if (laserOk) laser.setLaser(false);
                ctx.laserEnabled = false;
                disco.turnOff();
                ctx.discoOn = false;
                menuMgr.begin(display.getDisplay(), ctx, configMgr);
                calibHolding = false;
                return;  // skip rest of pollButtons
            }
        }
    } else {
        calibHolding = false;
    }

    // ── Button 4 (SHUTDOWN) — power off with post-measurement guard ──
    if (buttons.wasPressed(Button::SHUTDOWN)) {
        uint32_t elapsed = now - ctx.lastMeasurementTime;
        if (elapsed < Timing::MEASURE_GUARD_MS) {
            Serial.println(F("SHUTDOWN: guarding after measurement"));
            delay(Timing::MEASURE_GUARD_MS - elapsed);
        }
        Serial.println(F("SHUTDOWN: powering off"));
        doShutdown();
    }

    // ── Fire button (FIRE) — same as MEASURE (trigger button) ──
    if (buttons.wasPressed(Button::FIRE)) {
        ctx.purpleLatched = false;
        ctx.lastActivityTime = now;

        if (ctx.displayFrozen && !ctx.laserEnabled) {
            // Leg complete — laser off, just unfreeze + turn laser on
            ctx.displayFrozen = false;
            if (laserOk) {
                laser.setBuzzer(true);
                delay(25);
                laser.setLaser(true);
                delay(200);
                laser.setBuzzer(false);
                delay(25);
            }
            ctx.laserEnabled = true;
            lastDistance = 0;
            disco.turnOff();
            Serial.println(F("FIRE: unfreezing after leg, live readings"));
        } else if (ctx.displayFrozen && ctx.laserEnabled) {
            // Normal shot freeze — laser is on, go straight to measurement
            ctx.displayFrozen = false;
            ctx.currentState = SystemState::TAKING_MEASUREMENT;
            ctx.measurementTaken = false;
            measRedLedSet = false;
            lastDistance = 0;
            Serial.println(F("FIRE: taking measurement (from frozen)"));
        } else if (ctx.laserEnabled) {
            // Laser already on, live display — take measurement
            ctx.currentState = SystemState::TAKING_MEASUREMENT;
            ctx.measurementTaken = false;
            measRedLedSet = false;
            lastDistance = 0;
            Serial.println(F("FIRE: taking measurement"));
        } else if (ctx.currentState == SystemState::IDLE) {
            if (laserOk) {
                laser.setBuzzer(true);
                delay(25);
                laser.setLaser(true);
                delay(200);
                laser.setBuzzer(false);
                delay(25);
            }
            ctx.laserEnabled = true;
            lastDistance = 0;
            Serial.println(F("FIRE: laser re-enabled"));
        }
    }
}

// ── Measurement workflow ────────────────────────────────────────
static void pollMeasurement(uint32_t now) {
    if (ctx.currentState != SystemState::TAKING_MEASUREMENT) return;
    if (ctx.measurementTaken) return;
    if (!calOk || !magOk || !imuOk || !laserOk) {
        Serial.println(F("MEAS: sensors not ready"));
        ctx.quickShot = false;
        ctx.currentState = SystemState::IDLE;
        return;
    }

    // Set red LED once on entry
    if (!measRedLedSet) {
        if (!ctx.purpleLatched) {
            disco.setRed();
        }
        // Ensure laser is on (only sends UART if it was off)
        if (!ctx.laserEnabled) {
            laser.setLaser(true);
            delay(200);
            ctx.laserEnabled = true;
        }
        // Entry click buzzer
        if (laserOk) {
            laser.setBuzzer(true);
            delay(100);
            laser.setBuzzer(false);
            delay(25);
        }
        measRedLedSet = true;
    }

    // Wait for stability — use wider tolerance for quick shot
    float stabTol = ctx.quickShot ? Defaults::quickShotStabilityTol
                                  : ctx.config.stabilityTolerance;
    if (!sensorMgr.isStable(stabTol)) {
        static uint32_t lastStabDbg = 0;
        uint32_t n = millis();
        if (n - lastStabDbg > 1000) {
            lastStabDbg = n;
            Serial.print(F("MEAS: waiting stable  AZ="));
            Serial.print(sensorMgr.getAzimuth(), 1);
            Serial.print(F(" INC="));
            Serial.print(sensorMgr.getInclination(), 1);
            Serial.print(F(" tol="));
            Serial.println(stabTol, 1);
        }
        return;
    }
    Serial.println(F("MEAS: stable — measuring"));

    // ── Stable — take measurement ────────────────────────────
    ctx.readings.azimuth     = sensorMgr.getAzimuth();
    ctx.readings.inclination = sensorMgr.getInclination();
    ctx.readings.roll        = sensorMgr.getRoll();

    // Flush any stale UART data before talking to laser
    while (Serial1.available()) Serial1.read();
    delay(50);  // let the UART line settle

    // Take laser distance
    Serial.println(F("MEAS: sending measure cmd..."));
    int32_t distMm = 0;
    LaserError lErr = laser.measure(distMm);
    Serial.print(F("MEAS: result="));
    Serial.print(LaserEgismos::errorString(lErr));
    Serial.print(F(" mm="));
    Serial.println(distMm);
    if (lErr != LaserError::OK) {
        Serial.print(F("MEAS: laser error: "));
        Serial.println(LaserEgismos::errorString(lErr));
        resetLaser();
        alertError("LzrERR");
        ctx.quickShot = false;
        ctx.currentState = SystemState::IDLE;
        return;
    }

    ctx.readings.distance = (distMm / 1000.0f) + ctx.config.laserDistanceOffset;
    lastDistance = ctx.readings.distance;
    Serial.print(F("MEAS: dist=")); Serial.println(ctx.readings.distance, 3);

    // Anomaly detection
    Serial.println(F("MEAS: checking anomaly..."));
    if (ctx.config.anomalyDetection) {
        Eigen::Vector3f rawMag(lastMagX, lastMagY, lastMagZ);
        Eigen::Vector3f rawGrav(lastAccX, lastAccY, lastAccZ);
        MagCal::Strictness strict = {
            ctx.config.magTolerance,
            ctx.config.gravTolerance,
            ctx.config.dipTolerance
        };
        MagCal::AnomalyType anom = calibration.checkAnomaly(rawMag, rawGrav, strict);
        if (anom != MagCal::AnomalyType::NONE) {
            const char* errStr = "Err";
            switch (anom) {
                case MagCal::AnomalyType::MAGNETIC: errStr = "MagErr"; break;
                case MagCal::AnomalyType::GRAVITY:  errStr = "GravErr"; break;
                case MagCal::AnomalyType::DIP:      errStr = "DipErr"; break;
                default: break;
            }
            Serial.print(F("MEAS: anomaly: ")); Serial.println(errStr);
            alertError(errStr);
            // Still update display with the readings
            if (dispOk) {
                display.updateSensorReadings(ctx.readings.distance,
                                             ctx.readings.azimuth,
                                             ctx.readings.inclination);
                display.refresh();
            }
            ctx.quickShot = false;
            ctx.currentState = SystemState::IDLE;
            return;
        }
    }

    // Success!
    Serial.println(F("MEAS: calling handleSuccess..."));
    handleMeasurementSuccess();
    Serial.println(F("MEAS: handleSuccess done"));

    // Update display with shot readings
    if (dispOk) {
        display.updateSensorReadings(ctx.readings.distance,
                                     ctx.readings.azimuth,
                                     ctx.readings.inclination);
        display.refresh();
    }

    // Freeze display — stays showing shot readings until next button press
    ctx.displayFrozen = true;

    if (ctx.purpleLatched) {
        // Leg complete — laser off until user presses button again
        if (laserOk) laser.setLaser(false);
        ctx.laserEnabled = false;
    } else {
        // Normal shot — blink laser off 300ms then back on, ready for next shot
        if (laserOk) laser.setLaser(false);
        ctx.laserEnabled = false;
        delay(300);
        if (laserOk) laser.setLaser(true);
        ctx.laserEnabled = true;
        disco.turnOff();
    }

    ctx.quickShot = false;
    ctx.currentState = SystemState::IDLE;

    Serial.print(F("MEAS OK: AZ="));
    Serial.print(ctx.readings.azimuth, 1);
    Serial.print(F(" INC="));
    Serial.print(ctx.readings.inclination, 1);
    Serial.print(F(" DIST="));
    Serial.println(ctx.readings.distance, 2);
}

// ── Handle successful measurement ───────────────────────────────
static void handleMeasurementSuccess() {
    ctx.lastMeasurementTime = millis();

    // Green LED + success beep
    disco.setGreen();
    if (laserOk) {
        delay(25);
        laser.setBuzzer(true);
        delay(100);
        laser.setBuzzer(false);
        delay(25);
    } else {
        delay(150);
    }

    Serial.print(F("  HS:2 bleConn="));
    Serial.print(ctx.bleConnected);
    Serial.print(F(" bleOk="));
    Serial.println(bleOk);
    Serial.flush();
    delay(50);
    // Send via BLE or queue
    if (ctx.bleConnected && bleOk) {
        Serial.println(F("  HS:2a ble send")); Serial.flush(); delay(50);
        ble.sendSurveyData(ctx.readings.azimuth,
                           ctx.readings.inclination,
                           ctx.readings.distance);
        ctx.bleDisconnectionCounter = 0;
    } else {
        ctx.bleDisconnectionCounter++;
        if (dispOk) display.updateBTNumber(ctx.bleDisconnectionCounter);
        // Buffer reading in RAM — synced to flash during IDLE or shutdown
        configMgr.appendPendingReading(ctx.readings.azimuth,
                                       ctx.readings.inclination,
                                       ctx.readings.distance);
    }

    // ── Leg consistency buffer (skip for quick shots) ─────────
    if (ctx.quickShot) {
        Serial.println(F("  HS:3 quick shot — skipping leg buf"));
        ctx.measurementTaken = false;
        return;
    }
    Serial.println(F("  HS:3 leg buf")); Serial.flush();
    uint8_t idx = ctx.stableBufCount;
    if (idx < 3) {
        ctx.stableAzimuthBuf[idx]     = ctx.readings.azimuth;
        ctx.stableInclinationBuf[idx] = ctx.readings.inclination;
        ctx.stableDistanceBuf[idx]    = ctx.readings.distance;
        ctx.stableBufCount++;
    } else {
        // Shift left and add new
        ctx.stableAzimuthBuf[0]     = ctx.stableAzimuthBuf[1];
        ctx.stableAzimuthBuf[1]     = ctx.stableAzimuthBuf[2];
        ctx.stableAzimuthBuf[2]     = ctx.readings.azimuth;
        ctx.stableInclinationBuf[0] = ctx.stableInclinationBuf[1];
        ctx.stableInclinationBuf[1] = ctx.stableInclinationBuf[2];
        ctx.stableInclinationBuf[2] = ctx.readings.inclination;
        ctx.stableDistanceBuf[0]    = ctx.stableDistanceBuf[1];
        ctx.stableDistanceBuf[1]    = ctx.stableDistanceBuf[2];
        ctx.stableDistanceBuf[2]    = ctx.readings.distance;
    }

    Serial.println(F("  HS:4 leg check")); Serial.flush();
    // Check leg completion when we have 3 readings
    if (ctx.stableBufCount >= 3) {
        bool azOk   = bearingsWithinTol(ctx.stableAzimuthBuf, 3,
                                         ctx.config.legAngleTolerance);
        bool incOk  = linearWithinTol(ctx.stableInclinationBuf, 3,
                                       ctx.config.legAngleTolerance);
        bool distOk = linearWithinTol(ctx.stableDistanceBuf, 3,
                                       ctx.config.legDistanceTolerance);

        if (azOk && incOk && distOk) {
            // Leg complete! Triple buzz + white flash
            for (int i = 0; i < 3; i++) {
                if (laserOk) laser.setBuzzer(true);
                disco.setWhite();
                delay(100);
                if (laserOk) laser.setBuzzer(false);
                disco.turnOff();
                delay(100);
            }

            // Clear buffers
            ctx.stableBufCount = 0;

            // Latch purple
            disco.setPurple();
            ctx.purpleLatched = true;
            ctx.measurementTaken = true;

            Serial.println(F("LEG COMPLETE — 3 consistent readings"));
            return;
        }
    }

    Serial.println(F("  HS:5 done")); Serial.flush();
    ctx.measurementTaken = false;
}

// ── Error alert — red flash sequence ────────────────────────────
static void alertError(const char* errCode) {
    if (dispOk) {
        display.updateDistanceText(errCode);
        display.refresh();
    }

    disco.turnOff();
    for (int i = 0; i < 4; i++) {
        disco.setRed();
        delay(100);
        disco.turnOff();
        delay(100);
    }

    // Beep after error flashes, re-enable laser
    if (laserOk) {
        laser.setBuzzer(true);
        delay(100);
        laser.setBuzzer(false);
        delay(25);
        laser.setLaser(true);
        delay(25);
    }
    ctx.laserEnabled = true;
    ctx.measurementTaken = true;
}

// ── BLE pin monitoring (connection state + flush) ───────────────
static void pollBLEPin(uint32_t now) {
    if (!bleOk) return;
    if (now - lastBleCheck < Timing::BLE_PIN_CHECK_MS) return;
    lastBleCheck = now;

    bool connected = ble.isConnected();
    ctx.bleConnected = connected;

    // Transition: disconnected → connected
    if (connected && !lastBleConnected) {
        Serial.println(F("BLE: connected"));

        // Flush pending readings if any
        if (ctx.bleDisconnectionCounter > 0 && flashOk) {
            disco.setBlue();
            if (dispOk) {
                display.updateBTNumber(ctx.bleDisconnectionCounter);
                display.refresh();
            }
            delay(1000);   // let BLE slave be ready

            configMgr.flushPendingReadings(onFlushReading);
            configMgr.clearPendingReadings();

            ctx.bleDisconnectionCounter = 0;
            ctx.bleReadingsTransferredFlag = false;
            disco.turnOff();
            Serial.println(F("BLE: pending readings flushed"));
        }
    }

    if (!connected && lastBleConnected) {
        Serial.println(F("BLE: disconnected"));
    }

    // Update display
    if (dispOk) {
        display.updateBTLabel(connected);
        if (connected) {
            if (ctx.bleReadingsTransferredFlag) {
                display.updateBTNumber(0);
                ctx.bleDisconnectionCounter = 0;
            } else {
                display.updateBTNumber(0);
            }
        } else {
            display.updateBTNumber(ctx.bleDisconnectionCounter);
        }
    }

    lastBleConnected = connected;
}

// ── BLE UART command processing ─────────────────────────────────
static void pollBLECommands(uint32_t now) {
    if (!bleOk) return;
    if (now - lastBleUartPoll < Timing::BLE_UART_POLL_MS) return;
    lastBleUartPoll = now;

    ble.update();
    if (!ble.hasCommand()) return;

    BleCommand cmd = ble.readCommand();
    Serial.print(F("BLE CMD: "));
    Serial.println(BleManager::commandName(cmd));

    switch (cmd) {
        case BleCommand::ACK_RECEIVED:
            ctx.bleReadingsTransferredFlag = true;
            break;

        case BleCommand::READY:
            bleReadySeen = true;
            break;

        case BleCommand::TAKE_SHOT:
            ctx.currentState = SystemState::TAKING_MEASUREMENT;
            ctx.measurementTaken = false;
            measRedLedSet = false;
            ctx.lastActivityTime = now;
            break;

        case BleCommand::LASER_ON:
            if (laserOk) laser.setLaser(true);
            ctx.laserEnabled = true;
            break;

        case BleCommand::LASER_OFF:
            if (laserOk) laser.setLaser(false);
            ctx.laserEnabled = false;
            break;

        case BleCommand::DEVICE_OFF:
            doShutdown();
            break;

        case BleCommand::START_CAL:
            Serial.println(F("BLE: entering menu mode"));
            if (laserOk) laser.setLaser(false);
            ctx.laserEnabled = false;
            disco.turnOff();
            ctx.discoOn = false;
            menuMgr.begin(display.getDisplay(), ctx, configMgr);
            break;

        case BleCommand::STOP_CAL:
            ctx.currentState = SystemState::IDLE;
            break;

        default:
            break;
    }
}

// ── BLE startup name sync (robust against faster main-board boot) ──
static void pollBLEStartupNameSync(uint32_t now) {
    if (!bleOk || bleNameSyncDone) return;

    constexpr uint32_t NAME_SYNC_INITIAL_DELAY_MS = 1200;
    constexpr uint32_t NAME_SYNC_RETRY_MS         = 800;
    constexpr uint32_t NAME_SYNC_AFTER_READY_MS   = 150;
    constexpr uint8_t  NAME_SYNC_MAX_ATTEMPTS     = 6;

    uint32_t elapsed = now - bleNameSyncStartMs;
    if (elapsed < NAME_SYNC_INITIAL_DELAY_MS) return;
    if (bleNameSyncAttempts >= NAME_SYNC_MAX_ATTEMPTS) {
        bleNameSyncDone = true;
        Serial.println(F("BLE NAME: sync window ended"));
        return;
    }

    // If DiscoX reports READY, force one post-ready NAME send and stop.
    if (bleReadySeen && !bleNameSentAfterReady) {
        if (bleNameSyncLastSendMs != 0 &&
            (now - bleNameSyncLastSendMs) < NAME_SYNC_AFTER_READY_MS) {
            return;
        }

        ble.setName(ctx.config.bleName);
        bleNameSyncLastSendMs = now;
        bleNameSyncAttempts++;
        bleNameSentAfterReady = true;
        bleNameSyncDone = true;
        Serial.println(F("BLE NAME: sync complete (READY)"));
        return;
    }

    if (bleNameSyncLastSendMs != 0 &&
        (now - bleNameSyncLastSendMs) < NAME_SYNC_RETRY_MS) {
        return;
    }

    ble.setName(ctx.config.bleName);
    bleNameSyncLastSendMs = now;
    bleNameSyncAttempts++;

    Serial.print(F("BLE NAME: sync attempt "));
    Serial.println(bleNameSyncAttempts);

    // If BLE comes up and we sent at least once, stop retrying (fallback path).
    if (ctx.bleConnected && bleNameSyncAttempts >= 1) {
        bleNameSyncDone = true;
        Serial.println(F("BLE NAME: sync complete"));
    }
}

// ── Battery check (every 30s) ───────────────────────────────────
static void pollBattery(uint32_t now) {
    if (!batOk) return;
    if (now - lastBatRead < Timing::BATTERY_CHECK_MS) return;
    lastBatRead = now;

    lastBatPct = battery.cellPercent();
    ctx.readings.batteryLevel = lastBatPct;
    Serial.print(F("BAT: ")); Serial.print(battery.cellVoltage(), 3);
    Serial.print(F("V  ")); Serial.print(lastBatPct, 1); Serial.println(F("%"));

    if (dispOk) display.updateBattery(lastBatPct);

    if (lastBatPct > 0.5f && lastBatPct <= Timing::BATTERY_SHUTDOWN_PCT) {
        Serial.println(F("LOW BATTERY — shutting down"));
        if (dispOk) {
            display.updateDistanceText("LOW");
            display.updateAzimuth(0);
            display.updateInclination(0);
            display.refresh();
        }
        disco.setRed();
        delay(3000);
        doShutdown();
    }
}

// ── Auto shutdown check (every 5s) ──────────────────────────────
static void checkAutoShutoff(uint32_t now) {
    if (now - lastAutoShutCheck < Timing::AUTO_SHUTOFF_CHECK_MS) return;
    lastAutoShutCheck = now;

    uint32_t inactiveSec = (now - ctx.lastActivityTime) / 1000;
    if (inactiveSec > ctx.config.autoShutdownTimeout) {
        Serial.println(F("Inactivity timeout — shutting down"));
        doShutdown();
    }
}

// ── Laser timeout check (every 1s) ─────────────────────────────
static void checkLaserTimeout(uint32_t now) {
    if (now - lastLaserTimeCheck < Timing::LASER_TIMEOUT_CHECK_MS) return;
    lastLaserTimeCheck = now;

    if (!ctx.laserEnabled) return;

    uint32_t inactiveSec = (now - ctx.lastActivityTime) / 1000;
    if (inactiveSec > ctx.config.laserTimeout) {
        Serial.println(F("Laser timeout — turning off"));
        if (laserOk) laser.setLaser(false);
        ctx.laserEnabled = false;
    }
}

// ── Display update (4 Hz) ───────────────────────────────────────
static void updateDisplay(uint32_t now) {
    if (!dispOk) return;
    if (now - lastDisplayRefresh < Timing::DISPLAY_REFRESH_MS) return;
    lastDisplayRefresh = now;

    // Only update live sensor readings when display is not frozen.
    // After a measurement the display stays frozen showing the shot
    // reading until the user presses MEASURE again to resume live mode.
    if (!ctx.displayFrozen) {
        float liveAz, liveInc;
        if (calOk) {
            liveAz  = sensorMgr.getAzimuth();
            liveInc = sensorMgr.getInclination();
        } else {
            liveAz = atan2f(lastMagY, lastMagX) * (180.0f / PI);
            if (liveAz < 0) liveAz += 360.0f;
            liveInc = atan2f(lastAccZ,
                      sqrtf(lastAccX * lastAccX + lastAccY * lastAccY))
                      * (180.0f / PI);
        }

        // Deadband: only update displayed value when reading moves enough
        if (SensorManager::circularDiff(liveAz, dispAz) > DEADBAND_ANGLE)
            dispAz = liveAz;
        if (fabsf(liveInc - dispInc) > DEADBAND_ANGLE)
            dispInc = liveInc;

        display.updateSensorReadings(lastDistance, dispAz, dispInc);
    }

    display.updateBattery(lastBatPct);
    display.updateBTLabel(bleOk ? ble.isConnected() : false);
    display.refresh();
}

// ═══════════════════════════════════════════════════════════════════
// ── Helper Functions ──────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

static void resetLaser() {
    if (!laserOk) return;
    Serial.println(F("LASER: resetting"));
    laser.setLaser(false);
    delay(100);
    while (Serial1.available()) Serial1.read();  // flush UART
    laser.setLaser(true);
    delay(200);  // let module stabilise
    ctx.laserEnabled = true;
}

static void doShutdown() {
    Serial.println(F(">>> SHUTDOWN"));
    // Flush any unsaved readings to flash before power dies
    if (flashOk && configMgr.hasPendingToSync()) {
        configMgr.syncPendingToFlash();
    }
    Serial.flush();
    digitalWrite(PIN_POWER, LOW);
    // If still running (USB-powered), just hang
    while (true) { delay(1000); }
}

static float circularDiff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return fabsf(d);
}

static bool bearingsWithinTol(const float* vals, uint8_t n, float tol) {
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1; j < n; j++) {
            if (circularDiff(vals[i], vals[j]) > tol) return false;
        }
    }
    return true;
}

static bool linearWithinTol(const float* vals, uint8_t n, float tol) {
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1; j < n; j++) {
            if (fabsf(vals[i] - vals[j]) > tol) return false;
        }
    }
    return true;
}

static void onFlushReading(float az, float inc, float dist) {
    Serial.print(F("FLUSH: az=")); Serial.print(az, 1);
    Serial.print(F(" inc=")); Serial.print(inc, 1);
    Serial.print(F(" dist=")); Serial.println(dist, 2);
    ble.sendSurveyData(az, inc, dist);
    delay(50);   // pacing between BLE sends
}

// ═══════════════════════════════════════════════════════════════════
// ── Initialization Functions (unchanged from previous sessions) ───
// ═══════════════════════════════════════════════════════════════════

static void initPins() {
    pinMode(PIN_BTN_MEASURE,  INPUT_PULLUP);
    pinMode(PIN_BTN_DISCO,    INPUT_PULLUP);
    pinMode(PIN_BTN_CALIB,    INPUT_PULLUP);
    pinMode(PIN_BTN_SHUTDOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_FIRE,     INPUT_PULLUP);

    pinMode(PIN_POWER, OUTPUT);
    digitalWrite(PIN_POWER, HIGH);

    pinMode(PIN_BLE_STATUS, INPUT_PULLDOWN);
    pinMode(PIN_BLE_DRDY,   OUTPUT);
    digitalWrite(PIN_BLE_DRDY, LOW);

    pinMode(PIN_MAG_DRDY, INPUT);
    pinMode(PIN_CFG, INPUT_PULLUP);

    Serial.println(F("Pins initialized."));
}

static void scanI2C() {
    uint8_t count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) Serial.print('0');
            Serial.print(addr, HEX);
            if (addr == RM3100_I2C_ADDR)  Serial.print(F(" (RM3100 mag)"));
            if (addr == ISM330DHCX_ADDR)  Serial.print(F(" (ISM330DHCX accel/gyro)"));
            if (addr == MAX17048_ADDR)    Serial.print(F(" (MAX17048 battery)"));
            if (addr == SH1107_ADDR)      Serial.print(F(" (SH1107 OLED)"));
            Serial.println();
            count++;
        }
    }
    Serial.print(F("  Found "));
    Serial.print(count);
    Serial.println(F(" device(s)."));
}

// initSensors() inlined into setup() for splash progress updates

static void initDisplay() {
    dispOk = display.begin();
    Serial.print(F("SH1107:      "));
    Serial.println(dispOk ? F("OK") : F("FAILED"));

    // Splash is shown from setup() after initDisplay() returns

    Serial.println();
}

static void initLaser() {
    Serial1.begin(LASER_UART_BAUD_EGISMOS);
    laserOk = laser.begin(Serial1);
    Serial.print(F("Laser:       "));
    Serial.println(laserOk ? F("OK (Egismos @ 9600)") : F("FAILED"));

    // Buzzer disabled for debugging — skip init command

    Serial.println();
}

static void initBle() {
    bleOk = ble.begin();
    Serial.print(F("BLE UART:    "));
    Serial.println(bleOk ? F("OK (SERCOM1 @ 9600)") : F("FAILED"));

    Serial.println();
}

static void initFlash() {
    flashOk = configMgr.begin();
    Serial.print(F("Flash/FS:    "));
    Serial.println(flashOk ? F("OK") : F("FAILED (using defaults)"));

    if (flashOk) {
        if (configMgr.loadConfig(ctx.config)) {
            Serial.println(F("  Config loaded from flash"));
        } else {
            Serial.println(F("  No saved config — writing defaults"));
            configMgr.saveConfig(ctx.config);
        }

        uint16_t pending = configMgr.countPendingReadings();
        if (pending > 0) {
            Serial.print(F("  Pending readings: "));
            Serial.println(pending);
            ctx.bleDisconnectionCounter = pending;
        }

        if (configMgr.hasFlag(Flags::CALIBRATION)) {
            configMgr.clearFlag(Flags::CALIBRATION);
            Serial.println(F("  ** Calibration mode flag detected"));
            enterCalibMode = true;
        }
        if (configMgr.hasFlag(Flags::MENU)) {
            configMgr.clearFlag(Flags::MENU);
            Serial.println(F("  ** Menu mode flag detected"));
            enterMenuMode = true;
        }
        if (configMgr.hasFlag(Flags::SNAKE)) {
            configMgr.clearFlag(Flags::SNAKE);
            Serial.println(F("  ** Snake mode flag detected"));
            enterSnakeMode = true;
        }
        if (configMgr.hasFlag("usb_drive")) {
            configMgr.clearFlag("usb_drive");
            Serial.println(F("  ** USB drive mode flag detected"));
            enterUsbDriveMode();  // does not return
        }
    }

    Serial.println();
}

static void initCalibration() {
    Serial.print(F("Calibration: "));

    bool loaded = false;

    // 1. Try binary from flash (fastest — no JSON parse)
    if (flashOk) {
        MagCal::CalibrationBinary bin;
        if (configMgr.loadCalibrationBinary(bin)) {
            loaded = calibration.fromBinary(bin);
            if (loaded) {
                Serial.println(F("OK (from binary)"));
            }
        }
    }

    // 2. Fall back to JSON from flash
    if (!loaded && flashOk) {
        char calBuf[1024];
        size_t calLen = 0;
        if (configMgr.loadCalibrationJson(calBuf, sizeof(calBuf), calLen)) {
            loaded = calibration.fromJson(calBuf, calLen);
            if (loaded) {
                Serial.println(F("OK (from flash JSON)"));
            }
        }
    }

    // 3. Fall back to compiled-in PROGMEM JSON
    if (!loaded) {
        loaded = calibration.fromJson(CALIBRATION_JSON, sizeof(CALIBRATION_JSON) - 1);
        if (loaded) {
            Serial.println(F("OK (from PROGMEM)"));
        }
    }

    calOk = loaded;

    if (calOk) {
        Serial.print(F("  Mag axes:  ")); Serial.println(calibration.mag().axes().toString());
        Serial.print(F("  Grav axes: ")); Serial.println(calibration.grav().axes().toString());
        Serial.print(F("  Dip avg:   ")); Serial.print(calibration.dipAvg(), 1);
        Serial.println(F(" deg"));

        sensorMgr.init(&calibration, ctx.config.emaAlpha,
                        ctx.config.stabilityBufferLength);
        Serial.print(F("  Filter:    EMA alpha="));
        Serial.print(ctx.config.emaAlpha, 2);
        Serial.print(F(", stability buf="));
        Serial.println(ctx.config.stabilityBufferLength);
    } else {
        Serial.println(F("FAILED — using raw angles"));
    }

    Serial.println();
}

static void initDisco() {
    digitalWrite(PIN_NEOPIXEL_POWER, HIGH);
    disco.begin();
    Serial.println(F("NeoPixel:    OK"));
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// ── USB Mass Storage Drive Mode ───────────────────────────────────
// ═══════════════════════════════════════════════════════════════════

static void enterUsbDriveMode() {
    Serial.println(F("=== USB Drive Mode ==="));

    // Show OLED message
    if (dispOk) {
        auto& d = display.getDisplay();
        d.clearDisplay();
        d.setTextSize(1);
        d.setTextColor(SH110X_WHITE);
        d.setCursor(0, 10);
        d.println(F("USB Drive Mode"));
        d.println();
        d.println(F("Edit config.json"));
        d.println(F("on the USB drive."));
        d.println();
        d.println(F("Eject drive, then"));
        d.println(F("hold power button"));
        d.println(F("to restart."));
        d.display();
    }

    // Get pointer to the QSPI flash from ConfigManager
    s_mscFlash = configMgr.getFlash();
    if (!s_mscFlash) {
        Serial.println(F("USB MSC: flash not available!"));
        NVIC_SystemReset();
    }

    // Configure USB MSC
    usbMsc.setID("Mr_Zappy", "Settings", "1.0");
    usbMsc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
    usbMsc.setCapacity(s_mscFlash->size() / 512, 512);
    usbMsc.setUnitReady(true);
    usbMsc.begin();

    Serial.println(F("USB MSC started — waiting for host"));

    // Spin forever — device stays in USB drive mode until power cycled
    // The power button (LTC2952) will cut power, causing a fresh reboot
    while (true) {
        delay(100);
    }
    // Does not return
}
