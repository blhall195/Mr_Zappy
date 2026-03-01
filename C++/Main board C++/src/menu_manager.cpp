#include "menu_manager.h"
#include "config.h"

MenuManager* MenuManager::s_instance = nullptr;

// ── Public API ───────────────────────────────────────────────────────

void MenuManager::begin(Adafruit_SH1107& display, DeviceContext& ctx,
                         ConfigManager& cfgMgr) {
    _display  = &display;
    _ctx      = &ctx;
    _cfgMgr   = &cfgMgr;
    _active     = true;
    _exitAction = MenuExitAction::NONE;
    _lastActivity = millis();
    s_instance = this;

    Serial.println(F("Menu mode active"));
    buildMenu();
    _root.show();
}

void MenuManager::update(ButtonManager& buttons) {
    if (!_active) return;

    // ── Viewing cal metrics overlay: any button returns to menu ──
    if (_viewingCalMetrics) {
        if (buttons.wasPressed(Button::MEASURE) ||
            buttons.wasPressed(Button::DISCO)   ||
            buttons.wasPressed(Button::CALIB)   ||
            buttons.wasPressed(Button::FIRE)    ||
            buttons.wasPressed(Button::SHUTDOWN)) {
            _viewingCalMetrics = false;
            _lastActivity = millis();
            _root.show();
        }
        return;
    }

    bool interacted = false;

    // Scroll up
    if (buttons.wasPressed(Button::MEASURE)) {
        _root.scroll(-1);
        interacted = true;
    }

    // Scroll down
    if (buttons.wasPressed(Button::DISCO)) {
        _root.scroll(1);
        interacted = true;
    }

    // Click / select
    if (buttons.wasPressed(Button::CALIB)) {
        _root.click();
        interacted = true;
    }

    // Power off
    if (buttons.wasPressed(Button::SHUTDOWN)) {
        Serial.println(F("Menu: power off"));
        digitalWrite(PIN_POWER, LOW);
        return;
    }

    if (interacted) {
        _lastActivity = millis();
        if (!_viewingCalMetrics) _root.show();
    }

    // Auto shutdown
    uint32_t timeout = (uint32_t)_ctx->config.autoShutdownTimeout * 1000UL;
    if (millis() - _lastActivity > timeout) {
        Serial.println(F("Menu: inactivity timeout, shutting down"));
        digitalWrite(PIN_POWER, LOW);
    }
}

// ── Menu construction ────────────────────────────────────────────────

void MenuManager::buildMenu() {
    // ── Build dynamic labels ─────────────────────────────────────
    snprintf(_anomalyLabel, sizeof(_anomalyLabel), "Anomaly Detec: %s",
             _ctx->config.anomalyDetection ? "On" : "Off");

    uint32_t lt = _ctx->config.laserTimeout;
    if (lt >= 61) {
        snprintf(_laserLabel, sizeof(_laserLabel), "Laser off: %lu min",
                 (unsigned long)(lt / 60));
    } else {
        snprintf(_laserLabel, sizeof(_laserLabel), "Laser off: %lu Sec",
                 (unsigned long)lt);
    }

    snprintf(_shutdownLabel, sizeof(_shutdownLabel), "Shutdown: %lu min",
             (unsigned long)(_ctx->config.autoShutdownTimeout / 60));

    // ── Initialize all menus ─────────────────────────────────────
    _root.init(*_display, "Main Menu");
    _calSub.init(*_display, "Enter Calibration");
    _anomalySub.init(*_display, _anomalyLabel);
    _deleteSub.init(*_display, "Delete saved shots");
    _laserSub.init(*_display, _laserLabel);
    _shutdownSub.init(*_display, _shutdownLabel);
    _bootloaderSub.init(*_display, "Update / Settings");

    // ── Root menu items ────────────────────────────────
    _root.addAction("Exit", exitMenu);
    _root.addSubmenu("Enter Calibration", &_calSub);
    _root.addSubmenu(_anomalyLabel, &_anomalySub);
    _root.addSubmenu("Delete saved shots", &_deleteSub);
    _root.addSubmenu(_laserLabel, &_laserSub);
    _root.addSubmenu(_shutdownLabel, &_shutdownSub);
    _root.addSubmenu("Update / Settings", &_bootloaderSub);
    _root.addAction("Play Snake", enterSnakeGame);

    // ── Enter Calibration submenu ────────────────────────────────
    _calSub.addAction("Long Calibration", enterLongCalibration);
    _calSub.addAction("Short Calibration", enterShortCalibration);
    _calSub.addAction("Mag Field Check", enterFBCheck);
    _calSub.addAction("View Last Cal", viewLastCal);
    _calSub.addAction("<- Back", goToRoot);

    // ── Anomaly Detection submenu ────────────────────────────────
    _anomalySub.addAction("On", setAnomalyOn);
    _anomalySub.addAction("Off", setAnomalyOff);
    _anomalySub.addAction("<- Back", goToRoot);

    // ── Delete Saved Shots submenu ───────────────────────────────
    _deleteSub.addAction("No", goToRoot);
    _deleteSub.addAction("Yes DELETE them", deletePending);
    _deleteSub.addAction("<- Back", goToRoot);

    // ── Laser Timeout submenu ────────────────────────────────────
    _laserSub.addAction("30 sec", setLaserTimeout, 30);
    _laserSub.addAction("60 sec", setLaserTimeout, 60);
    _laserSub.addAction("2 min",  setLaserTimeout, 120);
    _laserSub.addAction("5 min",  setLaserTimeout, 300);
    _laserSub.addAction("15 min", setLaserTimeout, 900);
    _laserSub.addAction("30 min", setLaserTimeout, 1800);
    _laserSub.addAction("<- Back", goToRoot);

    // ── Auto Shutdown submenu ────────────────────────────────────
    _shutdownSub.addAction("5 min",  setAutoShutdown, 300);
    _shutdownSub.addAction("10 min", setAutoShutdown, 600);
    _shutdownSub.addAction("15 min", setAutoShutdown, 900);
    _shutdownSub.addAction("30 min", setAutoShutdown, 1800);
    _shutdownSub.addAction("60 min", setAutoShutdown, 3600);
    _shutdownSub.addAction("2 hr",   setAutoShutdown, 7200);
    _shutdownSub.addAction("<- Back", goToRoot);

    // ── Update / Settings submenu (direct actions, no confirmation) ──
    _bootloaderSub.addAction("Update Firmware", enterBootloader);
    _bootloaderSub.addAction("Edit Settings File", enterUsbDrive);
    _bootloaderSub.addAction("<- Back", goToRoot);
}

// ── Static callbacks ─────────────────────────────────────────────────

void MenuManager::goToRoot(int) {
    if (!s_instance) return;
    s_instance->_root.closeSub();
}

void MenuManager::enterLongCalibration(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: entering long calibration"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_LONG_CALIB;
}

void MenuManager::enterShortCalibration(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: entering short calibration"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_SHORT_CALIB;
}

void MenuManager::setAnomalyOn(int) {
    if (!s_instance) return;
    s_instance->_ctx->config.anomalyDetection = true;
    s_instance->_cfgMgr->saveConfig(s_instance->_ctx->config);
    Serial.println(F("Menu: anomaly detection ON"));
    s_instance->buildMenu();
}

void MenuManager::setAnomalyOff(int) {
    if (!s_instance) return;
    s_instance->_ctx->config.anomalyDetection = false;
    s_instance->_cfgMgr->saveConfig(s_instance->_ctx->config);
    Serial.println(F("Menu: anomaly detection OFF"));
    s_instance->buildMenu();
}

void MenuManager::deletePending(int) {
    if (!s_instance) return;
    s_instance->_cfgMgr->clearPendingReadings();
    s_instance->_ctx->bleDisconnectionCounter = 0;
    Serial.println(F("Menu: pending readings deleted"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::RETURN_NORMAL;
}

void MenuManager::setLaserTimeout(int value) {
    if (!s_instance) return;
    s_instance->_ctx->config.laserTimeout = (uint32_t)value;
    s_instance->_cfgMgr->saveConfig(s_instance->_ctx->config);
    Serial.print(F("Menu: laser timeout = "));
    Serial.println(value);
    s_instance->buildMenu();
}

void MenuManager::setAutoShutdown(int value) {
    if (!s_instance) return;
    s_instance->_ctx->config.autoShutdownTimeout = (uint32_t)value;
    s_instance->_cfgMgr->saveConfig(s_instance->_ctx->config);
    Serial.print(F("Menu: auto shutdown = "));
    Serial.println(value);
    s_instance->buildMenu();
}

void MenuManager::enterBootloader(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: entering bootloader for firmware update"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_BOOTLOADER;
}

void MenuManager::enterUsbDrive(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: entering USB drive mode for settings"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_USB_DRIVE;
}

void MenuManager::enterFBCheck(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: entering F/B field check"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_FB_CHECK;
}

void MenuManager::viewLastCal(int) {
    if (!s_instance) return;

    ConfigManager::CalMetrics m;
    auto& d = *s_instance->_display;
    d.clearDisplay();
    d.setTextColor(SH110X_WHITE);

    if (s_instance->_cfgMgr->loadCalMetrics(m)) {
        d.setTextSize(2);
        d.setCursor(0, 0);
        d.println(F("Last Cal"));

        char buf[24];
        d.setTextSize(1);
        d.setCursor(0, 28);
        snprintf(buf, sizeof(buf), "Mag:  %.5f", (double)m.mag);
        d.println(buf);
        d.setCursor(0, 40);
        snprintf(buf, sizeof(buf), "Grav: %.5f", (double)m.grav);
        d.println(buf);
        d.setCursor(0, 52);
        snprintf(buf, sizeof(buf), "Acc:  %.3f deg", (double)m.accuracy);
        d.println(buf);

        d.setCursor(0, 72);
        d.println(F("Lower = Better"));
        d.setCursor(0, 100);
        d.println(F("Any button to return"));
    } else {
        d.setTextSize(2);
        d.setCursor(10, 40);
        d.println(F("No data"));
        d.setTextSize(1);
        d.setCursor(0, 100);
        d.println(F("Any button to return"));
    }

    d.display();
    s_instance->_viewingCalMetrics = true;
    s_instance->_lastActivity = millis();
}

void MenuManager::enterSnakeGame(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: launching snake game"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_SNAKE;
}

void MenuManager::exitMenu(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: exiting to normal mode"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::RETURN_NORMAL;
}
