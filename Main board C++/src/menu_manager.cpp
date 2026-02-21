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
        _root.show();
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

    // ── Root menu items ──────────────────────────────────────────
    _root.addSubmenu("Enter Calibration", &_calSub);
    _root.addSubmenu(_anomalyLabel, &_anomalySub);
    _root.addSubmenu("Delete saved shots", &_deleteSub);
    _root.addSubmenu(_laserLabel, &_laserSub);
    _root.addSubmenu(_shutdownLabel, &_shutdownSub);
    _root.addAction("Exit", exitMenu);

    // ── Enter Calibration submenu ────────────────────────────────
    _calSub.addAction("No", goToRoot);
    _calSub.addAction("Yes", enterCalibration);
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
}

// ── Static callbacks ─────────────────────────────────────────────────

void MenuManager::goToRoot(int) {
    if (!s_instance) return;
    s_instance->_root.closeSub();
}

void MenuManager::enterCalibration(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: entering calibration mode"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::ENTER_CALIB;
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
    Serial.println(F("Menu: pending readings deleted"));
    s_instance->_root.closeSub();
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

void MenuManager::exitMenu(int) {
    if (!s_instance) return;
    Serial.println(F("Menu: exiting to normal mode"));
    s_instance->_active = false;
    s_instance->_exitAction = MenuExitAction::RETURN_NORMAL;
}
