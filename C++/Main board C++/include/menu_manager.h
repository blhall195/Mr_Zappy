#pragma once

#include <Arduino.h>
#include "fruity_menu.h"
#include "device_context.h"
#include "config_manager.h"
#include "button_manager.h"

/// What should happen after the menu closes
enum class MenuExitAction : uint8_t {
    NONE,               // still active or not yet used
    RETURN_NORMAL,      // go back to normal operation
    ENTER_LONG_CALIB,   // enter long calibration mode
    ENTER_SHORT_CALIB,  // enter short calibration mode
    ENTER_SNAKE,        // launch snake game
    ENTER_BOOTLOADER,   // reboot into UF2 bootloader for firmware update
    ENTER_USB_DRIVE,    // reboot into USB mass storage mode for settings
};

class MenuManager {
public:
    // Enter menu mode — takes over the display until exit
    void begin(Adafruit_SH1107& display, DeviceContext& ctx, ConfigManager& cfgMgr);

    // Poll buttons and drive the menu. Call once per loop() iteration.
    void update(ButtonManager& buttons);

    bool isActive() const { return _active; }

    /// Check what action was requested when menu became inactive
    MenuExitAction exitAction() const { return _exitAction; }

    /// Clear the exit action after it has been handled
    void clearExitAction() { _exitAction = MenuExitAction::NONE; }

private:
    void buildMenu();

    // Menu hierarchy (statically allocated, rebuilt on setting changes)
    FruityMenu _root;
    FruityMenu _calSub;
    FruityMenu _anomalySub;
    FruityMenu _deleteSub;
    FruityMenu _laserSub;
    FruityMenu _shutdownSub;
    FruityMenu _bootloaderSub;

    // Dynamic label buffers (updated in buildMenu)
    char _anomalyLabel[24];
    char _laserLabel[24];
    char _shutdownLabel[24];

    // State
    Adafruit_SH1107* _display  = nullptr;
    DeviceContext*    _ctx     = nullptr;
    ConfigManager*    _cfgMgr  = nullptr;
    bool              _active  = false;
    uint32_t          _lastActivity = 0;
    MenuExitAction    _exitAction = MenuExitAction::NONE;

    // Static singleton pointer for callbacks
    static MenuManager* s_instance;

    // Callbacks (static free functions, access state via s_instance)
    static void goToRoot(int);
    static void enterLongCalibration(int);
    static void enterShortCalibration(int);
    static void setAnomalyOn(int);
    static void setAnomalyOff(int);
    static void deletePending(int);
    static void setLaserTimeout(int value);
    static void setAutoShutdown(int value);
    static void enterSnakeGame(int);
    static void enterBootloader(int);
    static void enterUsbDrive(int);
    static void exitMenu(int);
};
