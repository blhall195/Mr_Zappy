#pragma once

#include <Adafruit_SH110X.h>

class FruityMenu;

// Callback signature: void function(int arg)
typedef void (*MenuAction)(int);

// A single item in a menu (action or submenu link)
struct FruityMenuItem {
    const char* text;
    MenuAction  action;     // non-null → action item
    int         actionArg;
    FruityMenu* submenu;    // non-null → submenu item
};

class FruityMenu {
public:
    // Initialize with display reference and title string.
    // Title pointer must remain valid for the menu's lifetime.
    void init(Adafruit_SH1107& display, const char* title);

    // Clear all items and reset selection (for rebuild)
    void clear();

    // Add an action item (callback + optional int argument)
    void addAction(const char* text, MenuAction cb, int arg = 0);

    // Add a submenu item (clicking opens the child menu)
    void addSubmenu(const char* text, FruityMenu* sub);

    // Navigation — delegates to active submenu if one is open
    void scroll(int8_t delta);  // +1 = down, -1 = up, wraps around
    void click();               // activate selected item

    // Render the currently visible menu to the display
    void show();

    // Close the active submenu and reset parent selection to 0
    void closeSub();

private:
    static constexpr uint8_t MAX_ITEMS    = 10;
    static constexpr uint8_t PX_PER_LINE  = 14;
    static constexpr uint8_t TEXT_X       = 4;
    static constexpr uint8_t TEXT_Y_PAD   = 3;
    static constexpr uint8_t DISPLAY_H    = 128;
    static constexpr uint8_t DISPLAY_W    = 128;

    Adafruit_SH1107* _display  = nullptr;
    const char*      _title    = nullptr;
    FruityMenuItem   _items[MAX_ITEMS] = {};
    uint8_t          _count     = 0;
    uint8_t          _selection = 0;
    FruityMenu*      _activeSub = nullptr;

    void render();
};
