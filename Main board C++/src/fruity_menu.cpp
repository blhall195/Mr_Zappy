#include "fruity_menu.h"

void FruityMenu::init(Adafruit_SH1107& display, const char* title) {
    _display = &display;
    _title   = title;
    clear();
}

void FruityMenu::clear() {
    _count     = 0;
    _selection = 0;
    _activeSub = nullptr;
    memset(_items, 0, sizeof(_items));
}

void FruityMenu::addAction(const char* text, MenuAction cb, int arg) {
    if (_count >= MAX_ITEMS) return;
    FruityMenuItem& item = _items[_count++];
    item.text      = text;
    item.action    = cb;
    item.actionArg = arg;
    item.submenu   = nullptr;
}

void FruityMenu::addSubmenu(const char* text, FruityMenu* sub) {
    if (_count >= MAX_ITEMS) return;
    FruityMenuItem& item = _items[_count++];
    item.text      = text;
    item.action    = nullptr;
    item.actionArg = 0;
    item.submenu   = sub;
}

void FruityMenu::scroll(int8_t delta) {
    if (_activeSub) {
        _activeSub->scroll(delta);
        return;
    }
    if (_count == 0) return;

    if (delta > 0) {
        _selection = (_selection + 1) % _count;
    } else if (delta < 0) {
        if (_selection == 0) {
            _selection = _count - 1;
        } else {
            _selection--;
        }
    }
}

void FruityMenu::click() {
    if (_activeSub) {
        _activeSub->click();
        return;
    }
    if (_count == 0) return;

    FruityMenuItem& item = _items[_selection];
    if (item.submenu) {
        // Open submenu, reset its selection to top
        _activeSub = item.submenu;
        _activeSub->_selection = 0;
    } else if (item.action) {
        item.action(item.actionArg);
    }
}

void FruityMenu::show() {
    if (_activeSub) {
        _activeSub->show();
        return;
    }
    render();
    _display->display();
}

void FruityMenu::closeSub() {
    _activeSub = nullptr;
    _selection = 0;
}

// ── Private rendering ────────────────────────────────────────────────

void FruityMenu::render() {
    if (!_display) return;

    _display->clearDisplay();
    _display->setTextSize(1);

    uint8_t y = 0;

    // Title bar — inverted (white background, black text)
    _display->fillRect(0, y, DISPLAY_W, PX_PER_LINE, SH110X_WHITE);
    _display->setTextColor(SH110X_BLACK);
    _display->setCursor(TEXT_X, y + TEXT_Y_PAD);
    _display->print(_title);
    y += PX_PER_LINE;

    // Calculate pagination
    uint8_t maxVisible = (DISPLAY_H - y) / PX_PER_LINE;
    if (maxVisible == 0) return;

    uint8_t page     = _selection / maxVisible;
    uint8_t startIdx = page * maxVisible;

    for (uint8_t i = 0; i < maxVisible && (startIdx + i) < _count; i++) {
        uint8_t idx = startIdx + i;

        if (idx == _selection) {
            // Selected item — inverted (white background, black text)
            _display->fillRect(0, y, DISPLAY_W, PX_PER_LINE, SH110X_WHITE);
            _display->setTextColor(SH110X_BLACK);
        } else {
            // Normal item — white text on black background
            _display->setTextColor(SH110X_WHITE);
        }

        _display->setCursor(TEXT_X, y + TEXT_Y_PAD);
        _display->print(_items[idx].text);
        y += PX_PER_LINE;
    }
}
