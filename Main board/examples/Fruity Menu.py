import os
import board
import displayio
import microcontroller
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297
from fruity_menu.Menu import Menu
import time

#Import config settings
from config import Config
CONFIG = Config()

# Import your button manager
from button_manager import ButtonManager  # adjust this to the actual filename
buttons = ButtonManager()

# --- Setup display ---
displayio.release_displays()
i2c = board.I2C()
display_bus = displayio.I2CDisplay(i2c, device_address=0x3d)
display = SH1107(
    display_bus,
    width=128,
    height=128,
    display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297,
    rotation=180
)


# --- Global variables ---
SETTINGS_FILE = "settings.toml"
laser_timeout = CONFIG.laser_timeout
auto_shutdown_timeout = CONFIG.auto_shutdown_timeout
anomaly_detection_bool = CONFIG.anomaly_detection
accuracy = "Medium"


# --- Helper functions ---
def save_readings():
    print("Save readings to non-volitile")
    #need to write code for this

def enter_calibration_mode():
    if "calibration_mode.txt" not in os.listdir("/"):
        try:
            with open("/calibration_mode.txt", "w") as f:
                f.write("1")
            microcontroller.reset()
        except OSError as e:
            print(f"Failed to write calibration_mode.txt: {e}")
            microcontroller.reset()
        print("Rebooting...")
    microcontroller.reset()


def anomaly_detection(value):
    global anomaly_detection_bool, menu
    anomaly_detection_bool = value
    print(f"Anom Det: {anomaly_detection_bool}")
    try:
        # Read the file
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()
        # Replace the laser_timeout line
        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                if line.strip().startswith("anomaly_detection"):
                    f.write(f"anomaly_detection = {value}\n")
                else:
                    f.write(line)
        print(f"anomaly_detection updated to {value} in {SETTINGS_FILE}")
    except Exception as e:
        print(f"Error updating settings file: {e}")
    # Rebuild menu to reflect new value
    menu = Menu(display, height=128, width=128, title="Main Menu")
    menu_structure = get_menu_structure()
    build_submenus(menu, menu_structure)
    menu.show_menu()


def set_accuracy(level):
    global accuracy, menu
    accuracy = level
    # Map levels to values
    level_map = {
        "Low":    {"stability_tolerance": 0.5, "stability_buffer_length": 4, "EMA_alpha": 0.8},
        "Medium": {"stability_tolerance": 0.25, "stability_buffer_length": 6, "EMA_alpha": 0.5},
        "High":   {"stability_tolerance": 0.1, "stability_buffer_length": 8, "EMA_alpha": 0.2},
        "Ultra":  {"stability_tolerance": 0.05, "stability_buffer_length": 10, "EMA_alpha": 0.1},
    }
    if level not in level_map:
        print(f"Unknown accuracy level: {level}")
        return
    updates = level_map[level]
    try:
        # Read existing file
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()
        # Write updated file
        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                stripped = line.strip()
                for key, value in updates.items():
                    if stripped.startswith(key):
                        f.write(f"{key} = {value}\n")
                        break
                else:
                    f.write(line)
        print(f"Accuracy updated to '{level}' in {SETTINGS_FILE}")
    except Exception as e:
        print(f"Error updating settings file: {e}")
    # Rebuild menu to reflect new value
    menu = Menu(display, height=128, width=128, title="Main Menu")
    menu_structure = get_menu_structure()
    build_submenus(menu, menu_structure)
    menu.show_menu()



def set_laser_timeout(value):
    global laser_timeout, menu
    laser_timeout = value
    print(f"Laser timeout set to {laser_timeout}")
    try:
        # Read the file
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()
        # Replace the laser_timeout line
        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                if line.strip().startswith("laser_timeout"):
                    f.write(f"laser_timeout = {value}\n")
                else:
                    f.write(line)
        print(f"laser_timeout updated to {value} in {SETTINGS_FILE}")
    except Exception as e:
        print(f"Error updating settings file: {e}")
    # Rebuild menu to reflect new value
    menu = Menu(display, height=128, width=128, title="Main Menu")
    menu_structure = get_menu_structure()
    build_submenus(menu, menu_structure)
    menu.show_menu()


def set_auto_shutdown_timeout(value):
    global auto_shutdown_timeout, menu
    auto_shutdown_timeout = value
    print(f"Auto Shutdown Timeout set to {auto_shutdown_timeout}")
    try:
        # Read the file
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()
        # Replace the laser_timeout line
        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                if line.strip().startswith("auto_shutdown_timeout"):
                    f.write(f"auto_shutdown_timeout = {value}\n")
                else:
                    f.write(line)
        print(f"auto_shutdown_timeout updated to {value} in {SETTINGS_FILE}")
    except Exception as e:
        print(f"Error updating settings file: {e}")
    # Rebuild menu to reflect new value
    menu = Menu(display, height=128, width=128, title="Main Menu")
    menu_structure = get_menu_structure()
    build_submenus(menu, menu_structure)
    menu.show_menu()


# ---Create Menu structure---
def get_menu_structure():
    return [
        ("Save_readings", save_readings),
        ("Enter Calibration", enter_calibration_mode),
        ("Anomoly Detec: " + ("On" if anomaly_detection_bool == "True" else "Off"), [
            ("On", lambda: anomaly_detection("True")),
            ("Off", lambda: anomaly_detection("False")),
        ]),
        ("Accuracy: " + (accuracy), [
            ("Low", lambda: set_accuracy("Low")),
            ("Medium", lambda: set_accuracy("Medium")),
            ("High", lambda: set_accuracy("High")),
            ("Ultra", lambda: set_accuracy("Ultra")),
        ]),
        ("Laser off: " + (str(laser_timeout//60) + " min" if laser_timeout >= 61 else str(laser_timeout) + " Sec"), [
            ("30 sec", lambda: set_laser_timeout(30)),
            ("60 sec", lambda: set_laser_timeout(60)),
            ("2 min", lambda: set_laser_timeout(120)),
            ("5 min", lambda: set_laser_timeout(300)),
            ("15 min", lambda: set_laser_timeout(900)),
            ("30 min", lambda: set_laser_timeout(1800)),
        ]),
        ("Shutdown: " + str(auto_shutdown_timeout//60) + " min", [
            ("5 min", lambda: set_auto_shutdown_timeout(300)),
            ("10 min", lambda: set_auto_shutdown_timeout(600)),
            ("15 min", lambda: set_auto_shutdown_timeout(900)),
            ("30 min", lambda: set_auto_shutdown_timeout(1800)),
            ("60 min", lambda: set_auto_shutdown_timeout(3600)),
            ("2 hr", lambda: set_auto_shutdown_timeout(7200)),
        ]),
        ("Extras", [
            ("Diagnostics", lambda: print("Running diagnostics...")),
            ("About", lambda: print("Device version 1.0")),
        ]),
    ]


# --- Function to recursively build menus ---
def build_submenus(parent_menu, items):
    for name, action_or_sub in items:
        if isinstance(action_or_sub, list):  # submenu
            sub = parent_menu.create_menu(name)
            build_submenus(sub, action_or_sub)
            parent_menu.add_submenu_button(name, sub)
        else:  # direct action
            parent_menu.add_action_button(name, action_or_sub)

# --- Utility to get the active menu/submenu ---
def active_menu(m):
    while getattr(m, "_activated_submenu", None):
        m = m._activated_submenu
    return m

# --- Build the menu ---
menu = Menu(display, height=128, width=128, title="Main Menu")
menu_structure = get_menu_structure()
build_submenus(menu, menu_structure)
menu.show_menu()


# --- Main loop ---
while True:
    buttons.update()
    m = active_menu(menu)

    # Scroll up
    if buttons.was_pressed("Button 1") and hasattr(m, "scroll"):
        m.scroll(-1)
        if hasattr(m, "show_menu"):
            m.show_menu()
        elif hasattr(m, "build_displayio_group"):
            display.root_group = m.build_displayio_group()

    # Scroll down
    if buttons.was_pressed("Button 2") and hasattr(m, "scroll"):
        m.scroll(1)
        if hasattr(m, "show_menu"):
            m.show_menu()
        elif hasattr(m, "build_displayio_group"):
            display.root_group = m.build_displayio_group()

    # Click / select
    if buttons.was_pressed("Button 3"):
        m.click()
        m = active_menu(menu)
        if hasattr(m, "show_menu"):
            m.show_menu()
        elif hasattr(m, "build_displayio_group"):
            display.root_group = m.build_displayio_group()

    time.sleep(0.01)
