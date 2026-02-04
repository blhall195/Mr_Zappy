import os
import warnings
warnings.simplefilter('ignore')

# Delete old menu mode flag if present
if "menu_mode.txt" in os.listdir("/"):
    try:
        os.remove("/menu_mode.txt")
    except OSError as e:
        print("Error deleting file:", e)

import board
import digitalio
import displayio
import microcontroller
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297
from fruity_menu.Menu import Menu
import time

# Setup power pin for LTC2952 shutdown
pwr_pin = digitalio.DigitalInOut(board.A2)
pwr_pin.direction = digitalio.Direction.OUTPUT
pwr_pin.value = True  # Keep power on

#Import config settings
from config import Config
CONFIG = Config()

# Import your button manager
from button_manager import ButtonManager  # adjust this to the actual filename
buttons = ButtonManager()

# --- Global variables ---
SETTINGS_FILE = "settings.toml"
laser_timeout = CONFIG.laser_timeout
auto_shutdown_timeout = CONFIG.auto_shutdown_timeout
accuracy = CONFIG.accuracy
anomaly_detection_bool = CONFIG.anomaly_detection
last_activity_time = time.monotonic()



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
    anomaly_detection_bool = value.lower() in ("true", "1", "yes")
    try:
        # Read the file
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()
        # Replace the anomaly_detection line
        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                if line.strip().startswith("anomaly_detection"):
                    # Keep quotes if value is a string
                    if isinstance(value, str):
                        f.write(f'anomaly_detection = "{value}"\n')
                    else:
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

# --- Delete pending readings ---
def delete_pending_readings():
    file_path = "pending_readings.txt"
    if file_path in os.listdir("/"):
        try:
            os.remove(file_path)
            print(f"{file_path} deleted.")
        except OSError as e:
            print(f"Failed to delete {file_path}: {e}")
    else:
        print(f"{file_path} does not exist.")

    # Go back to root menu
    go_to_root()



def set_laser_timeout(value):
    global laser_timeout, menu
    laser_timeout = value
    print(f"Laser timeout set to {laser_timeout}")
    try:
        updated = False
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()

        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                stripped = line.strip()
                if stripped.startswith("laser_timeout"):
                    f.write(f"laser_timeout = {value}\n")
                    updated = True
                else:
                    f.write(line)
            if not updated:  # add it if not present
                f.write(f"laser_timeout = {value}\n")

        print(f"laser_timeout updated to {value} in {SETTINGS_FILE}")

    except Exception as e:
        print(f"Error updating settings file: {e}")

    # Rebuild menu
    menu = Menu(display, height=128, width=128, title="Main Menu")
    menu_structure = get_menu_structure()
    build_submenus(menu, menu_structure)
    menu.show_menu()


def set_auto_shutdown_timeout(value):
    global auto_shutdown_timeout, menu
    auto_shutdown_timeout = value
    print(f"Auto Shutdown Timeout set to {auto_shutdown_timeout}")
    try:
        updated = False
        with open(SETTINGS_FILE, "r") as f:
            lines = f.readlines()

        with open(SETTINGS_FILE, "w") as f:
            for line in lines:
                stripped = line.strip()
                if stripped.startswith("auto_shutdown_timeout"):
                    f.write(f"auto_shutdown_timeout = {value}\n")
                    updated = True
                else:
                    f.write(line)
            if not updated:  # add it if not present
                f.write(f"auto_shutdown_timeout = {value}\n")

        print(f"auto_shutdown_timeout updated to {value} in {SETTINGS_FILE}")

    except Exception as e:
        print(f"Error updating settings file: {e}")

    # Rebuild menu
    menu = Menu(display, height=128, width=128, title="Main Menu")
    menu_structure = get_menu_structure()
    build_submenus(menu, menu_structure)
    menu.show_menu()

def update_display(menu_obj):
    """
    Update the display to show the given menu object.
    Handles both show_menu() and build_displayio_group() cases.
    """
    if hasattr(menu_obj, "show_menu"):
        menu_obj.show_menu()
    elif hasattr(menu_obj, "build_displayio_group"):
        display.root_group = menu_obj.build_displayio_group()

def go_to_root():
    # walk up to root menu
    m = menu
    while getattr(m, "_activated_submenu", None):
        m._activated_submenu = None
        m = m._activated_submenu if m._activated_submenu else m
    m.show_menu()


def exit_menu():
    microcontroller.reset()

# ---Create Menu structure---
def get_menu_structure():
    return [
        ("Enter Calibration", [
            ("No", lambda: go_to_root()),  # Go back to main menu
            ("Yes", enter_calibration_mode),
        ]),
        ("Anomoly Detec: " + ("On" if anomaly_detection_bool else "Off"), [
            ("On", lambda: anomaly_detection("True")),
            ("Off", lambda: anomaly_detection("False")),
        ]),
        ("Delete saved shots", [
            ("No", lambda: go_to_root()),  # Go back to main menu
            ("Yes DELETE them", delete_pending_readings),
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
        ("Exit", exit_menu),
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
        last_activity_time = time.monotonic()
        m.scroll(-1)
        if hasattr(m, "show_menu"):
            m.show_menu()
        elif hasattr(m, "build_displayio_group"):
            display.root_group = m.build_displayio_group()

    # Scroll down
    if buttons.was_pressed("Button 2") and hasattr(m, "scroll"):
        last_activity_time = time.monotonic()
        m.scroll(1)
        if hasattr(m, "show_menu"):
            m.show_menu()
        elif hasattr(m, "build_displayio_group"):
            display.root_group = m.build_displayio_group()

    # Click / select
    if buttons.was_pressed("Button 3"):
        last_activity_time = time.monotonic()
        m.click()
        m = active_menu(menu)
        if hasattr(m, "show_menu"):
            m.show_menu()
        elif hasattr(m, "build_displayio_group"):
            display.root_group = m.build_displayio_group()

    # Auto shutdown timeout
    if time.monotonic() - last_activity_time > auto_shutdown_timeout:
        print("Inactivity timeout, shutting down device")
        pwr_pin.value = False

    # Power off (Button 4)
    if buttons.was_pressed("Button 4"):
        pwr_pin.value = False

    time.sleep(0.01)
