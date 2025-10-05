import board
import displayio
import terminalio
from adafruit_bitmap_font import bitmap_font
from adafruit_display_text import bitmap_label as label
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297
import random
import time

class DisplayManager:
    def __init__(self):
        displayio.release_displays()
        self.font = bitmap_font.load_font("lib/fonts/terminal.bdf")
        i2c = board.I2C()
        display_bus = displayio.I2CDisplay(i2c, device_address=0x3d)
        self.display = SH1107(display_bus, width=128, height=128, display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297, rotation=180)

    def display_screen_initialise(self):
        # Swapped black and white palettes
        self.black_palette = displayio.Palette(1)
        self.black_palette[0] = 0x000000  # Black for filled portion of battery
        self.white_palette = displayio.Palette(1)
        self.white_palette[0] = 0xFFFFFF  # White for empty portion of battery

        self.battery_outline = displayio.TileGrid(displayio.Bitmap(32, 15, 1), pixel_shader=self.white_palette, x=90, y=0)
        self.battery_inner = displayio.TileGrid(displayio.Bitmap(30, 13, 1), pixel_shader=self.black_palette, x=91, y=1)
        self.battery_tip = displayio.TileGrid(displayio.Bitmap(3, 6, 1), pixel_shader=self.white_palette, x=122, y=4)

        # Fixed size battery fill bitmap
        self.battery_fill_bitmap = displayio.Bitmap(30, 13, 1)  # Always the same size
        self.battery_fill = displayio.TileGrid(self.battery_fill_bitmap, pixel_shader=self.white_palette, x=91, y=1)

        # Create labels
        self.distance_label = label.Label(self.font, scale=3, text="0m", x=0, y=44)
        self.azimuth_label = label.Label(self.font, scale=3, text="0.0°", x=0, y=78)
        self.inclination_label = label.Label(self.font, scale=3, text="0.0°", x=0, y=112)
        self.BT_label = label.Label(self.font, scale=2, text="BT", x=0, y=6)
        self.BT_number = label.Label(self.font, scale=2, text="0", x=39, y=6)

        # Create display group
        self.splash = displayio.Group()
        self.splash.append(self.distance_label)
        self.splash.append(self.azimuth_label)
        self.splash.append(self.inclination_label)
        self.splash.append(self.BT_label)
        self.splash.append(self.BT_number)

        # Add battery symbol elements
        self.splash.append(self.battery_outline)
        self.splash.append(self.battery_inner)
        self.splash.append(self.battery_tip)
        self.splash.append(self.battery_fill)

        self.display.root_group = self.splash

    def update_sensor_readings(self, distance, azimuth, inclination):
        # Ensure distance is numeric before formatting
        if isinstance(distance, (int, float)):
            if distance == 0:
                self.distance_label.text = ""
            else:
                self.distance_label.text = f"{distance:.2f}m"
        else:
            # Handles any string value, including anomaly messages
            self.distance_label.text = str(distance)

        self.azimuth_label.text = f"{azimuth:.1f}°"
        self.inclination_label.text = f"{inclination:.1f}°"


    def update_distance(self, distance):
        """Update the distance label on the display. Accepts numeric or string values."""
        if isinstance(distance, (int, float)):
            if distance == 0:
                self.distance_label.text = ""
            else:
                self.distance_label.text = f"{distance:.2f}m"
        else:
            self.distance_label.text = str(distance)

    def update_azimuth(self, azimuth):
        self.azimuth_label.text = f"{round(azimuth, 1)}°"

    def update_inclination(self, inclination):
        self.inclination_label.text = f"{round(inclination, 1)}°"

    def update_BT_number(self, number: int = 0):
        """Updates the Bluetooth symbol with a number (e.g., 'BT1', 'BT2', etc.).
        Hides the number if it's 0."""
        if number == 0:
            self.BT_number.text = ""  # Hide the number
        else:
            self.BT_number.text = f"{number}"


    def update_BT_label(self, state: bool = False):
        """Turns the Bluetooth symbol on or off."""
        if state:
            self.BT_label.text = "BT"  # Show BT symbol
        else:
            self.BT_label.text = "  "  # Hide BT symbol

    def update_battery(self, battery_percentage):
        # Scale battery fill width
        fill_width = int((battery_percentage / 100) * 30)  # Scale to 30px width
        # Update the pixels of the existing bitmap directly
        for x in range(30):  # Iterate over the 30 width pixels
            for y in range(13):  # Iterate over the height of the battery
                if x < fill_width:
                    self.battery_fill_bitmap[x, y] = 0  # Black for filled portion
                else:
                    self.battery_fill_bitmap[x, y] = 1  # White for empty portion

    def release_displays(self):
        """Releases the display resources."""
        displayio.release_displays()


    def blank_screen(self):
        # Clears the display
        splash = displayio.Group()

        black_bitmap = displayio.Bitmap(self.display.width, self.display.height, 1)
        black_palette = displayio.Palette(1)
        black_palette[0] = 0x000000  # Black

        black_tilegrid = displayio.TileGrid(black_bitmap, pixel_shader=black_palette)
        splash.append(black_tilegrid)

        self.display.root_group = splash
        self.display.refresh()

    def show_starting_menu(self):
        """Clears the screen and displays 'Starting Calibration'."""
        # First, blank the screen
        self.blank_screen()

        # Create a new group for the message
        splash = displayio.Group()

        # Create label for "Starting Calibration"
        text = "Starting\nMenu\nIf this takes longer\n than 10 seconds\n turn the device\n on/off again\n and reattempt"
        calibration_label = label.Label(
            self.font,
            text=text,
            scale=1,
            x=0,  # Adjust horizontal position
            y=10  # Roughly vertical center
        )
        splash.append(calibration_label)

        # Set the display to show this new group
        self.display.root_group = splash
        self.display.refresh()

    def show_initialising_message(self):
        """Clears the screen and displays 'Device Initialising\nPlease wait...'"""
        self.blank_screen()  # Clear the screen first

        # Create a new group for the message
        splash = displayio.Group()

        # Create label for initialisation message
        text = "Device Initialising\nPlease wait..."
        init_label = label.Label(
            self.font,
            text=text,
            scale=1,
            x=0,
            y=10
        )
        splash.append(init_label)

        # Show the group on display
        self.display.root_group = splash
        self.display.refresh()



