from adafruit_bitmap_font import bitmap_font
from adafruit_display_text import bitmap_label as label
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297
import analogio
import board
import displayio
import time

# Initialize battery monitoring
VBAT_PIN = board.VOLTAGE_MONITOR  # Adjust based on your board, if needed
vbat_pin = analogio.AnalogIn(VBAT_PIN)


def read_battery_percentage(vbat_pin):
    raw_value = vbat_pin.value  # Read the voltage
    if raw_value > 40000:
        bat_percentage = 100
    else:
        bat_percentage = ((raw_value-34000)/6000)*100  # Convert to battery percentage
        bat_percentage = round(bat_percentage, 1)
    return bat_percentage


class ZappyScreen:
    def __init__(self):
        self.BT_label = None
        self.battery_label = None
        self.small_square_4 = None
        self.azimuth_label = None
        self.black_background = None
        self.inclination_label = None
        self.distance_label = None
        self.font = None
        self.initialise_screen()
        self.prev_azimuth = 0
        self.prev_inclination = 0
        self.screen_off = False

    def initialise_screen(self):
        # Initialise display

        displayio.release_displays()  # Release any existing displays
        self.font = bitmap_font.load_font("lib/fonts/terminal.bdf")  # Load font

        i2c = board.I2C()
        display_bus = displayio.I2CDisplay(i2c, device_address=0x3D)
        self.display = SH1107(
            display_bus,
            width=128,
            height=128,
            display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297,
            rotation=90,
        )
        white_palette = displayio.Palette(1)
        white_palette[0] = 0xFFFFFF
        black_palette = displayio.Palette(1)
        black_palette[0] = 0x000000

        color_bitmap = displayio.Bitmap(128, 128, 1)
        small_square_1 = displayio.TileGrid(
            displayio.Bitmap(32, 15, 1), pixel_shader=white_palette, x=90, y=0
        )
        small_square_2 = displayio.TileGrid(
            displayio.Bitmap(30, 13, 1), pixel_shader=black_palette, x=91, y=1
        )
        small_square_3 = displayio.TileGrid(
            displayio.Bitmap(3, 6, 1), pixel_shader=white_palette, x=122, y=4
        )
        self.small_square_4 = displayio.TileGrid(
            displayio.Bitmap(30, 13, 1),
            pixel_shader=white_palette,
            x=91,
            y=1,
        )
        self.black_background = displayio.TileGrid(
            displayio.Bitmap(128, 128, 1), pixel_shader=black_palette, x=0, y=0
        )

        # Create labels
        self.distance_label = label.Label(self.font, scale=3, text="0m", x=0, y=44)
        self.azimuth_label = label.Label(self.font, scale=3, text="0.0°", x=0, y=78)
        self.inclination_label = label.Label(self.font, scale=3, text="0.0°", x=0, y=112)
        battery_percentage = read_battery_percentage(vbat_pin)
        self.battery_label = label.Label(self.font, scale=2, text=f"{battery_percentage:.0f}%", x=40, y=6)
        self.BT_label = label.Label(self.font, scale=2, text="BT", x=0, y=6)

        splash = displayio.Group()
        self.display.root_group = splash  # Set the splash group as the root group

        # Add labels to the splash group
        splash.append(self.black_background)
        splash.append(self.distance_label)
        splash.append(self.azimuth_label)
        splash.append(self.inclination_label)
        splash.append(self.battery_label)
        splash.append(self.BT_label)
        splash.append(small_square_1)
        splash.append(small_square_2)
        splash.append(small_square_3)
        splash.append(self.small_square_4)


    def _update_angle_if_different(
        self, prev_value: float, new_value: float, tolerance: float, label: float
    ) -> float:
        difference = abs(new_value - prev_value)
        if difference > tolerance:
            label.text = f"{round(new_value, 1)}°"
            return new_value
        else:
            return prev_value

    def update_angles(self, azimuth: float, inclination: float, force_update: bool):
        if force_update:
            tolerance = 0
        else:
            tolerance = 1

        self.prev_inclination = self._update_angle_if_different(
            self.prev_inclination, inclination, tolerance, self.inclination_label
        )
        self.prev_azimuth = self._update_angle_if_different(
            self.prev_azimuth, azimuth, tolerance, self.azimuth_label
        )

    def update_battery_display(self):
        """Updates the battery label to show the current battery percentage."""
        if hasattr(self, "battery_label") and hasattr(self, "font"):
            # Read the current battery percentage
            battery_percentage = read_battery_percentage(vbat_pin)

            # Calculate the width of the bar based on the battery percentage
            bar_width = int(battery_percentage / 100 * 30)

            # Update the existing bitmap rather than creating a new one
            for x in range(30):  # Assuming the bitmap width is 30 pixels
                for y in range(13):  # Assuming the bitmap height is 13 pixels
                    if x < bar_width:
                        self.small_square_4.bitmap[x, y] = 0  # Empty space#
                    else:
                        self.small_square_4.bitmap[x, y] = 1  # Fill the bar

            # Update the label's text based on the current battery percentage
            if battery_percentage == 100:
                self.battery_label.text = f"{battery_percentage:.0f}%"
                self.battery_label.x = 40  # Adjust position for full percentage display
            else:
                self.battery_label.text = f"{battery_percentage:.0f}%"
                self.battery_label.x = 50  # Adjust for shorter percentage values
        else:
            # Error message if battery_label or font isn't set up properly
            print("Error: Battery label or font is not initialized.")

    def turn_off_screen(self):
        """Turns off the screen by overlaying the black background."""
        if hasattr(self, "display") and hasattr(self, "black_background"):
            root_group = self.display.root_group

            # Add the black background (if not already added)
            if not self.screen_off:
                if root_group and self.black_background not in root_group:
                    root_group.append(self.black_background)  # Add black background
                self.display.root_group = root_group  # Refresh the display
                self.screen_off = True  # Update the state
                time.sleep(1)#sleeps for a moment to give the screen time to turn off
                print("Screen has been turned off.")
            else:
                print("Screen is already turned off.")
        else:
            print("Error: Display or black_background is not initialized.")

    def turn_on_screen(self):
        """Turns the screen back on by removing the black background."""
        if hasattr(self, "display") and hasattr(self, "black_background"):
            root_group = self.display.root_group

            # Remove the black background (if it exists)
            if self.screen_off:
                if root_group and self.black_background in root_group:
                    root_group.remove(self.black_background)  # Remove black background
                self.display.root_group = root_group  # Refresh the display
                self.screen_off = False  # Update the state
                print("Screen has been turned back on.")
            else:
                print("Screen is already turned on.")
        else:
            print("Error: Display or black_background is not initialized.")



