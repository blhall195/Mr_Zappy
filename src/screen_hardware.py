import displayio
import board
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297

from adafruit_bitmap_font import bitmap_font
from adafruit_display_text import bitmap_label as label
import analogio

# Initialize battery monitoring
VBAT_PIN = board.VOLTAGE_MONITOR  # Adjust based on your board, if needed
vbat_pin = analogio.AnalogIn(VBAT_PIN)

def read_battery_percentage(vbat_pin):
    raw_value = vbat_pin.value  # Read the voltage
    voltage = ((raw_value / 65535) * 3.3) * 2
    bat_percentage = (voltage / 4.2) * 100  # Convert to battery percentage
    return bat_percentage

class ZappyScreen:
    def __init__(self):
        self.font = None
        self.initialise_screen()


    def initialise_screen(self):
        # Initialise display

        # Release any existing displays
        displayio.release_displays()

        self.font = bitmap_font.load_font("lib/fonts/terminal.bdf")  # Load font

        i2c = board.I2C()
        display_bus = displayio.I2CDisplay(i2c, device_address=0x3D)
        display = SH1107(
            display_bus,
            width=128,
            height=128,
            display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297,
            rotation=90,
        )

        displayio.release_displays()  # Release any existing displays
        self.font = bitmap_font.load_font("lib/fonts/terminal.bdf")  # Load font

        i2c = board.I2C()
        display_bus = displayio.I2CDisplay(i2c, device_address=0x3D)
        display = SH1107(
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
        small_square_4 = displayio.TileGrid(
            displayio.Bitmap(int(read_battery_percentage(vbat_pin) / 100 * 30), 13, 1),
            pixel_shader=white_palette,
            x=91,
            y=1,
        )

        # Create labels
        self.distance_label = label.Label(self.font, scale=3, text="0m", x=0, y=44)
        self.azimuth_label = label.Label(self.font, scale=3, text="0.0°", x=0, y=78)
        self.inclination_label = label.Label(self.font, scale=3, text="0.0°", x=0, y=112)
        battery_percentage = read_battery_percentage(vbat_pin)
        self.battery_label = label.Label(self.font, scale=2, text=f"{battery_percentage:.0f}%", x=50, y=6)
        self.BT_label = label.Label(self.font, scale=2, text="BT", x=0, y=6)

        splash = displayio.Group()
        display.root_group = splash  # Set the splash group as the root group

        # Add labels to the splash group
        splash.append(self.distance_label)
        splash.append(self.azimuth_label)
        splash.append(self.inclination_label)
        splash.append(self.battery_label)
        splash.append(self.BT_label)
        splash.append(small_square_1)
        splash.append(small_square_2)
        splash.append(small_square_3)
        splash.append(small_square_4)