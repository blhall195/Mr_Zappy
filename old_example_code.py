import time

from adafruit_bitmap_font import bitmap_font
from adafruit_display_text import bitmap_label as label
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import analogio
import board
import busio
import digitalio
import displayio
from laser_egismos import Laser
from mag_cal.calibration import Calibration
import rm3100
import terminalio

# initialize mag grav sensors--------------------------------------------------------------------------------------------
i2c = board.I2C()
mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20)
grav_sensor = ISM330DHCX(i2c)
calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")

# Initalise Laser--------------------------------------------------------------------------------------------------------
laser_power = digitalio.DigitalInOut(board.D5)
laser_power.switch_to_output(True)
time.sleep(
    0.25
)  # Delay before UART initialization (Keep this 0.25sec delay, the laser needs a moment to wake up when not connected to a usb C)
uart = busio.UART(board.TX, board.RX, baudrate=9600)
time.sleep(
    0.25
)  # Delay after UART initialization (Keep this 0.25sec delay, the code doesn't work when powered by a battery otherwise)
laser = Laser(uart)
laser.set_laser(True)

# Initalise Fire Button--------------------------------------------------------------------------------------------------
fire_button = digitalio.DigitalInOut(board.D13)
fire_button.direction = digitalio.Direction.INPUT
fire_button.pull = digitalio.Pull.UP


# Initalise display------------------------------------------------------------------------------------------------------
displayio.release_displays()  # Release any existing displays
font = bitmap_font.load_font("lib/fonts/terminal.bdf")  # Load font
i2c = board.I2C()
display_bus = displayio.I2CDisplay(i2c, device_address=0x3D)
display = SH1107(
    display_bus,
    width=128,
    height=128,
    display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297,
    rotation=90,
)

# Initialize battery monitoring
VBAT_PIN = board.VOLTAGE_MONITOR  # Adjust based on your board, if needed
vbat_pin = analogio.AnalogIn(VBAT_PIN)


def read_battery_percentage(vbat_pin):
    raw_value = vbat_pin.value  # Read the voltage
    voltage = ((raw_value / 65535) * 3.3) * 2
    bat_percentage = (voltage / 4.2) * 100  # Convert to battery percentage
    return bat_percentage


# perform calibration before screen loads
print(
    "Carry out the initial device calibration by pressing the fire button while rotating the device around as many different positions as possible."
)


def take_calibration_readings(button, mag_sensor, grav_sensor, mag_array: list, grav_array: list):
    previous_state = button.value
    iteration = 0
    while iteration < 20:
        current_state = not button.value
        if current_state and current_state != previous_state:
            time.sleep(0.01)
            iteration += 1
            # Collect readings from sensors
            mag_array.append(mag_sensor.magnetic)
            grav_array.append(grav_sensor.acceleration)
            print(iteration)
        previous_state = current_state
        time.sleep(0.1)


mag_array = []
grav_array = []

# take_calibration_readings(fire_button, mag_sensor, grav_sensor, mag_array, grav_array)
# mag_accuracy, grav_accuracy = calib.fit_ellipsoid(mag_array, grav_array)
# runs = calib.find_similar_shots(mag_array, grav_array)
# paired_data = [(mag_array[a:b], grav_array[a:b]) for a, b in runs]
# calib.fit_to_axis(paired_data)
# print(runs)

# print('mag_acc: ', mag_accuracy)
# print('grav_acc: ', grav_accuracy)
# paired_data = list(zip(mag_array, grav_array))
# calibration_dict = calib.as_dict()
# print(calibration_dict)

calibration_dict = {
    "mag": {
        "axes": "-X-Y-Z",
        "transform": [
            [0.0231683, -4.50966e-05, -0.000208465],
            [-4.50968e-05, 0.0233006, -2.46289e-05],
            [-0.000208464, -2.46296e-05, 0.0231333],
        ],
        "centre": [0.407859, -1.9058, 2.11295],
        "rbfs": [],
        "field_avg": None,
        "field_std": None,
    },
    "dip_avg": None,
    "grav": {
        "axes": "-Y-X+Z",
        "transform": [
            [0.101454, 0.00155312, -0.000734401],
            [0.00155312, 0.101232, 0.00149594],
            [-0.000734397, 0.00149594, 0.0987455],
        ],
        "centre": [0.364566, -0.0656354, 0.193454],
        "rbfs": [],
        "field_avg": None,
        "field_std": None,
    },
}
calib = calib.from_dict(calibration_dict)


# Reitialise display so the display goes to readout mode, this chunky code needs changing into a class probably...
displayio.release_displays()  # Release any existing displays
font = bitmap_font.load_font("lib/fonts/terminal.bdf")  # Load font
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
distance_label = label.Label(font, scale=3, text="0m", x=0, y=44)
azimuth_label = label.Label(font, scale=3, text="0.0°", x=0, y=78)
inclination_label = label.Label(font, scale=3, text="0.0°", x=0, y=112)
battery_percentage = read_battery_percentage(vbat_pin)
battery_label = label.Label(font, scale=2, text=f"{battery_percentage:.0f}%", x=50, y=6)
BT_label = label.Label(font, scale=2, text="BT", x=0, y=6)

splash = displayio.Group()
display.root_group = splash  # Set the splash group as the root group

# Add labels to the splash group
splash.append(distance_label)
splash.append(azimuth_label)
splash.append(inclination_label)
splash.append(battery_label)
splash.append(BT_label)
splash.append(small_square_1)
splash.append(small_square_2)
splash.append(small_square_3)
splash.append(small_square_4)


# Start loop ------------------------------------------------------------------------------------------------------------
paused = False
previous_state = fire_button.value

laser.set_laser(True)
laser_on_flag = True

while True:
    current_state = not fire_button.value  # Active Low: pressed is False, not pressed is True

    if current_state and current_state != previous_state:
        paused = not paused  # Toggle the paused state

        if paused:  # If paused state is entered (button pressed to pause)
            # Fetch and update laser distance only once
            distance = laser.distance / 100
            distance_label.text = f"{distance}m"
            time.sleep(0.1)  # Small delay to prevent bouncing
            laser_on_flag = True
            battery_percentage = read_battery_percentage(vbat_pin)
            battery_label.text = f"{battery_percentage:.0f}%"
            print(distance)

    if not paused:
        if laser_on_flag:
            distance_label.text = ""
            laser.set_laser(True)
            laser_on_flag = False

        # Fetch and update azimuth and inclination continuously when not paused
        azimuth, inclination, roll = calib.get_angles(mag_sensor.magnetic, grav_sensor.acceleration)
        azimuth_label.text = f"{round(azimuth, 1)}°"
        inclination_label.text = f"{round(inclination, 1)}°"

    previous_state = current_state
    time.sleep(0.1)
