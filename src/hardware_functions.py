import time

from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import board
import busio
import digitalio
from laser_egismos import Laser
from mag_cal.calibration import Calibration
import rm3100

CALIBRATION_DICT = {
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


class CustomAngleClass:
    def __init__(self, azimuth: float, inclination: float, roll: float):
        self.azimuth = azimuth
        self.inclination = inclination
        self.roll = roll


class MrZappy:
    def __init__(self):
        self.laser = None
        self.fire_button = None
        self.mag_sensor = None
        self.grav_sensor = None
        self.calib = None
        self.init_io()

    def init_io(self):
        # Initialise mag grav sensors
        i2c = board.I2C()

        self.mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20)
        self.grav_sensor = ISM330DHCX(i2c)

        # self.calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")
        self.calib = Calibration.from_dict(CALIBRATION_DICT)

        # Initialise Laser
        laser_power = digitalio.DigitalInOut(board.D5)
        laser_power.switch_to_output(True)
        time.sleep(0.25)
        # Delay before UART initialization (Keep this 0.25sec delay, the laser needs a moment to
        # wake up when not connected to a usb C)
        uart = busio.UART(board.TX, board.RX, baudrate=9600)
        time.sleep(0.25)
        # Delay after UART initialization (Keep this 0.25sec delay, the code doesn't work when
        # powered by a battery otherwise)
        self.laser = Laser(uart)
        self.laser.set_laser(True)

        # Initialise Fire Button
        self.fire_button = digitalio.DigitalInOut(board.D13)
        self.fire_button.direction = digitalio.Direction.INPUT
        self.fire_button.pull = digitalio.Pull.UP

    def get_calibrated_angles(self):
        azimuth, inclination, roll = self.calib.get_angles(
            self.mag_sensor.magnetic, self.grav_sensor.acceleration
        )
        return CustomAngleClass(azimuth, inclination, roll)
