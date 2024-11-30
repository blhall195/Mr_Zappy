import time

from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import board
import busio
import digitalio
from laser_egismos import Laser
import rm3100

from screen_hardware import ZappyScreen


# Properties and functions for setting up main esp32 board
class MrZappy:
    def __init__(self):
        self.laser = None

        self.mag_sensor = None
        self.grav_sensor = None
        self.calib = None

        self.fire_button = board.D13
        self.button_1 = board.D24
        self.button_2 = board.D23
        self.button_3 = board.D19
        self.button_4 = board.D25

        self.init_io()

        self.screen = ZappyScreen()

    def init_io(self):
        # Initialise mag grav sensors
        i2c = board.I2C()

        self.mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20)
        self.grav_sensor = ISM330DHCX(i2c)

        # self.calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")
        # self.calib = Calibration.from_dict(CALIBRATION_DICT)

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

    def get_mag_readings(self):
        return self.mag_sensor.magnetic

    def get_grav_readings(self):
        return self.grav_sensor.acceleration
