import time
import board
import busio
import digitalio
from laser_egismos import Laser
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import rm3100

class SensorManager:
    def __init__(self):
        # Enable laser power
        self.laser_power = digitalio.DigitalInOut(board.D5)
        self.laser_power.switch_to_output(False)
        time.sleep(0.3)
        self.laser_power.switch_to_output(True)
        time.sleep(0.1)

        # Initialize laser over UART
        self.uart = busio.UART(board.TX, board.RX, baudrate=9600)
        self.laser = Laser(self.uart)

        # Initialize I2C and accelerometer
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.grav_sensor = ISM330DHCX(i2c)

        # Initialize mag_sensor
        self.mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20)

    def get_distance(self):
        """Get the current distance reading from the laser, with error handling."""
        try:
            return self.laser.distance
        except Exception as e:
            print(f"Error reading distance: {e}")
            return "Err"

    def get_grav(self):
        """Get the current grav reading from the accelerometer."""
        return self.grav_sensor.acceleration

    def get_mag(self):
        """Get the current mag reading from the magnetometer."""
        return self.mag_sensor.magnetic

