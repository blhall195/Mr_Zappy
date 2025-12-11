import warnings
warnings.simplefilter('ignore')
import time
import board
import busio
import digitalio
from laser_egismos import Laser
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import rm3100
import adafruit_max1704x
from collections import deque

class SensorManager:
    def __init__(self):
        # Initialize laser over UART
        self.uart = busio.UART(board.TX, board.RX, baudrate=9600)
        self.laser = Laser(self.uart)

        # Initialize I2C and accelerometer
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.grav_sensor = ISM330DHCX(i2c)
        self._grav_history = []  # use a plain list
        self._grav_maxlen = 3    # smoothing window size

        # Initialize mag_sensor
        self.mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20, cycle_count=400)

        # Initialize bat_sensor
        self.max17 = adafruit_max1704x.MAX17048(i2c)

        # List to store the last 5 magnetometer readings
        self.mag_readings = []

    def get_distance(self):
        """Get the current distance reading from the laser, with error handling."""
        try:
            return self.laser.distance
        except Exception as e:
            print(f"Error reading distance: {e}")
            return "Err"

    def set_laser(self, value: bool):
        """Enable or disable the laser emitter."""
        try:
            return self.laser.set_laser(value)
        except Exception as e:
            print(f"Error setting laser state: {e}")
            return "Err"

    def set_buzzer(self, value: bool):
        """Enable or disable the buzzer."""
        try:
            return self.laser.set_buzzer(value)
        except Exception as e:
            print(f"Error setting buzzer state: {e}")
            return None

    def get_grav(self):
        """Return smoothed gravity reading using a rolling average."""
        raw = self.grav_sensor.acceleration
        if raw is None:
            return (0.0, 0.0, 0.0)

        # Add new reading to history
        self._grav_history.append(raw)

        # Enforce fixed-length history
        if len(self._grav_history) > self._grav_maxlen:
            self._grav_history.pop(0)  # remove oldest

        # Calculate average
        gx_sum = gy_sum = gz_sum = 0.0
        for gx, gy, gz in self._grav_history:
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz

        n = len(self._grav_history)
        return (gx_sum / n, gy_sum / n, gz_sum / n)

    def get_mag(self):
        try:
            reading = self.mag_sensor.magnetic  # Read [x, y, z]
            return reading
        except Exception as e:
            print(f"Error reading magnetometer: {e}")
            return [0, 0, 0]

    def get_bat(self):
        """Get the current battery percentage."""
        return self.max17.cell_percent
