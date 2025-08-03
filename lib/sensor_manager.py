import time
import board
import busio
import digitalio
from laser_egismos import Laser
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import rm3100
import adafruit_max1704x

class SensorManager:
    def __init__(self):
        # Enable laser power
        self.laser_power = digitalio.DigitalInOut(board.A2)
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
        self.mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20, cycle_count=3000)

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

    # Orientation stuff
    def get_grav(self):
        """Get the current grav reading from the accelerometer."""
        return self.grav_sensor.acceleration

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
