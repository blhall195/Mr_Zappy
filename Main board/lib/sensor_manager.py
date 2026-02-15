import warnings
warnings.simplefilter('ignore')
import time
import board
import busio
import digitalio
from laser_egismos import Laser, LaserError
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
import rm3100
import adafruit_max1704x
import math

class SensorManager:
    def __init__(self):
        # Initialize laser over UART
        self.uart = busio.UART(board.TX, board.RX, baudrate=9600)
        self.laser = Laser(self.uart)

        # Initialize I2C and accelerometer
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.grav_sensor = ISM330DHCX(i2c)
        self._grav_filtered = None  # complementary filter state (initialized on first read)
        self._grav_last_time = None
        self._grav_alpha_low = 0.03    # when still: heavy smoothing
        self._grav_alpha_high = 0.8    # when moving: track fast
        self._gyro_thresh_low = 0.05   # rad/s — below this, device is stationary
        self._gyro_thresh_high = 0.5   # rad/s — above this, full tracking
        self._gyro_bias = [0.0, 0.0, 0.0]  # estimated gyro bias, learned when stationary
        self._gyro_bias_alpha = 0.001       # slow learning rate for bias estimation

        # Initialize mag_sensor with DRDY pin for non-blocking reads
        self.mag_drdy = digitalio.DigitalInOut(board.D9)
        self.mag_drdy.direction = digitalio.Direction.INPUT
        self.mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20, cycle_count=400, drdy_pin=self.mag_drdy)

        # Initialize bat_sensor
        self.max17 = adafruit_max1704x.MAX17048(i2c)

        # List to store the last 5 magnetometer readings
        self.mag_readings = []

    def get_distance(self):
        """Get the current distance reading from the laser, with error handling."""
        try:
            return self.laser.distance
        except LaserError:
            raise  # Let laser-specific errors propagate for display handling
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
        """Return gravity reading using gyro-accelerometer complementary filter."""
        raw = self.grav_sensor.acceleration
        if raw is None:
            return self._grav_filtered or (0.0, 0.0, 0.0)

        now = time.monotonic()

        # Initialize on first call
        if self._grav_filtered is None:
            self._grav_filtered = raw
            self._grav_last_time = now
            return raw

        dt = now - self._grav_last_time
        self._grav_last_time = now

        # Guard against large gaps or bad timing
        if dt <= 0 or dt > 0.5:
            self._grav_filtered = raw
            return raw

        # Read gyro angular rates (rad/s) from the same ISM330DHCX
        gyro = self.grav_sensor.gyro
        if gyro is None:
            self._grav_filtered = raw
            return raw

        # Subtract estimated gyro bias
        wx = gyro[0] - self._gyro_bias[0]
        wy = gyro[1] - self._gyro_bias[1]
        wz = gyro[2] - self._gyro_bias[2]
        gx, gy, gz = self._grav_filtered

        # Rotate previous gravity estimate by measured angular velocity
        # (small-angle cross product approximation: v_new ≈ v + (ω × v) · dt)
        pred_gx = gx + (wy * gz - wz * gy) * dt
        pred_gy = gy + (wz * gx - wx * gz) * dt
        pred_gz = gz + (wx * gy - wy * gx) * dt

        # Dynamic alpha: smooth when still, track when moving
        gyro_mag = math.sqrt(wx * wx + wy * wy + wz * wz)

        # When stationary, slowly learn the gyro bias
        if gyro_mag <= self._gyro_thresh_low:
            ba = self._gyro_bias_alpha
            self._gyro_bias[0] += ba * (gyro[0] - self._gyro_bias[0])
            self._gyro_bias[1] += ba * (gyro[1] - self._gyro_bias[1])
            self._gyro_bias[2] += ba * (gyro[2] - self._gyro_bias[2])

        if gyro_mag <= self._gyro_thresh_low:
            a = self._grav_alpha_low
        elif gyro_mag >= self._gyro_thresh_high:
            a = self._grav_alpha_high
        else:
            t = (gyro_mag - self._gyro_thresh_low) / (self._gyro_thresh_high - self._gyro_thresh_low)
            a = self._grav_alpha_low + t * (self._grav_alpha_high - self._grav_alpha_low)
        self._grav_filtered = (
            a * raw[0] + (1 - a) * pred_gx,
            a * raw[1] + (1 - a) * pred_gy,
            a * raw[2] + (1 - a) * pred_gz,
        )

        return self._grav_filtered

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

    def reset_laser(self):
        """Reset the laser sensor to clear any error state."""
        try:
            # Turn off laser
            self.laser.set_laser(False)
            time.sleep(0.1)
            # Flush UART buffer
            self.uart.reset_input_buffer()
            # Turn laser back on
            self.laser.set_laser(True)
            time.sleep(0.2)  # Give it time to stabilize
        except Exception as e:
            print(f"Error resetting laser: {e}")
