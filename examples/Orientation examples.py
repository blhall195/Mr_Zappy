import time
import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX
from adafruit_lis3mdl import LIS3MDL

i2c = board.I2C()  # uses board.SCL and board.SDA
#i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = ISM330DHCX(i2c)
mag = LIS3MDL(i2c)

while True:
    magnetic = mag.magnetic
    acceleration = sensor.acceleration
    gyro = sensor.gyro

    #print(f"Magnetometer X:{magnetic[0]:.2f} Y:{magnetic[1]:.2f} Z:{magnetic[2]:.2f}")
    print(f"Accelerometer X:{acceleration[0]:.2f} Y:{acceleration[1]:.2f} Z:{acceleration[2]:.2f}")
    #print(f"Gyroscope X:{gyro[0]:.2f} Y:{gyro[1]:.2f} Z:{gyro[2]:.2f}")
    
    
    # Optional: sleep to avoid overloading the serial output
    time.sleep(0.1)
