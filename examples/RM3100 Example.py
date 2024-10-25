import board
import rm3100
import time

i2c = board.I2C()
rm = rm3100.RM3100_I2C(i2c, i2c_address=0x20)

while True:
    print(rm.magnetic)
    time.sleep(0.1)
    