import time
import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX

i2c = board.I2C()  # uses board.SCL and board.SDA
grav_sensor = ISM330DHCX(i2c)

# Initialize an empty list to store the results
grav_array = []

# Define the number of times you want to run the rm.magnetic function
num_iterations = 24

# Loop to run the rm.magnetic function and collect its results
for _ in range(num_iterations):
    grav_reading = grav_sensor.acceleration
    print(grav_reading)
    grav_array.append(grav_reading)  # Directly call grav_sensor.acceleration() and append the result
    time.sleep(1)

# Now, grav_array should contain 24 tuples with accelerometer data
print(grav_array)

#while True:
    #accel = grav_sensor.acceleration
    #print(accel)
    #time.sleep(0.1)
