import board
import rm3100
import time

i2c = board.I2C()
mag_sensor = rm3100.RM3100_I2C(i2c, i2c_address=0x20)

# Initialize an empty list to store the results
mag_array = []

# Define the number of times you want to run the rm.magnetic function
num_iterations = 24

# Loop to run the rm.magnetic function and collect its results
for _ in range(num_iterations):
    mag_reading = mag_sensor.magnetic
    print(mag_reading)
    mag_array.append(mag_reading)  # Directly call rm.magnetic() and append the result
    time.sleep(1)

# Now, mag_array should contain 24 tuples with magnetic data
print(mag_array)

#while True:
    #print(rm.magnetic)
    #time.sleep(0.1)
    
    