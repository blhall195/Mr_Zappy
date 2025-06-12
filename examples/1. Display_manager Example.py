from display_manager import DisplayManager
import random
import time

# Initialize display manager
display = DisplayManager()

# Generate random values for each parameter
random_distance = random.uniform(0, 100)  # Random distance between 0 and 100 meters
random_azimuth = random.uniform(0, 360)  # Random azimuth angle between 0 and 360 degrees
random_inclination = random.uniform(-90, 90)  # Random inclination between -90 and 90 degrees
random_battery_percentage = random.uniform(0, 100)  # Random battery percentage between 0 and 100
random_bt_number = random.randint(0, 10)  # Random Bluetooth number between 0 and 10

# Update the display with the random values
display.update_distance(random_distance)
display.update_azimuth(random_azimuth)
display.update_inclination(random_inclination)
display.update_sensor_readings(random_distance, random_azimuth, random_inclination)
display.update_battery(random_battery_percentage)
display.update_BT_label(True)
display.update_BT_number(random_bt_number)

# Wait a bit so you can see it
time.sleep(0.5)

# Release display when done
display.release_displays()
display = DisplayManager()
print("test")



