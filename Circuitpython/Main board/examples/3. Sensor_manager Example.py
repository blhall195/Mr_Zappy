from sensor_manager import SensorManager
import time
# Instantiate and read values
sensor_manager = SensorManager()
time.sleep(1)

print("Distance:", sensor_manager.get_distance())
print("Gravity:", sensor_manager.get_grav())
print("Mag:", sensor_manager.get_mag())
print("Bat:", sensor_manager.get_bat())

while True:
    x, y, z = sensor_manager.get_grav()  # Or use get_mag(), etc.
    print((x, y, z,))
    time.sleep(0.1)

