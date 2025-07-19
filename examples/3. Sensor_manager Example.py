from sensor_manager import SensorManager

# Instantiate and read values
sensor_manager = SensorManager()
print("Distance:", sensor_manager.get_distance())
print("Gravity:", sensor_manager.get_grav())
print("Mag:", sensor_manager.get_mag())

while True:
    x, y, z = sensor_manager.get_grav()  # Or use get_mag(), etc.
    print((x, y, z,))
    time.sleep(0.1)

