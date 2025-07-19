import asyncio
import board
import digitalio
import time
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
from calibration_manager import PerformCalibration
from mag_cal.calibration import Calibration
from ble_manager import BleManager

# Create instances
calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")
sensor_manager = SensorManager()
button_manager = ButtonManager()
display = DisplayManager()
ble = BleManager()

async def main():

    # ----Initialize the PerformCalibration with both sensor and button managers
    #calibration = PerformCalibration(sensor_manager, button_manager, calib)
    #await calibration.start_calibration()

    # Use a new variable name for the updated calib data
    calibration_dict = {'mag': {'axes': '-X-Y-Z', 'transform': [[0.0231683, -4.50966e-05, -0.000208465], [-4.50968e-05, 0.0233006, -2.46289e-05], [-0.000208464, -2.46296e-05, 0.0231333]], 'centre': [0.407859, -1.9058, 2.11295], 'rbfs': [], 'field_avg': None, 'field_std': None}, 'dip_avg': None, 'grav': {'axes': '-Y-X+Z', 'transform': [[0.101454, 0.00155312, -0.000734401], [0.00155312, 0.101232, 0.00149594], [-0.000734397, 0.00149594, 0.0987455]], 'centre': [0.364566, -0.0656354, 0.193454], 'rbfs': [], 'field_avg': None, 'field_std': None}}

    # Update calib from the dictionary
    calib_updated = calib.from_dict(calibration_dict)

    # Display initialization
    display.display_screen_initialise()
    display.update_battery(sensor_manager.get_bat())

    # Print calibration data to confirm
    calibration_dict = calib_updated.as_dict()
    print(calibration_dict)

    # Start angle reading loop
    while True:
        try:
            azimuth, inclination, roll = calib_updated.get_angles(
                sensor_manager.get_mag(), sensor_manager.get_grav()
            )
            print(f"({azimuth})")
            display.update_sensor_readings(0, azimuth, inclination)
        except Exception as e:
            print("Error getting angles:", e)

        button_manager.update()

        # If "Fire Button" is pressed, perform a special action
        if button_manager.was_pressed("Fire Button"):
            print("Fire Button pressed!")
            distance = sensor_manager.get_distance()
            print(distance)
            ble.send_message(azimuth,inclination,distance)

        await asyncio.sleep(0.1)

# Run the main async event loop
asyncio.run(main())
