import asyncio
import board
import digitalio
import time
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
from calibration_manager import PerformCalibration
from mag_cal.calibration import Calibration

calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")  # create calibration object
sensor_manager = SensorManager()
button_manager = ButtonManager()

display = DisplayManager()

async def main():
    # Initialize the PerformCalibration with both sensor and button managers
    calibration = PerformCalibration(sensor_manager, button_manager, calib)

    # Start the calibration process
    await calibration.start_calibration()

    display.display_screen_initialise()


    # Start angle reading loop
    while True:
        try:
            azimuth, inclination, roll = calib.get_angles(
                sensor_manager.get_mag(), sensor_manager.get_grav()
            )
            print(f"({azimuth})")
            display.update_sensor_readings(0, azimuth, inclination)
        except Exception as e:
            print("Error getting angles:", e)
        time.sleep(0.25)

# Run the main async event loop
asyncio.run(main())
