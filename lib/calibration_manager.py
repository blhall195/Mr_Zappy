import asyncio
import board
import digitalio
import time
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
from mag_cal.calibration import Calibration

class PerformCalibration:
    def __init__(self, sensor_manager: SensorManager, button_manager: ButtonManager, calib):
        self.calib = calib
        self.sensor_manager = sensor_manager  # Pass in the SensorManager instance
        self.button_manager = button_manager  # Pass in the ButtonManager instance
        self.mag_array = []
        self.grav_array = []

    async def start_calibration(self):
        print("Carry out the initial device calibration by pressing the fire button while rotating the device "
              "around as many different positions as possible 20   times.")
        self.sensor_manager.set_laser(False)

        iteration = 0
        while iteration < 16:
            # Update button state
            # Update button state
            self.button_manager.update()

            # Check if fire button is pressed (use ButtonManager for that)
            if self.button_manager.was_pressed("Fire Button"):
                iteration += 1
                # Collect readings from sensors
                mag_data = self.sensor_manager.get_mag()
                grav_data = self.sensor_manager.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"Calib Num. {iteration}/20")

                # Reset button state
                self.button_manager.was_pressed("Fire Button")  # Reset the state
                self.sensor_manager.set_buzzer(True)

            # Allow button updates
            await asyncio.sleep(0.1)

        mag_accuracy, grav_accuracy =  self.calib.fit_ellipsoid(self.mag_array, self.grav_array)
        print(mag_accuracy)
        print(grav_accuracy)
        print("First calibration step complete.")
        await asyncio.sleep(0.1)
#
        # Now, collect laser data (second step)
        print("\n")
        print("\n")
        print("Now align the laser, take one set of 8 re-adings while rotating the device with the  laser pointed at a   single spot, repeat  in the opposite      direction.")

        self.mag_array = []
        self.grav_array = []
        self.sensor_manager.set_laser(True)

        iteration = 0
        while iteration < 16:
            # Update button state
            self.button_manager.update()

            # Check if fire button is pressed (use ButtonManager for that)
            if self.button_manager.was_pressed("Fire Button"):
                iteration += 1
                # Collect laser readings (distance)
                self.sensor_manager.set_buzzer(True)
                mag_data = self.sensor_manager.get_mag()
                grav_data = self.sensor_manager.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"Align Num. {iteration}/20")

                # Reset button state
                self.button_manager.was_pressed("Fire Button")  # Reset the state

            # Allow button updates
            await asyncio.sleep(0.1)

        runs = self.calib.find_similar_shots(self.mag_array, self.grav_array)
        paired_data = [(self.mag_array[a:b], self.grav_array[a:b]) for a, b in runs]
        print(runs)
        print(paired_data)
        self.calib.fit_to_axis(paired_data)


        print("Calibration process complete.")

