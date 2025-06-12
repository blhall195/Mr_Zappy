import asyncio
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
#
class PerformCalibration:
    def __init__(self, sensor_manager: SensorManager, button_manager: ButtonManager):
        self.sensor_manager = sensor_manager  # Pass in the SensorManager instance
        self.button_manager = button_manager  # Pass in the ButtonManager instance
        self.mag_array = []
        self.grav_array = []

    async def start_calibration(self):
        print("Carry out the initial device calibration by pressing the fire button while rotating the device "
              "around as many different positions as possible 20   times.")

        iteration = 0
        while iteration < 2:
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
                print(f"Calibration iteration {iteration}: Mag: {mag_data}, Grav: {grav_data}")

                # Reset button state
                self.button_manager.was_pressed("Fire Button")  # Reset the state

            # Allow button updates
            await asyncio.sleep(0.1)

        print("First calibration step complete.")
        await asyncio.sleep(0.1)
#
        # Now, collect laser data (second step)
        print("\n")
        print("\n")
        print("Now align the laser, take one set of 8 re-adings while rotating the device with the  laser pointed at a   single spot, repeat  in the opposite      direction.")

        iteration = 0
        while iteration < 16:
            # Update button state
            self.button_manager.update()

            # Check if fire button is pressed (use ButtonManager for that)
            if self.button_manager.was_pressed("Fire Button"):
                iteration += 1
                # Collect laser readings (distance)
                laser_distance = self.sensor_manager.get_distance()
                print(f"Laser reading {iteration}: {laser_distance}")

                # Reset button state
                self.button_manager.was_pressed("Fire Button")  # Reset the state

            # Allow button updates
            await asyncio.sleep(0.1)

        print("Calibration process complete.")

# Example of running the calibration process
async def main():
    # Create instances of the SensorManager and ButtonManager
    sensor_manager = SensorManager()
    button_manager = ButtonManager()

    # Initialize the PerformCalibration with both sensor and button managers
    calibration = PerformCalibration(sensor_manager, button_manager)

    # Start the calibration process
    await calibration.start_calibration()


# Run the main async event loop
asyncio.run(main())
