import asyncio

class PerformCalibration:
    def __init__(self, sensor_manager, button_manager, calib):
        self.calib = calib
        self.sensor_manager = sensor_manager
        self.button_manager = button_manager
        self.mag_array = []
        self.grav_array = []

    async def start_calibration(self, device):
        print("🔧 Starting calibration. Press fire button while rotating the device in all directions.")
        self.sensor_manager.set_laser(False)
        self.mag_array = []
        self.grav_array = []

        # --- First phase ---
        iteration = 0
        while iteration < 16:
            if device.current_state != "CALIBRATING":
                print("❌ Calibration cancelled during phase 1.")
                return

            self.button_manager.update()

            if self.button_manager.was_pressed("Fire Button"):
                iteration += 1
                mag_data = self.sensor_manager.get_mag()
                grav_data = self.sensor_manager.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"📍 Calib Point {iteration}/16")
                self.sensor_manager.set_buzzer(True)

            await asyncio.sleep(0.01)

        mag_accuracy, grav_accuracy = self.calib.fit_ellipsoid(self.mag_array, self.grav_array)
        print(f"✅ Mag accuracy: {mag_accuracy}")
        print(f"✅ Grav accuracy: {grav_accuracy}")
        print("✔ First calibration step complete.\n\n")
        await asyncio.sleep(0.1)

        # --- Second phase ---
        if device.current_state != "CALIBRATING":
            print("❌ Calibration cancelled before second phase.")
            return

        print("📐 Now align the laser. Rotate the device 8 times with laser on target, then repeat in the opposite direction.")
        self.sensor_manager.set_laser(True)
        self.mag_array = []
        self.grav_array = []

        iteration = 0
        while iteration < 16:
            if device.current_state != "CALIBRATING":
                print("❌ Calibration cancelled during phase 2.")
                return

            self.button_manager.update()

            if self.button_manager.was_pressed("Fire Button"):
                iteration += 1
                mag_data = self.sensor_manager.get_mag()
                grav_data = self.sensor_manager.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"📍 Align Point {iteration}/16")
                self.sensor_manager.set_buzzer(True)

            await asyncio.sleep(0.01)

        # --- Final fitting ---
        runs = self.calib.find_similar_shots(self.mag_array, self.grav_array)
        paired_data = [(self.mag_array[a:b], self.grav_array[a:b]) for a, b in runs]
        print(f"🔗 Paired runs: {runs}")
        print(f"🔍 Sample data: {paired_data}")
        self.calib.fit_to_axis(paired_data)

        print("🎉 Calibration process complete.")
