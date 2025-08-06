import asyncio
import json
#import gc

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

            if self.button_manager.was_pressed("Button 1"):
                iteration += 1
                mag_data = self.sensor_manager.get_mag()
                grav_data = self.sensor_manager.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"📍 Calib Point {iteration}/16")
                self.sensor_manager.set_buzzer(True)

            await asyncio.sleep(0.01)

        mag_accuracy, grav_accuracy = self.calib.fit_ellipsoid(self.mag_array, self.grav_array)
        print(f"✅ Mag: {mag_accuracy}")
        print(f"✅ Grav: {grav_accuracy}")
        print("")
        await asyncio.sleep(0.1)


        # --- Second phase ---
        if device.current_state != "CALIBRATING":
            print("❌ Calibration cancelled before second phase.")
            return

        uniformity = self.calib.uniformity(self.mag_array, self.grav_array)

        print(f"Uniformity: {uniformity} lower = better")
        print(f"")
        print("Laser alignment, rotate 8 times on target, repeat @ 90 degrees")
        self.sensor_manager.set_laser(True)
        await asyncio.sleep(0.1)
        self.sensor_manager.set_buzzer(True)
        await asyncio.sleep(0.1)
        self.sensor_manager.set_buzzer(True)

        iteration = 0
        while iteration < 16:
            if device.current_state != "CALIBRATING":
                print("❌ Calibration cancelled during phase 2.")
                return

            self.button_manager.update()

            if self.button_manager.was_pressed("Button 1"):
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
        #print(f"🔗 Paired runs: {runs}")
        #print(f"🔍 Sample data: {paired_data}")

        self.calib.fit_to_axis(paired_data)
        #gc.collect()
        self.calib.fit_non_linear_quick(paired_data, param_count=3)

        calibration_dict = self.calib.as_dict()

        accuracy = self.calib.accuracy(paired_data)

        print(f"Accuracy: {accuracy}, should be lower than 0.5")
        print("✅ Hold 1+2 to SAVE")
        print("❌ Hold 2 to DISCARD")

        selected = None
        save_hold_time = 0.06  # seconds
        hold_counter = 0.0

        while selected is None:
            self.button_manager.update()

            b1 = self.button_manager.is_pressed("Button 1")
            b2 = self.button_manager.is_pressed("Button 2")

            if b1 and b2:
                hold_counter += 0.01
                if hold_counter >= save_hold_time:
                    selected = True
                    print("")
                    print("")
                    print("Saving calibration...")
                    await asyncio.sleep(0.5)
            elif not b1 and b2:
                hold_counter += 0.01
                if hold_counter >= save_hold_time:
                    selected = False
                    print("")
                    print("")
                    print("Calibration not saved.")
                    await asyncio.sleep(0.5)

            else:
                hold_counter = 0.0  # reset if not holding the required combo

            await asyncio.sleep(0.01)

        if selected:
            try:
                with open("/calibration_dict.json", "w") as f:
                    json.dump(calibration_dict, f)
                print("✅ Calibration saved.")
            except Exception as e:
                print(f"❌ Failed to save calibration: {e}")
            await asyncio.sleep(0.5)
        else:
            print("⚠ Calibration data discarded.")

            await asyncio.sleep(0.5)

        print("🎉 Calibration process complete.")
        await asyncio.sleep(0.5)
