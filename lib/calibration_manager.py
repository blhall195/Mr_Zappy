import asyncio
import json
import gc

class PerformCalibration:
    def __init__(self, sensor_manager, button_manager, calib):
        self.calib = calib
        self.sensor_manager = sensor_manager
        self.button_manager = button_manager
        self.mag_array = []
        self.grav_array = []

    async def start_calibration(self, device):
        print("🔧 Starting calibration. Press fire button while rotating the device in all directions.")
        sensor_mgr = self.sensor_manager
        button_mgr = self.button_manager
        calib = self.calib

        sensor_mgr.set_laser(False)
        self.mag_array.clear()
        self.grav_array.clear()

        # --- First phase ---
        iteration = 0
        while iteration < 16:
            if device.current_state != "CALIBRATING":
                print("❌ Calibration cancelled during phase 1.")
                return

            button_mgr.update()

            if button_mgr.was_pressed("Button 1"):
                iteration += 1
                mag_data = sensor_mgr.get_mag()
                grav_data = sensor_mgr.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"📍 Calib Point {iteration}/24")
                sensor_mgr.set_buzzer(True)

            await asyncio.sleep(0.02)  # slight increase for less CPU load

        mag_accuracy, grav_accuracy = calib.fit_ellipsoid(self.mag_array, self.grav_array)
        print(f"✅ Mag: {mag_accuracy}")
        print(f"✅ Grav: {grav_accuracy}")
        print("")
        await asyncio.sleep(0.1)

        # Free memory from first phase data if no longer needed there
        # If the data is still needed later, skip these lines
        # self.mag_array.clear()
        # self.grav_array.clear()
        # gc.collect()

        if device.current_state != "CALIBRATING":
            print("❌ Calibration cancelled before second phase.")
            return

        uniformity = calib.uniformity(self.mag_array, self.grav_array)
        print(f"Uniformity: {uniformity} lower = better")
        print("")
        print("Laser alignment, rotate 8 times on target, repeat @ 90 degrees")
        sensor_mgr.set_laser(True)
        await asyncio.sleep(0.1)
        sensor_mgr.set_buzzer(True)
        await asyncio.sleep(0.1)
        sensor_mgr.set_buzzer(True)

        self.mag_array.clear()
        self.grav_array.clear()

        iteration = 0
        while iteration < 16:
            if device.current_state != "CALIBRATING":
                print("❌ Calibration cancelled during phase 2.")
                return

            button_mgr.update()

            if button_mgr.was_pressed("Button 1"):
                iteration += 1
                mag_data = sensor_mgr.get_mag()
                grav_data = sensor_mgr.get_grav()

                self.mag_array.append(mag_data)
                self.grav_array.append(grav_data)
                print(f"📍 Align Point {iteration}/16")
                sensor_mgr.set_buzzer(True)

            await asyncio.sleep(0.02)  # slightly more delay

        # --- Final fitting ---
        runs = calib.find_similar_shots(self.mag_array, self.grav_array)
        paired_data = [(self.mag_array[a:b], self.grav_array[a:b]) for a, b in runs]

        calib.fit_to_axis(paired_data)
        gc.collect()

        calib.fit_non_linear_quick(paired_data, param_count=5)
        calib.align_sensor_roll(self.mag_array, self.grav_array)

        calibration_dict = calib.as_dict()
        accuracy = calib.accuracy(paired_data)

        print(f"Accuracy: {accuracy}, should be lower than 0.5")
        print("✅ Hold 1+2 to SAVE")
        print("❌ Hold 2 to DISCARD")

        selected = None
        save_hold_time = 0.06  # seconds
        hold_counter = 0.0

        while selected is None:
            button_mgr.update()

            b1 = button_mgr.is_pressed("Button 1")
            b2 = button_mgr.is_pressed("Button 2")

            if b1 and b2:
                hold_counter += 0.01
                if hold_counter >= save_hold_time:
                    selected = True
                    print("\n\nSaving calibration...")
                    await asyncio.sleep(0.5)
            elif not b1 and b2:
                hold_counter += 0.01
                if hold_counter >= save_hold_time:
                    selected = False
                    print("\n\nCalibration not saved.")
                    await asyncio.sleep(0.5)
            else:
                hold_counter = 0.0  # reset if not holding combo

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
