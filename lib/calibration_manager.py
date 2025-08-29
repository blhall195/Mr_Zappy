import asyncio
import json
import gc
from disco_manager import DiscoMode
import microcontroller
import os

class PerformCalibration:
    def __init__(self, sensor_manager, button_manager, calib):
        self.calib = calib
        self.sensor_manager = sensor_manager
        self.button_manager = button_manager
        self.mag_array = []
        self.grav_array = []

    async def start_calibration(self, device, disco_mode):
        print("🔧 Starting calibration. Press fire button while rotating the device in all directions.")
        sensor_mgr = self.sensor_manager
        button_mgr = self.button_manager
        calib = self.calib

        sensor_mgr.set_laser(True)
        self.mag_array.clear()
        self.grav_array.clear()

        # --- First phase ---
        iteration = 0
        mag_buffer = []
        grav_buffer = []
        waiting_for_stable_sample = False  # Tracks if we're in measurement mode

        while iteration < 2:
            if device.current_state != "CALIBRATING":
                print("❌ Calibration cancelled during phase 1.")
                return

            button_mgr.update()

            # Trigger new sampling cycle
            if button_mgr.was_pressed("Button 1") and not waiting_for_stable_sample:
                waiting_for_stable_sample = True
                disco_mode.set_red()  # Indicate measurement in progress

            # Always collect latest readings
            mag_data = sensor_mgr.get_mag()
            grav_data = sensor_mgr.get_grav()

            # Maintain buffer of last 3 readings
            mag_buffer.append(mag_data)
            grav_buffer.append(grav_data)

            if len(mag_buffer) > 3:
                mag_buffer.pop(0)
            if len(grav_buffer) > 3:
                grav_buffer.pop(0)

            # If waiting for stable reading, evaluate buffers
            if waiting_for_stable_sample and len(mag_buffer) == 3 and len(grav_buffer) == 3:
                def is_consistent(buffer, threshold=0.15):
                    base = buffer[0]
                    for other in buffer[1:]:
                        diffs = [abs(a - b) for a, b in zip(base, other)]
                        if any(diff > threshold for diff in diffs):
                            return False
                    return True

                mag_ok = is_consistent(mag_buffer)
                grav_ok = is_consistent(grav_buffer)

                if mag_ok and grav_ok:
                    disco_mode.set_green()
                    self.mag_array.append(mag_data)
                    self.grav_array.append(grav_data)
                    iteration += 1
                    print(f"Calib Point: {iteration}/16")
                    sensor_mgr.set_buzzer(True)
                    await asyncio.sleep(0.2)  # Allow time for user to register feedback
                    disco_mode.turn_off()  # ✅ Turn off LED after success
                    waiting_for_stable_sample = False  # ✅ Reset for next button press

            await asyncio.sleep(0.02)

        gc.collect() #free up a bit of RAM
        mag_accuracy, grav_accuracy = calib.fit_ellipsoid(self.mag_array, self.grav_array)
        print(f"✅ Mag: {mag_accuracy}")
        print(f"✅ Grav: {grav_accuracy}")
        print("✅ Hold 1+2 to SAVE")
        print("❌ Hold 2 to DISCARD")
        await asyncio.sleep(0.02)

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

        calibration_dict = calib.as_dict() #load existing calibration to be overwritten

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

        print("🎉 Calibration data saved.")
        await asyncio.sleep(0.5)

        await asyncio.sleep(0.1)
        sensor_mgr.set_buzzer(True)
        await asyncio.sleep(0.1)
        sensor_mgr.set_buzzer(True)
        await asyncio.sleep(0.1)
        sensor_mgr.set_buzzer(True)
        print("")
        print("Laser alignment, rotate 8 times on target, repeat @ 90 degrees")
        print("")
        print("")
        print("")
        print("")

        self.mag_array.clear()
        self.grav_array.clear()

        # --- Second phase ---
        iteration = 0
        mag_buffer = []
        grav_buffer = []
        waiting_for_stable_sample = False  # Tracks if we're in measurement mode

        while iteration < 16:

            button_mgr.update()

            # Trigger new sampling cycle
            if button_mgr.was_pressed("Button 1") and not waiting_for_stable_sample:
                waiting_for_stable_sample = True
                disco_mode.set_red()  # Indicate measurement in progress

            # Always collect latest readings
            mag_data = sensor_mgr.get_mag()
            grav_data = sensor_mgr.get_grav()

            # Maintain buffer of last 3 readings
            mag_buffer.append(mag_data)
            grav_buffer.append(grav_data)

            if len(mag_buffer) > 3:
                mag_buffer.pop(0)
            if len(grav_buffer) > 3:
                grav_buffer.pop(0)

            # If waiting for a stable reading, check buffer
            if waiting_for_stable_sample and len(mag_buffer) == 3 and len(grav_buffer) == 3:
                def is_consistent(buffer, threshold=0.3):
                    base = buffer[0]
                    for other in buffer[1:]:
                        diffs = [abs(a - b) for a, b in zip(base, other)]
                        if any(diff > threshold for diff in diffs):
                            return False
                    return True

                mag_ok = is_consistent(mag_buffer)
                grav_ok = is_consistent(grav_buffer)

                if mag_ok and grav_ok:
                    disco_mode.set_green()
                    self.mag_array.append(mag_data)
                    self.grav_array.append(grav_data)
                    iteration += 1
                    print(f"Align Point {iteration}/16")
                    sensor_mgr.set_buzzer(True)
                    await asyncio.sleep(0.2)
                    disco_mode.turn_off()
                    waiting_for_stable_sample = False  # Reset for next button press

            await asyncio.sleep(0.02)


        # --------------------------------------------------
        # Save data for reboot
        # Build and save raw data to its own dict and file
        raw_data = {
            "mag": [list(m) for m in self.mag_array],
            "grav": [list(g) for g in self.grav_array]
        }
        try:
            with open("/alignment_mag_grav_data.json", "w") as f:
                json.dump(raw_data, f)
            print("📦 Raw sensor data saved.")
        except Exception as e:
            print(f"❌ Failed to save raw data: {e}")
        # --------------------------------------------------

        if "calibration_mode_activate.txt" not in os.listdir("/"):
            try:
                with open("/calibration_mode_activate.txt", "w") as f:
                    f.write("1")
                print("✅ calibration_mode_activate.txt written")
                await asyncio.sleep(2)  # ⬅️ Give time to flush before reset
            except OSError as e:
                print(f"❌ Failed to write calibration_mode_activate.txt: {e}")

        print("resetting device")
        microcontroller.reset()

