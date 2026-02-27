import warnings
warnings.simplefilter('ignore')

import asyncio
import os
import json
import microcontroller
from config import Config
CONFIG = Config()


class CalibrationFlags:
    """Simple calibration flag loader — identical behavior to flat import logic."""
    def __init__(self):
        self.loaded_modules = []
        self._check_and_import_flags()

    def _check_and_import_flags(self):
        if "calibration_mode.txt" in os.listdir("/"):
            import calibration_mode
            self.loaded_modules.append("calibration_mode")

        if "calculate_ellipsoid.txt" in os.listdir("/"):
            import calibration_calculate_ellipsoid
            self.loaded_modules.append("calibration_calculate_ellipsoid")

        if "calculate_alignment.txt" in os.listdir("/"):
            import calibration_calculate_alignment
            self.loaded_modules.append("calibration_calculate_alignment")

        if "menu_mode.txt" in os.listdir("/"):
            import menu_manager
            self.loaded_modules.append("menu_manager")

class PerformCalibration:
    def __init__(self, sensor_manager, button_manager, calib, pwr_pin=None):
        self.calib = calib
        self.sensor_manager = sensor_manager
        self.button_manager = button_manager
        self.pwr_pin = pwr_pin
        self.mag_array = []
        self.grav_array = []

    async def collect_ellipsoid_data(self, device, disco_mode, display_manager=None):
        print("Starting ellipsoid   calibration...")
        self.mag_array.clear()
        self.grav_array.clear()
        sensor_mgr = self.sensor_manager
        button_mgr = self.button_manager

        sensor_mgr.set_laser(True)

        # Initialize coverage bar if display_manager provided
        coverage_bar = None
        if display_manager:
            from coverage_bar import CoverageBar
            coverage_bar = CoverageBar(display_manager.display, display_manager.font)
            coverage_bar.show()

        iteration = 0
        mag_buffer = []
        grav_buffer = []
        waiting_for_stable_sample = False

        while iteration < 56:
            if device.current_state != "CALIBRATING":
                print("Calibration cancelled.")
                return

            button_mgr.update()
            if button_mgr.was_pressed("Button 1") and not waiting_for_stable_sample:
                waiting_for_stable_sample = True
                disco_mode.set_red()
            if button_mgr.was_pressed("Button 4") and self.pwr_pin is not None:
                self.pwr_pin.value = False

            mag_data = sensor_mgr.get_mag()
            grav_data = sensor_mgr.get_grav()

            mag_buffer.append(mag_data)
            grav_buffer.append(grav_data)
            if len(mag_buffer) > 5: mag_buffer.pop(0)
            if len(grav_buffer) > 5: grav_buffer.pop(0)

            if waiting_for_stable_sample and len(mag_buffer) == 5 and len(grav_buffer) == 5:
                if self._is_consistent(mag_buffer, 0.4) and self._is_consistent(grav_buffer, 0.1):
                    disco_mode.set_green()

                    avg_mag = tuple(sum(axis_vals) / 5 for axis_vals in zip(*mag_buffer))
                    avg_grav = tuple(sum(axis_vals) / 5 for axis_vals in zip(*grav_buffer))

                    self.mag_array.append(avg_mag)
                    self.grav_array.append(avg_grav)

                    iteration += 1

                    # Update coverage bar with gravity (shows device orientation)
                    if coverage_bar:
                        coverage_bar.add_point(*avg_grav)
                    print(f"{iteration}/56")

                    sensor_mgr.set_buzzer(True)
                    await asyncio.sleep(0.2)
                    disco_mode.turn_off()
                    waiting_for_stable_sample = False

            await asyncio.sleep(0.002)

        print("Done")

        await self._save_data("/ellipsoid_data.json", "ellipsoid")
        self._write_flag("calculate_ellipsoid.txt")
        print("Rebooting device")
        await asyncio.sleep(1.5)
        microcontroller.reset()


    async def collect_alignment_data(self, disco_mode, display_manager=None):
        print("\nLaser alignment.     Rotate 8 times on    target. Repeat in 3  different directions.\n")
        self.mag_array.clear()
        self.grav_array.clear()

        sensor_mgr = self.sensor_manager
        button_mgr = self.button_manager

        # Initialize display for showing progress
        stage_label = None
        progress_label = None
        if display_manager:
            import displayio
            from adafruit_display_text import bitmap_label as label

            splash = displayio.Group()
            stage_label = label.Label(display_manager.font, text="Stage 1", scale=3, x=1, y=40)
            progress_label = label.Label(display_manager.font, text="0/8", scale=3, x=35, y=80)
            splash.append(stage_label)
            splash.append(progress_label)
            display_manager.display.root_group = splash

        iteration = 0
        mag_buffer = []
        grav_buffer = []
        waiting_for_stable_sample = False

        while iteration < 24:
            button_mgr.update()
            if button_mgr.was_pressed("Button 1") and not waiting_for_stable_sample:
                waiting_for_stable_sample = True
                disco_mode.set_red()
            if button_mgr.was_pressed("Button 4") and self.pwr_pin is not None:
                self.pwr_pin.value = False

            mag_data = sensor_mgr.get_mag()
            grav_data = sensor_mgr.get_grav()

            mag_buffer.append(mag_data)
            grav_buffer.append(grav_data)
            if len(mag_buffer) > 5: mag_buffer.pop(0)
            if len(grav_buffer) > 5: grav_buffer.pop(0)

            if waiting_for_stable_sample and len(mag_buffer) == 5 and len(grav_buffer) == 5:
                if self._is_consistent(mag_buffer, 0.4) and self._is_consistent(grav_buffer, 0.1):
                    disco_mode.set_green()

                    avg_mag = tuple(sum(axis_vals) / 5 for axis_vals in zip(*mag_buffer))
                    avg_grav = tuple(sum(axis_vals) / 5 for axis_vals in zip(*grav_buffer))

                    self.mag_array.append(avg_mag)
                    self.grav_array.append(avg_grav)
                    iteration += 1

                    # Update display with progress
                    if stage_label and progress_label:
                        stage = (iteration - 1) // 8 + 1
                        reading_in_stage = ((iteration - 1) % 8) + 1
                        stage_label.text = f"Stage {stage}"
                        progress_label.text = f"{reading_in_stage}/8"

                    print(f"Align Point: {iteration}/24")
                    sensor_mgr.set_buzzer(True)
                    await asyncio.sleep(0.2)
                    disco_mode.turn_off()
                    waiting_for_stable_sample = False

                    if iteration % 8 == 0:
                        print("Change direction...")
                        for _ in range(3):
                            sensor_mgr.set_buzzer(True)
                            await asyncio.sleep(0.1)

            await asyncio.sleep(0.002)

        for _ in range(3):
            sensor_mgr.set_buzzer(True)
            await asyncio.sleep(0.1)

        await self._save_data("/alignment_data.json", "raw sensor")
        self._write_flag("calculate_alignment.txt")
        print("Rebooting device")
        await asyncio.sleep(1.5)
        microcontroller.reset()

    async def wait_for_calibration_choice(self, device, disco_mode, display_manager=None):
        print("\nPress Button 1 for   Ellipsoid Calibration")
        print("")
        print("Press Button 2 for   Alignment Calibration\n")

        while True:
            self.button_manager.update()

            if self.button_manager.was_pressed("Button 1"):
                await self.collect_ellipsoid_data(device, disco_mode, display_manager)
                break  # Should not be reached due to reset
            elif self.button_manager.was_pressed("Button 2"):
                await self.collect_alignment_data(disco_mode, display_manager)
                break  # Should not be reached due to reset
            elif self.button_manager.was_pressed("Button 4") and self.pwr_pin is not None:
                self.pwr_pin.value = False

            await asyncio.sleep(0.01)

    def _is_consistent(self, buffer, threshold):
        base = buffer[0]
        for other in buffer[1:]:
            diffs = [abs(a - b) for a, b in zip(base, other)]
            if any(diff > threshold for diff in diffs):
                return False
        return True

    async def _save_data(self, filename, label):
        raw_data = {
            "mag": [list(m) for m in self.mag_array],
            "grav": [list(g) for g in self.grav_array]
        }
        try:
            with open(filename, "w") as f:
                json.dump(raw_data, f)
            print("Processing data...")
        except Exception as e:
            print(f"❌ Failed to save {label} data: {e}")

    def _write_flag(self, filename):
        if filename not in os.listdir("/"):
            try:
                with open(f"/{filename}", "w") as f:
                    f.write("1")
            except OSError as e:
                print(f"Failed to write {filename}: {e}")
