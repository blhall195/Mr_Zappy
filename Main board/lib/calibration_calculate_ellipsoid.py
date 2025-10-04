import os

# Delete calibration flag if it exists
if "calculate_ellipsoid.txt" in os.listdir("/"):
    try:
        os.remove("/calculate_ellipsoid.txt")
    except OSError as e:
        print("Error deleting file:", e)

import json
from mag_cal.calibration import Calibration
import asyncio
import microcontroller
import time

calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")

# Load sensor data
with open("/ellipsoid_data.json", "r") as f:
    raw_data = json.load(f)

mag_array = raw_data["mag"]
grav_array = raw_data["grav"]

mag_accuracy, grav_accuracy = calib.fit_ellipsoid(mag_array, grav_array)
calib.set_field_characteristics(mag_array, grav_array)
calib.set_expected_mean_dip(mag_array, grav_array)
calibration_dict = calib.as_dict() #load calibration into dictionary

time.sleep(0.5)
from button_manager import ButtonManager
button_manager = ButtonManager()

print("")
print("")
print(f"  Mag: {mag_accuracy}")
print(f"  Grav: {grav_accuracy}")
print("")
print("✅ Hold 1+2 to SAVE")
print("❌ Hold 2 to DISCARD")
print("Lower = Better")
print("")

async def main():

    selected = None
    hold_counter = 0.0
    save_hold_time = 0.06  # seconds

    while selected is None:
        button_manager.update()
        b1 = button_manager.is_pressed("Button 1")
        b2 = button_manager.is_pressed("Button 2")

        if b1 and b2:
            hold_counter += 0.01
            if hold_counter >= save_hold_time:
                selected = True
                print("Saving calibration")
                await asyncio.sleep(0.5)
                # Delete calibration flag if it exists
        elif not b1 and b2:
            hold_counter += 0.01
            if hold_counter >= save_hold_time:
                selected = False
                print("Discarding calibration...")
                await asyncio.sleep(0.5)
        else:
            hold_counter = 0.0

        await asyncio.sleep(0.01)

    if selected:
        try:
            with open("/calibration_dict.json", "w") as f:
                json.dump(calib.as_dict(), f)
            print("Calibration saved.")
        except Exception as e:
            print(f"Failed to save calibration: {e}")
    else:
        print("Calibration discarded.")

    if "ellipsoid_data.json" in os.listdir("/"):
        try:
            os.remove("/ellipsoid_data.json")
        except OSError as e:
            print("Error deleting file:", e)

    print("Rebooting...")
    print("Please wait")
    await asyncio.sleep(2)
    microcontroller.reset()

# Run it
asyncio.run(main())
