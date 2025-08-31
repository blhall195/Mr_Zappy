import os
import json
from display_manager import DisplayManager
display = DisplayManager()
from button_manager import ButtonManager
button_manager = ButtonManager()

# Delete calibration flag if it exists
if "calibration_mode_activate.txt" in os.listdir("/"):
    try:
        os.remove("/calibration_mode_activate.txt")
        print("Device Rebooted")
        print("Processing...")
    except OSError as e:
        print("Error deleting file:", e)

from mag_cal.calibration import Calibration
from ulab import numpy as np
import asyncio


import microcontroller

# ---- Define your custom align_sensor_roll and dependencies ----
def lstsq(A, B):
    try:
        from ulab import scipy
    except ImportError:
        import scipy

    A_sq = np.dot(A.transpose(), A)
    R = np.linalg.cholesky(A_sq)
    try:
        x = scipy.linalg.cho_solve(R, np.dot(A.transpose(), B))
    except TypeError:
        # SciPy expects (R, True) tuple if not using ulab
        x = scipy.linalg.cho_solve((R, True), np.dot(A.transpose(), B))
    return x, None

def rot3Dsc(s, c, ax):
    rot = np.zeros((3, 3))
    rot[ax, ax] = 1
    axp = ax - 1 if ax > 0 else 2
    axn = ax + 1 if ax < 2 else 0
    rot[axp, axp] = c
    rot[axn, axn] = c
    rot[axp, axn] = -s
    rot[axn, axp] = s
    return rot

def calib_fit_rotM_cstdip(mcorr, gcorr, axis=0, verbose=True):
    oa = tuple({0, 1, 2} - {axis})  # other axes
    sign = (1, -1, -1)
    rot = np.eye(3)
    prevrotsin = None
    for it in range(3):
        m = np.dot(rot, mcorr.transpose()).transpose()
        xb = np.sum(m * gcorr, axis=-1)
        xa = m[:, oa[0]] * gcorr[:, oa[1]] - m[:, oa[1]] * gcorr[:, oa[0]]
        D = np.concatenate(
            (
                xa.reshape((mcorr.shape[0], 1)),
                np.ones((mcorr.shape[0], 1)),
            ),
            axis=-1,
        )
        coeffs, _ = lstsq(D, xb)
        s = coeffs[0]
        if prevrotsin is not None and abs(s) > prevrotsin:
            raise ValueError("rotation must decrease at each iteration.")
        prevrotsin = abs(s)
        c = np.sqrt(1 - s * s)
        rot = np.dot(rot, rot3Dsc(sign[axis] * s, c, axis))
    return rot

# This is the method to monkey patch
def align_sensor_roll(self, mag_data, grav_data):
    mag_data = self.mag.apply(mag_data)
    grav_data = self.grav.apply(grav_data)
    rot = calib_fit_rotM_cstdip(mag_data, grav_data, 1, False)
    self.mag.transform = np.dot(self.mag.transform, rot.transpose())

# ---- Monkey patch it into the Calibration class ----
Calibration.align_sensor_roll = align_sensor_roll
# ---- Main async logic ----
async def main():

    # Display and button setup

    await asyncio.sleep(2)
    calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")


    # Load existing calibration
    try:
        with open("/calibration_dict.json", "r") as f:
            calibration_dict = json.load(f)
        calib = Calibration.from_dict(calibration_dict)
    except Exception as e:
        print(f"⚠ Failed to load existing calibration: {e}")
        microcontroller.reset()

    # Load sensor data
    with open("/alignment_data.json", "r") as f:
        raw_data = json.load(f)

    mag_array = raw_data["mag"]
    grav_array = raw_data["grav"]

    runs = calib.find_similar_shots(mag_array, grav_array)
    paired_data = [(mag_array[a:b], grav_array[a:b]) for a, b in runs]

    calib.fit_to_axis(paired_data)
    calib.fit_non_linear_quick(paired_data, param_count=5)
    calib.align_sensor_roll(mag_array, grav_array)
    accuracy = calib.accuracy(paired_data)



    print(f"Accuracy: {accuracy:.3f}      Should be 0.5 or     lower)")
    print("")
    print("")
    print("✅ Hold 1+2 to SAVE")
    print("❌ Hold 2 to DISCARD")

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


    print("Rebooting again")
    print("Please wait")
    await asyncio.sleep(2)
    microcontroller.reset()

# Run it
asyncio.run(main())
