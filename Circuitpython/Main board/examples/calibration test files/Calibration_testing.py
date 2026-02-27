import json
from mag_cal.calibration import Calibration
from ulab import numpy as np

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

calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")

with open("/raw_ellipsoid_data.json", "r") as f:
    raw_data = json.load(f)

mag_array = raw_data["mag"]
grav_array = raw_data["grav"]

#print(mag_array)

mag_accuracy, grav_accuracy = calib.fit_ellipsoid(mag_array, grav_array)
print(f"✅ Mag: {mag_accuracy}")
print(f"✅ Grav: {grav_accuracy}")



with open("/raw_alignment_mag_grav_data.json", "r") as f:
    raw_data = json.load(f)

mag_array = raw_data["mag"]
grav_array = raw_data["grav"]


angles = [calib.get_angles(m, g) for m, g in zip(mag_array, grav_array)]

print(angles)

runs = calib.find_similar_shots(mag_array, grav_array)
paired_data = [(mag_array[a:b], grav_array[a:b]) for a, b in runs]
calib.fit_to_axis(paired_data)
calib.fit_non_linear_quick(paired_data, param_count=5)
calib.align_sensor_roll(mag_array, grav_array)



angles = [calib.get_angles(m, g) for m, g in zip(mag_array, grav_array)]
print("")

print(angles)

with open("/calibration_dict.json", "w") as f:
    json.dump(calib.as_dict(), f)
