import json

from mag_cal.calibration import Calibration

with open("/calibration_dict.json", "r") as f:
    data = json.load(f)

calib = Calibration.from_dict(data)

with open("/raw_alignment_mag_grav_data.json", "r") as f:
    raw_data = json.load(f)

mag_array = raw_data["mag"]
grav_array = raw_data["grav"]


angles = [calib.get_angles(m, g) for m, g in zip(mag_array, grav_array)]

print(angles)
