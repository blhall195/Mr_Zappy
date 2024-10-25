import json
import storage
from mag_cal.calibration import Calibration

calib = Calibration() #create calibration object

# Load the calibration data from the file
with open("/calibration_data.json", "r") as file:
    calibration_dict = json.load(file)
    
print("Calibration data open.")

# Restore the calibration object from the saved dictionary
calib = calib.from_dict(calibration_dict)  # Ensure `from_dict` is implemented properly

#test the calibration works...

#example sensor readings
mag = ([30.5, -12.8, 45.3])  # x, y, z components of the magnetic field
grav = ([0.0, 9.81, 0.0])  # x, y, z components of gravity

#adjusted output
azimuth, inclination, roll = calib.get_angles(mag, grav)

print(azimuth,inclination)

