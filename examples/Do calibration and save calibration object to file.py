import json
import storage
import array
import math
import gc #this might come in handy later on if you run into memory issues.

from mag_cal.calibration import Calibration

calib = Calibration(mag_axes= "+Y-X+Z",grav_axes="-X+Y+Z") #create calibration object

#----------------------------------------------------------------------------------------
#Run calibration

# Define the arrays (your data)
mag_array =[(67.9, 13.5625, -62.275), (77.7625, 3.5375, 42.35), (-4.8125, 8.2125, 89.025), (-89.3, 16.7125, 5.1625), (-74.675, 51.2875, 6.8125), (-46.25, -52.8375, 45.05), (51.225, -58.45, 27.8875), (19.4, -62.6875, -54.45), (-62.325, -8.9625, -56.4875), (-72.275, 14.4625, 40.1625), (-48.8625, 14.175, 67.075), (66.575, 32.025, 44.3125), (42.5875, 75.55, 2.0625), (21.9375, 82.2875, 5.4875), (-15.375, 80.725, 13.225), (-23.075, 79.4125, -9.2125), (-36.3375, 66.05, -40.75), (25.6375, 65.85, -50.375), (45.85, 65.075, 27.1), (-46.3375, 65.975, 18.7875), (-1.5125, 62.9875, 53.85), (13.7, 62.0625, -58.1875), (-54.9625, 60.675, -7.1125), (-25.7875, 61.625, 45.1375)]
grav_array = [(0.142373, -0.295513, 10.2975), (0.647258, 9.6323, -0.423529), (-0.802791, 0.713061, -9.85125), (0.259621, -9.72921, -3.06401), (6.36012, -6.71545, 0.218943), (10.8144, 0.721436, -2.19661), (9.37029, 1.35075, -2.65364), (10.0594, 1.36391, -0.405583), (-0.0813559, -1.03609, -8.99342), (-2.68953, 7.05045, -3.99242), (0.777667, 4.46261, 3.06879), (1.11625, -2.10329, 9.96969), (1.17846, -7.39023, 6.42233), (-1.54576, 8.26241, 4.88136), (-1.21555, 3.00299, -9.60957), (-0.866201, -6.12682, -8.53639), (0.946361, -10.1216, 4.35972), (-0.184247, 0.43669, 9.32482), (8.22771, 1.05284, -2.1679), (9.17408, -2.0985, 1.36869), (6.15075, -5.14337, 4.55952), (6.28235, 4.17308, 6.44746), (6.98584, 7.60319, 0.0155533), (6.14955, 1.73958, -7.44646)]
mag_accuracy, grav_accuracy =  calib.fit_ellipsoid(mag_array, grav_array)
#calib.fit_to_axis(aligned)
#calib.fit_non_linear_quick(aligned, param_count=5)

#----------------------------------------------------------------------------------------
#Test calibration

#create example sensor readings
mag = ([30.5, -12.8, 45.3])  # x, y, z components of the magnetic field
grav = ([0.0, 9.81, 0.0])  # x, y, z components of gravity

#run get angles code
azimuth, inclination, roll = calib.get_angles(mag, grav)
print(azimuth,inclination)

#----------------------------------------------------------------------------------------
#Save calibration

# Convert calibration object to a dictionary
calibration_dict = calib.as_dict()

# Save the calibration to a file on the device
with open("/calibration_data.json", "w") as file:
    json.dump(calibration_dict, file)

print("Calibration data saved.")