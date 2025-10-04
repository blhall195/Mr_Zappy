# Write your code here :-)
from sensor_manager import SensorManager
import time
import math
import json
from mag_cal.calibration import Calibration

# === Load Calibration ===
calib = Calibration()
with open("/calibration_dict.json", "r") as file:
    calibration_dict = json.load(file)
calib = calib.from_dict(calibration_dict)

print("Calibration data loaded.")

# === Create Sensor Manager ===
sensor = SensorManager()

# === Complementary filter alpha ===
ALPHA = 0.98

# === Initial filtered angles ===
fused_pitch = 0.0
fused_roll = 0.0
fused_yaw = 0.0  # From gyro integration
mag_yaw = 0.0    # Independent yaw from magnetometer

# === Rolling average buffer for mag yaw ===
mag_yaw_window = []

# === Timing setup ===
last_time = time.monotonic()
last_mag_update = 0
MAG_UPDATE_INTERVAL = 0.1  # seconds

# === Accelerometer-only pitch/roll helper ===
def get_pitch_roll_from_accel(accel):
    ax, ay, az = accel
    try:
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180 / math.pi
        roll = math.atan2(ay, az) * 180 / math.pi
    except ZeroDivisionError:
        pitch = 0.0
        roll = 0.0
    return pitch, roll

print_counter = 0

# === Main loop ===
while True:
    # --- Time delta ---
    current_time = time.monotonic()
    dt = current_time - last_time
    last_time = current_time

    # --- Read sensors ---
    gyro = sensor.get_gyro()       # (gx, gy, gz) in deg/sec
    accel = sensor.get_accel()     # (ax, ay, az)

    gx, gy, gz = gyro

    # --- Integrate gyro rates to update orientation ---
    fused_pitch += gx * dt
    fused_roll  += gy * dt
    fused_yaw   += gz * dt
    fused_yaw %= 360  # Keep in 0–360 range

    # --- Get pitch and roll from accelerometer only ---
    accel_pitch, accel_roll = get_pitch_roll_from_accel(accel)

    # --- Apply complementary filter for pitch & roll ---
    fused_pitch = ALPHA * fused_pitch + (1 - ALPHA) * accel_pitch
    fused_roll  = ALPHA * fused_roll  + (1 - ALPHA) * accel_roll

    # --- Read magnetometer occasionally for independent azimuth ---
    if (current_time - last_mag_update) >= MAG_UPDATE_INTERVAL:
        mag = sensor.get_mag()  # (mx, my, mz)
        new_mag_yaw, _, _ = calib.get_angles(mag, accel)  # Only compute mag-based yaw
        new_mag_yaw %= 360

        # --- Rolling average logic ---
        mag_yaw_window.append(new_mag_yaw)
        if len(mag_yaw_window) > 3:
            mag_yaw_window.pop(0)
        mag_yaw = sum(mag_yaw_window) / len(mag_yaw_window)

        last_mag_update = current_time

    # --- Print every 10 cycles (~0.5 seconds if running at 2 kHz) ---
    print_counter += 1
    if print_counter >= 10:
        print((fused_pitch,))
        # To show filtered mag yaw, uncomment:
        #print((mag_yaw,))

        print_counter = 0

    time.sleep(0.0001)  # 2000 Hz loop (adjust as needed)
