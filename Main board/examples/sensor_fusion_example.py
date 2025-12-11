import time
import math
import json
from sensor_manager import SensorManager
from mag_cal.calibration import Calibration

# ============================
# Load Calibration
# ============================
with open("/calibration_dict.json", "r") as file:
    calibration_dict = json.load(file)

calib = Calibration().from_dict(calibration_dict)
print("Calibration data loaded.")

# ============================
# Setup
# ============================

def measure_gyro_bias(sensor, samples=200, delay=0.002):
    bx = by = bz = 0.0
    get_gyro = sensor.get_gyro

    for _ in range(samples):
        gx, gy, gz = get_gyro()
        bx += gx
        by += gy
        bz += gz
        time.sleep(delay)

    return [bx / samples, by / samples, bz / samples]

sensor = SensorManager()

print("Measuring gyro bias...")
gyro_bias = measure_gyro_bias(sensor)
print("Gyro bias:", gyro_bias)

GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z = gyro_bias

# ============================
# Parameters
# ============================

ALPHA = 0.90            # more responsive
MAG_UPDATE_INTERVAL = 0.3

fused_pitch = 0.0
fused_roll = 0.0
fused_yaw = 0.0
mag_yaw = 0.0

mag_yaw_window = []
last_time = time.monotonic()
last_mag_update = last_time

print_counter = 0

# ============================
# Bind functions locally
# ============================
get_grav = sensor.get_grav
get_mag = sensor.get_mag
get_gyro = sensor.get_gyro
get_cgrav = calib.get_calibrated_grav
get_cmag = calib.get_calibrated_mag
get_angles = calib.get_angles

# ============================
# Accelerometer → pitch/roll
# ============================
def accel_to_pitch_roll(acc):
    ax, ay, az = acc
    try:
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
        roll  = math.degrees(math.atan2(ay, az))
    except ZeroDivisionError:
        return 0.0, 0.0
    return pitch, roll

# ============================
# Main loop
# ============================
while True:

    # Δt
    now = time.monotonic()
    dt = now - last_time
    last_time = now

    # RAW readings
    accel_raw = get_grav()
    mag_raw   = get_mag()
    gx, gy, gz = get_gyro()

    # Apply bias
    gx -= GYRO_BIAS_X
    gy -= GYRO_BIAS_Y
    gz -= GYRO_BIAS_Z

    # CALIBRATED readings
    accel = get_cgrav(accel_raw)
    mag   = get_cmag(mag_raw)

    # Gyro integration
    fused_pitch += gx * dt
    fused_roll  += gy * dt
    fused_yaw   = (fused_yaw + gz * dt) % 360

    # Complementary filter (pitch/roll)
    accel_pitch, accel_roll = accel_to_pitch_roll(accel)
    fused_pitch = ALPHA * fused_pitch + (1 - ALPHA) * accel_pitch
    fused_roll  = ALPHA * fused_roll  + (1 - ALPHA) * accel_roll

    # Magnetometer yaw (slow, non-blocking)
    if now - last_mag_update >= MAG_UPDATE_INTERVAL:
        try:
            mag_yaw_update, _, _ = get_angles(mag, accel)
            mag_yaw_update %= 360

            mag_yaw_window.append(mag_yaw_update)
            if len(mag_yaw_window) > 3:
                mag_yaw_window.pop(0)

            mag_yaw = sum(mag_yaw_window) / len(mag_yaw_window)
        except Exception:
            pass

        last_mag_update = now

    # Output
    print_counter += 1
    if print_counter >= 10:
        print((fused_pitch,))
        print_counter = 0

    # Zero-cost yield
    time.sleep(0)
