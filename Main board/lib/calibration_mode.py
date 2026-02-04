import os
import json
import asyncio
import board
import digitalio
import time
import math

from sensor_manager import SensorManager
from button_manager import ButtonManager
from calibration_manager import PerformCalibration
from mag_cal.calibration import Calibration, MagneticAnomalyError, GravityAnomalyError, DipAnomalyError, Strictness
from ble_manager import BleManager
from disco_manager import DiscoMode

# Optional patch for roll calibration
try:
    from calibrate_roll import align_sensor_roll
    if not hasattr(Calibration, "align_sensor_roll"):
        Calibration.align_sensor_roll = align_sensor_roll
except Exception as e:
    print(f"❌ Failed to patch align_sensor_roll: {e}")

# Delete old calibration flag if present
if "calibration_mode.txt" in os.listdir("/"):
    try:
        os.remove("/calibration_mode.txt")
    except OSError as e:
        print("Error deleting file:", e)

# Setup power pin for LTC2952 shutdown
pwr_pin = digitalio.DigitalInOut(board.A2)
pwr_pin.direction = digitalio.Direction.OUTPUT
pwr_pin.value = True  # Keep power on

# Initialize all components
sensor_manager = SensorManager()
sensor_manager.set_laser(True)
button_manager = ButtonManager()
ble = BleManager()
disco_mode = DiscoMode(sensor_manager, brightness=1)
calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")

# Store readings
class Readings:
    def __init__(self):
        self.azimuth = 0
        self.inclination = 0
        self.roll = 0
        self.distance = 0
        self.calib_updated = None
        self.battery_level = 0
readings = Readings()

# System states
class SystemState:
    IDLE = "IDLE"
    TAKING_MEASURMENT = "TAKING_MEASURMENT"
    CALIBRATING = "CALIBRATING"
    DISPLAYING = "DISPLAYING"
    SHUTTING_DOWN = "SHUTTING_DOWN"

# Device context
class DeviceContext:
    def __init__(self):
        self.current_state = SystemState.CALIBRATING
        self.readings = Readings()
        self.measurement_taken = False

device = DeviceContext()

# Try to load saved calibration
try:
    with open("/calibration_dict.json", "r") as f:
        calibration_dict = json.load(f)
    calib = calib.from_dict(calibration_dict)
    readings.calib_updated = calib.from_dict(calibration_dict)
except OSError:
    print("⚠️ Calibration file not found. Proceeding to calibration...")

# BLE monitoring
async def monitor_ble_uart(ble_manager, device, sensor_manager):
    while True:
        msg = ble_manager.read_message()
        if msg:
            if msg == "Shutting_Down":
                print("\nShutdown in...")
                await asyncio.sleep(1)
                print("   3")
                await asyncio.sleep(1)
                print("   2")
                await asyncio.sleep(1)
                print("   1")
        await asyncio.sleep(0.01)

# Activity timeout / power management
last_activity_time = time.monotonic()
ACTIVITY_TIMEOUT = 2700  # 45 min

def signal_activity():
    global last_activity_time
    last_activity_time = time.monotonic()

async def send_keep_alive_periodically(ble_manager):
    global last_activity_time
    while True:
        now = time.monotonic()
        if now - last_activity_time > ACTIVITY_TIMEOUT:
            try:
                print("Shutting Down (timeout)")
                pwr_pin.value = False
            except Exception as e:
                print(f"Error sending keep-alive: {e}")
        await asyncio.sleep(5)

# MAIN
async def main():
    calibration = PerformCalibration(sensor_manager, button_manager, calib, pwr_pin)

    # Background tasks
    asyncio.create_task(monitor_ble_uart(ble, device, sensor_manager))
    asyncio.create_task(send_keep_alive_periodically(ble))

    # User chooses calibration type
    await calibration.wait_for_calibration_choice(device, disco_mode)

asyncio.run(main())
