import time
time.sleep(0.2)

import board
import digitalio

cfg_pin = digitalio.DigitalInOut(board.SCK)
cfg_pin.direction = digitalio.Direction.INPUT
cfg_pin.pull = digitalio.Pull.UP  # internal pull-up

ble_status_pin = digitalio.DigitalInOut(board.D11)
ble_status_pin.direction = digitalio.Direction.INPUT
ble_status_pin.pull = digitalio.Pull.DOWN


from display_manager import DisplayManager
display = DisplayManager()
print("Loading\nPlease wait...")

from calibration_manager import CalibrationFlags
calibration_flags = CalibrationFlags()

# Setup power pin for LTC2952 shutdown (after CalibrationFlags to avoid conflict with menu_manager)
pwr_pin = digitalio.DigitalInOut(board.A2)
pwr_pin.direction = digitalio.Direction.OUTPUT
pwr_pin.value = not cfg_pin.value

import os
import asyncio
import math
from sensor_manager import SensorManager
from button_manager import ButtonManager
from ble_manager import BleManager
from disco_manager import DiscoMode
import json
import microcontroller
from mag_cal.calibration import Calibration, MagneticAnomalyError, GravityAnomalyError, DipAnomalyError, Strictness
from laser_egismos import LaserError
from config import Config

CONFIG = Config()
sensor_manager = SensorManager()
sensor_manager.set_buzzer(False)
sensor_manager.set_laser(True)
sensor_manager.set_buzzer(True)
button_manager = ButtonManager()
ble = BleManager()
ble.set_name("SAP6_Unicorn")
disco_mode = DiscoMode(sensor_manager, brightness=1)
time.sleep(0.05)#prevents brownout
display.display_screen_initialise()

class SystemState:
    IDLE = "IDLE"
    TAKING_MEASURMENT = "TAKING_MEASURMENT"
    MENU = "MENU"
    DISPLAYING = "DISPLAYING"

class Readings:
    def __init__(self):
        self.azimuth = 0
        self.inclination = 0
        self.roll = 0
        self.distance = 0
        self.calib_updated = None
        self.battery_level = 0

class DeviceContext:
    """Central device state container"""
    def __init__(self):
        self.current_state = SystemState.IDLE
        self.readings = Readings()
        self.measurement_taken = False

        # Shared flags & buffers
        self.laser_enabled = True
        self.buzzer_enabled = False
        self.disco_on = False
        self.laser_on_flag = True
        self.current_disco_color = None
        self.ble_connected = False
        self.ble_disconnection_counter = 0
        self.ble_readings_transfered_flag = False
        self.last_activity_time = time.monotonic()
        self.purple_latched = False
        self.last_measurement_time = 0.0

        # Buffers for stability checking
        self.azimuth_buffer = []
        self.inclination_buffer = []
        self.stable_azimuth_buffer = []
        self.stable_inclination_buffer = []
        self.stable_distance_buffer = []

device = DeviceContext()

calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")

# Load calibration
try:
    with open("/calibration_dict.json", "r") as f:
        calibration_dict = json.load(f)
    calib = calib.from_dict(calibration_dict)
    device.readings.calib_updated = calib
except OSError:
    print("Calibration file not found, please calibrate the device.")
    device.current_state = SystemState.MENU

# ===== Helper Functions =====
def update_readings():
    try:
        mag = sensor_manager.get_mag()
        grav = sensor_manager.get_grav()
        angles = device.readings.calib_updated.get_angles(mag, grav)
        device.readings.azimuth, device.readings.inclination, device.readings.roll = angles
    except Exception as e:
        print(f"[ERROR] update_readings exception: {type(e).__name__}: {e}")

def ema_stable_update(new_az, new_inc):
    alpha = CONFIG.EMA_alpha

    prev_az = device.azimuth_buffer[-1] if device.azimuth_buffer else new_az
    prev_inc = device.inclination_buffer[-1] if device.inclination_buffer else new_inc

    smoothed_az = alpha * new_az + (1 - alpha) * prev_az
    smoothed_inc = alpha * new_inc + (1 - alpha) * prev_inc

    device.azimuth_buffer.append(smoothed_az)
    device.inclination_buffer.append(smoothed_inc)

    if len(device.azimuth_buffer) > CONFIG.stability_buffer_length:
        device.azimuth_buffer.pop(0)
        device.inclination_buffer.pop(0)

    if len(device.azimuth_buffer) < CONFIG.stability_buffer_length:
        return False, smoothed_az, smoothed_inc

    az_diff = max(device.azimuth_buffer) - min(device.azimuth_buffer)
    inc_diff = max(device.inclination_buffer) - min(device.inclination_buffer)
    stable = az_diff < CONFIG.stability_tolerance and inc_diff < CONFIG.stability_tolerance

    #print(az_diff,)

    return stable, smoothed_az, smoothed_inc


#  TOLERANCE CHECK HELPERS
def circular_diff(a, b):
    return abs((a - b + 180) % 360 - 180)

def bearings_within_tol(values, tol):
    if len(values) != 3:
        return False
    for i in range(3):
        for j in range(i + 1, 3):
            if circular_diff(values[i], values[j]) > tol:
                return False
    return True

def linear_within_tol(values, tol):
    if len(values) != 3:
        return False
    for i in range(3):
        for j in range(i + 1, 3):
            if abs(values[i] - values[j]) > tol:
                return False
    return True

def update_buffer(buf, value):
    buf.append(value)
    if len(buf) > 3:
        buf.pop(0)

def save_reading_to_file(az, inc, dist):
    try:
        with open("/pending_readings.txt", "a") as f:
            f.write(f"{az},{inc},{dist}\n")
    except Exception as e:
        print("File write error:", e)

async def flush_file_to_ble(ble):
    try:
        with open("/pending_readings.txt", "r") as f:
            for line in f:
                az, inc, dist = line.strip().split(",")
                ble.send_message(float(az), float(inc), float(dist))
                await asyncio.sleep(0.05)  # yields properly
    except OSError:
        return

    await asyncio.sleep(0.2)

    try:
        os.remove("/pending_readings.txt")
    except Exception as e:
        print("Failed to delete pending file:", e)

def count_pending_readings():
    try:
        with open("/pending_readings.txt", "r") as f:
            return sum(1 for _ in f)
    except OSError:
        return 0

pending = count_pending_readings()
device.ble_disconnection_counter = pending
display.update_BT_number(pending)

# ===== Async Functions =====
async def alert_error(err_code, flashes=4):
    try:
        device.readings.distance = err_code
        disco_mode.turn_off()
        for _ in range(flashes):
            disco_mode.set_red()
            await asyncio.sleep(0.1)
            disco_mode.turn_off()
            await asyncio.sleep(0.1)
        sensor_manager.set_buzzer(True)
        device.buzzer_enabled = False
        sensor_manager.set_laser(True)
        device.laser_enabled = True
        device.measurement_taken = True
    except Exception as e:
        print(f"[ERROR] alert_error exception: {type(e).__name__}: {e}")

async def handle_success():
    try:
        device.last_measurement_time = time.monotonic()
        disco_mode.set_green()
        sensor_manager.set_laser(True)
        await asyncio.sleep(0.1)
        sensor_manager.set_buzzer(True)
        sensor_manager.set_laser(False)
        await asyncio.sleep(0.1)
        sensor_manager.set_laser(True)
        device.buzzer_enabled = False

        if device.ble_connected:
            ble.send_message(device.readings.azimuth, device.readings.inclination, device.readings.distance)
            device.ble_disconnection_counter = 0
        else:
            device.ble_disconnection_counter += 1
            display.update_BT_number(device.ble_disconnection_counter)
            save_reading_to_file(device.readings.azimuth,
                                 device.readings.inclination,
                                 device.readings.distance)


        update_buffer(device.stable_azimuth_buffer, device.readings.azimuth)
        update_buffer(device.stable_inclination_buffer, device.readings.inclination)
        update_buffer(device.stable_distance_buffer, device.readings.distance)

        # Only evaluate if we *actually* have 3 samples per channel
        if (len(device.stable_azimuth_buffer) == 3 and
            len(device.stable_inclination_buffer) == 3 and
            len(device.stable_distance_buffer) == 3):

            az_ok = bearings_within_tol(
                device.stable_azimuth_buffer,
                CONFIG.leg_angle_tolerance
            )

            inc_ok = linear_within_tol(
                device.stable_inclination_buffer,
                CONFIG.leg_angle_tolerance
            )

            dist_ok = linear_within_tol(
                device.stable_distance_buffer,
                CONFIG.leg_distance_tolerance
            )


            # All three must be stable
            if az_ok and inc_ok and dist_ok:

                # Buzz + flash 3×
                for _ in range(3):
                    sensor_manager.set_buzzer(True)
                    disco_mode.set_white()
                    await asyncio.sleep(0.1)
                    sensor_manager.set_buzzer(False)
                    disco_mode.turn_off()

                # Clear buffers for next leg
                device.stable_azimuth_buffer.clear()
                device.stable_inclination_buffer.clear()
                device.stable_distance_buffer.clear()

                # Latch purple mode
                disco_mode.set_purple()
                device.purple_latched = True
                device.current_disco_color = "purple"

                device.measurement_taken = True
                return

        device.measurement_taken = False
    except Exception as e:
        print(f"[ERROR] handle_success exception: {type(e).__name__}: {e}")

# ===== Async Task =====
async def sensor_read_display_update():
    while True:
        try:
            if device.current_state == SystemState.IDLE:
                await asyncio.sleep(0.2)
                continue

            # remove purple_latched reset here
            if device.current_state != SystemState.TAKING_MEASURMENT:
                await asyncio.sleep(0.05)
                continue

            if device.measurement_taken:
                await asyncio.sleep(0.05)
                continue

            if not device.buzzer_enabled:
                sensor_manager.set_buzzer(True)
                sensor_manager.set_buzzer(False)
                device.buzzer_enabled = True
            print(f"DEBUG: state={device.current_state}, purple={device.purple_latched}, taken={device.measurement_taken}")

            if not device.purple_latched:
                if device.current_disco_color != "red":
                    device.current_disco_color = "red"
                    disco_mode.set_red()

            if not device.laser_enabled:
                sensor_manager.set_laser(True)
                device.laser_enabled = True

            update_readings()
            stable, smoothed_az, smoothed_inc = ema_stable_update(device.readings.azimuth, device.readings.inclination)

            if stable:
                device.readings.azimuth = smoothed_az
                device.readings.inclination = smoothed_inc

                try:
                    distance_cm = sensor_manager.get_distance()
                    if distance_cm is None or not isinstance(distance_cm, (int, float)):
                        raise ValueError("Invalid distance reading")
                    device.readings.distance = (distance_cm / 100) + CONFIG.laser_distance_offset

                    if CONFIG.anomaly_detection:
                        strictness = Strictness(CONFIG.mag_tolerance, CONFIG.grav_tolerance, CONFIG.dip_tolerance)
                        calib.raise_if_anomaly(sensor_manager.get_mag(), sensor_manager.get_grav(), strictness)

                except (MagneticAnomalyError, GravityAnomalyError, DipAnomalyError) as e:
                    abbrev = {"MagneticAnomalyError":"MagErr","GravityAnomalyError":"GravErr","DipAnomalyError":"DipErr"}
                    await alert_error(abbrev.get(type(e).__name__, "Err"))

                except LaserError as e:
                    print(f"[ERROR] Laser error: {type(e).__name__}: {e}")
                    sensor_manager.reset_laser()
                    await alert_error("LzrERR")

                except Exception as e:
                    print(f"[ERROR] Distance read failed: {type(e).__name__}: {e}")
                    sensor_manager.reset_laser()
                    await alert_error("ERR")

                else:
                    await handle_success()

                display.update_sensor_readings(device.readings.distance, device.readings.azimuth, device.readings.inclination)
                if not device.purple_latched:
                    disco_mode.turn_off()
                    device.current_disco_color = None
                device.current_state = SystemState.IDLE


            await asyncio.sleep(0.02)

        except Exception as e:
            print(f"[ERROR] sensor_read_display_update main loop exception: {type(e).__name__}: {e}")
            await asyncio.sleep(0.1)


async def watch_for_button_presses():
    calibrate_button_start = None
    button2_hold_start = None
    device.laser_on_flag = False

    while True:
        button_manager.update()
        # Button logic
        if button_manager.was_pressed("Button 1"):
            device.purple_latched = False
            device.last_activity_time = time.monotonic()
            if device.laser_enabled:
                sensor_manager.set_laser(True)
                device.current_state = SystemState.TAKING_MEASURMENT
                device.measurement_taken = False
                device.laser_enabled = True
            elif device.current_state == SystemState.IDLE:
                sensor_manager.set_buzzer(True)
                sensor_manager.set_laser(True)
                sensor_manager.set_buzzer(False)
                device.laser_enabled = True
            else:
                print("System busy, ignoring button press.")
                device.current_state = SystemState.IDLE

        # Button 2: short press = disco toggle, hold 5s = snake game
        if button_manager.is_pressed("Button 2"):
            device.last_activity_time = time.monotonic()
            if button2_hold_start is None:
                button2_hold_start = time.monotonic()
            else:
                held_time = time.monotonic() - button2_hold_start
                if held_time >= 5.0 and device.current_state == SystemState.IDLE:
                    print("Launching Snake Game (Button 2 held 5s)")
                    sensor_manager.set_laser(False)
                    disco_mode.turn_off()
                    device.disco_on = False
                    import gc
                    gc.collect()
                    from snake import start_snake_game
                    await start_snake_game(display, button_manager, disco_mode, pwr_pin)
                    gc.collect()
                    button2_hold_start = None
        else:
            # Button released - check if it was a short press for disco toggle
            if button2_hold_start is not None:
                held_time = time.monotonic() - button2_hold_start
                if held_time < 5.0:
                    # Short press - toggle disco mode
                    if device.disco_on:
                        disco_mode.turn_off()
                        device.disco_on = False
                        sensor_manager.set_buzzer(False)
                        sensor_manager.set_laser(False)
                        device.laser_enabled = False
                    else:
                        sensor_manager.set_buzzer(False)
                        disco_mode.start_disco()
                        device.disco_on = True
                        sensor_manager.set_laser(False)
            button2_hold_start = None

        if button_manager.was_pressed("Button 4"):
            now = time.monotonic()
            elapsed = now - device.last_measurement_time
            if elapsed < 1.5:  # 1.5 seconds guard
                print("Measurement just taken, delaying shutdown")
                await asyncio.sleep(1.5 - elapsed)
            pwr_pin.value = False

        if button_manager.is_pressed("Button 3"):
            device.last_activity_time = time.monotonic()
            if calibrate_button_start is None:
                calibrate_button_start = time.monotonic()
            else:
                held_time = time.monotonic() - calibrate_button_start
                if held_time >= 2.0 and device.current_state == SystemState.IDLE:
                    print("Entering calibration mode (Button 3 held 2s)")
                    #display.show_starting_menu()
                    sensor_manager.set_laser(False)
                    # Write file and reset once
                    if "menu_mode.txt" not in os.listdir("/"):
                        try:
                            with open("/menu_mode.txt", "w") as f:
                                f.write("1")
                            microcontroller.reset()
                        except OSError as e:
                            print(f"Failed to write menu_mode.txt: {e}")
                            microcontroller.reset()
                    calibrate_button_start = None  # reset after triggering
        else:
            calibrate_button_start = None

          # Adaptive delay to improve CPU perfomance when taking a measurment
        if device.current_state == SystemState.TAKING_MEASURMENT:
            await asyncio.sleep(0.1)  # longer delay during measurement
        else:
            await asyncio.sleep(0.05)  # very short delay otherwise


async def check_battery_sensor():
    while True:
        device.readings.battery_level = sensor_manager.get_bat()
        display.update_battery(device.readings.battery_level)
        await asyncio.sleep(30)


async def monitor_ble_pin():
    last_connected = False

    while True:
        current_connected = ble_status_pin.value
        device.ble_connected = current_connected
        # Flush once upon new connection
        if current_connected and not last_connected:
            if device.ble_disconnection_counter > 0:
                # Set the flag to False to indicate "transfer in progress"
                device.ble_readings_transfered_flag = False
                display.update_BT_number("...")
                disco_mode.set_blue()
                await asyncio.sleep(1)  # allow BLE slave to be ready before flushing
                await flush_file_to_ble(ble)
                disco_mode.turn_off()
        # Update BT symbol
        display.update_BT_label(current_connected)
        # Update transfer indicator
        if current_connected:
            if device.ble_readings_transfered_flag:
                display.update_BT_number(0)
                device.ble_disconnection_counter = 0
            else:
                display.update_BT_number("")
        else:
            display.update_BT_number(device.ble_disconnection_counter)
        last_connected = current_connected
        await asyncio.sleep(0.3)


async def monitor_ble_uart():
    while True:
        msg = ble.read_message()
        if msg is not None:
            print(f"📥 UART message from slave: {msg}")

            # Ensure msg is an integer
            try:
                cmd = int(msg)  # convert string to int if needed
            except ValueError:
                print(f"❌ Invalid command format: {msg}")
                await asyncio.sleep(0.01)
                continue
            if cmd == 0x55:  # ACK0
                print("✅ ACK0 received")
                device.ble_readings_transfered_flag = True
            elif cmd == 0x56:  # ACK1
                print("✅ ACK1 received")
                device.ble_readings_transfered_flag = True
            elif cmd == 0x31:  # START_CAL
                print("⚙️ START_CAL received")
                device.current_state = SystemState.MENU
            elif cmd == 0x30:  # STOP_CAL
                print("⚙️ STOP_CAL received")
                device.current_state = SystemState.IDLE
            elif cmd == 0x36:  # LASER_ON
                print("💡 LASER_ON received")
                sensor_manager.set_laser(True)
            elif cmd == 0x37:  # LASER_OFF
                print("💡 LASER_OFF received")
                sensor_manager.set_laser(False)
            elif cmd == 0x34:  # DEVICE_OFF
                print("🛑 DEVICE_OFF received")
                # implement shutdown logic
            elif cmd == 0x38:  # TAKE_SHOT
                print("📸 TAKE_SHOT received")
                device.current_state = SystemState.TAKING_MEASURMENT
                device.measurement_taken = False
                device.last_activity_time = time.monotonic()
            else:
                print(f"❌ Unknown command received: {cmd}")

          # Adaptive delay
        if device.current_state == SystemState.TAKING_MEASURMENT:
            await asyncio.sleep(0.1)  # longer delay during measurement
        else:
            await asyncio.sleep(0.01)  # very short delay otherwise

async def auto_switch_off_timeount():
    while True:
        now = time.monotonic()
        if now - device.last_activity_time > CONFIG.auto_shutdown_timeout:
            try:
                print("Inactivity timeout, shutting down device")
                pwr_pin.value = False  # sets A1 high
                #insert shutdown code
            except Exception as e:
                print(f"Error shutting down device: {e}")
        await asyncio.sleep(5)

async def laser_timeout_watch():
    while True:
        now = time.monotonic()
        if device.laser_enabled and (now - device.last_activity_time > CONFIG.laser_timeout):
            try:
                print("Laser timeout reached, turning laser off")
                sensor_manager.set_laser(False)
                device.laser_enabled = False
            except Exception as e:
                print(f"Error turning off laser: {e}")
        await asyncio.sleep(1)  # check every second



async def main():
    tasks = [
        asyncio.create_task(sensor_read_display_update()),
        asyncio.create_task(watch_for_button_presses()),
        asyncio.create_task(check_battery_sensor()),
        asyncio.create_task(monitor_ble_pin()),
        asyncio.create_task(monitor_ble_uart()),
        asyncio.create_task(auto_switch_off_timeount()),
        asyncio.create_task(laser_timeout_watch())
    ]

    while True:
        while device.current_state != SystemState.MENU:
            await asyncio.sleep(0.2)



asyncio.run(main())
