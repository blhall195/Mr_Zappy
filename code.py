import os

if "calibration_mode.txt" in os.listdir("/"):
    import calibration_mode

if "calculate_ellipsoid.txt" in os.listdir("/"):
    import calibration_calculate_ellipsoid

if "calculate_alignment.txt" in os.listdir("/"):
    import calibration_calculate_alignment

# Load toml file settings
mag_tolerance = float(os.getenv("mag") or 10.0)
grav_tolerance = float(os.getenv("grav") or 10.0)
dip_tolerance = float(os.getenv("dip") or 10.0)
auto_shutdown_delay = int(os.getenv("auto_shutdown_delay") or 1800)
leg_tolerance = float(os.getenv("leg_tolerance") or 1.7)

import asyncio
import board
import digitalio
import time
import math
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
from calibration_manager import PerformCalibration
from mag_cal.calibration import Calibration, MagneticAnomalyError, GravityAnomalyError, DipAnomalyError, Strictness
from ble_manager import BleManager
from disco_manager import DiscoMode
import json
from calibrate_roll import align_sensor_roll
import microcontroller

try:
    from calibrate_roll import align_sensor_roll
    if not hasattr(Calibration, "align_sensor_roll"):
        Calibration.align_sensor_roll = align_sensor_roll
        print("✅ align_sensor_roll successfully patched into Calibration")
except Exception as e:
    print(f"❌ Failed to patch align_sensor_roll: {e}")

sensor_manager = SensorManager()
button_manager = ButtonManager()
display = DisplayManager()
ble = BleManager()
disco_mode = DiscoMode(sensor_manager, brightness=1)
calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")

# A class for storing device readings
class Readings:
    def __init__(self):
        self.azimuth = 0
        self.inclination = 0
        self.roll = 0
        self.distance = 0
        self.calib_updated = None
        self.battery_level = 0
readings = Readings()

#class for storing system states
class SystemState():
    IDLE = "IDLE"
    TAKING_MEASURMENT = "TAKING_MEASURMENT"
    CALIBRATING = "CALIBRATING"
    DISPLAYING = "DISPLAYING"

system_state = SystemState()

#class for updating system states
class DeviceContext:
    def __init__(self):
        self.current_state = SystemState.IDLE
        self.readings = Readings()
        self.measurement_taken = False  # NEW FLAG

device = DeviceContext()
device.current_state = SystemState.IDLE

try:
    with open("/calibration_dict.json", "r") as f:
        calibration_dict = json.load(f)
    calib = calib.from_dict(calibration_dict)
    readings.calib_updated = calib.from_dict(calibration_dict)
except OSError:
    print("Calibration file not found, using default calibration.")
    calibration = PerformCalibration(sensor_manager, button_manager, calib)
    asyncio.run(calibration.start_calibration(device, disco_mode))
    device.current_state = "CALIBRATING"

COMMANDS = {
    "ACK0": 0x55,
    "ACK1": 0x56,
    "START_CAL": 0x31,
    "STOP_CAL": 0x30,
    "LASER_ON": 0x36,
    "LASER_OFF": 0x37,
    "DEVICE_OFF": 0x34,
    "TAKE_SHOT": 0x38,
}

display.display_screen_initialise()

ble_status_pin = digitalio.DigitalInOut(board.D11)
ble_status_pin.direction = digitalio.Direction.INPUT
ble_status_pin.pull = digitalio.Pull.DOWN


async def sensor_read_display_update(readings, device):
    prev_azimuth = None
    laser_enabled = False  # Track laser state manually

    def safe_number(val):
        return isinstance(val, (int, float)) and math.isfinite(val)

    def is_consistent(buffer, threshold=0.5):
        base = buffer[0]
        for other in buffer[1:]:
            if abs(base - other) > threshold:
                return False
        return True

    azimuth_buffer = []
    inclination_buffer = []
    waiting_for_stable_measurement = False
    laser_enabled = False
    prev_azimuth = None
    current_disco_color = None
    stable_azimuth_buffer = []
    stable_inclination_buffer = []

    while True:
        if device.current_state == SystemState.IDLE:
            if laser_enabled:
                sensor_manager.set_laser(True)
                laser_enabled = False

            update_readings(readings)

            if safe_number(readings.azimuth) and safe_number(readings.inclination):
                if prev_azimuth is None or abs(readings.azimuth - prev_azimuth) >= 0.2:
                    display.update_sensor_readings(0, readings.azimuth, readings.inclination)
                    prev_azimuth = readings.azimuth

            # Reset buffers if entering idle
            azimuth_buffer.clear()
            inclination_buffer.clear()
            waiting_for_stable_measurement = False

        elif device.current_state == SystemState.TAKING_MEASURMENT:

            if current_disco_color != "red":
                disco_mode.turn_off()
                disco_mode.set_red()
                current_disco_color = "red"

            if not laser_enabled:
                sensor_manager.set_laser(True)
                laser_enabled = True

            if not device.measurement_taken:
                if not waiting_for_stable_measurement:
                    print("wait")
                    waiting_for_stable_measurement = True

                update_readings(readings)

                # Buffer azimuth and inclination only
                azimuth_buffer.append(readings.azimuth)
                inclination_buffer.append(readings.inclination)

                if len(azimuth_buffer) > 3:
                    azimuth_buffer.pop(0)
                    inclination_buffer.pop(0)

                if len(azimuth_buffer) == 3:
                    if is_consistent(azimuth_buffer) and is_consistent(inclination_buffer):

                        # ✅ Compute average azimuth & inclination
                        readings.azimuth = sum(azimuth_buffer) / len(azimuth_buffer)
                        readings.inclination = sum(inclination_buffer) / len(inclination_buffer)

                        try:
                            # Read distance
                            distance_cm = sensor_manager.get_distance()
                            if distance_cm is None or not isinstance(distance_cm, (int, float)):
                                raise ValueError("Invalid distance reading")
                            readings.distance = distance_cm / 100

                            # Perform anomaly check BEFORE sending
                            strictness = Strictness(mag=mag_tolerance, grav=grav_tolerance, dip=dip_tolerance)
                            calib.raise_if_anomaly(sensor_manager.get_mag(), sensor_manager.get_grav(), strictness=strictness)

                        except MagneticAnomalyError:
                            print("❌ Magnetic anomaly detected")
                            readings.distance = "Mag ERR"
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            await asyncio.sleep(0.1)
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            await asyncio.sleep(0.1)
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            device.measurement_taken = True

                        except GravityAnomalyError:
                            print("❌ Gravity anomaly detected – likely motion")
                            readings.distance = "Grav ERR"
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            await asyncio.sleep(0.1)
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            await asyncio.sleep(0.1)
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            device.measurement_taken = True

                        except DipAnomalyError:
                            print("❌ Dip angle anomaly detected")
                            readings.distance = "Dip ERR"
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            await asyncio.sleep(0.1)
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            await asyncio.sleep(0.1)
                            disco_mode.turn_off()
                            await asyncio.sleep(0.1)
                            disco_mode.set_red()
                            device.measurement_taken = True

                        except Exception as e:
                            print(f"❌ Distance read failed or other error: {e}")
                            readings.distance = "ERR"

                        else:
                            # No anomaly detected: send BLE message
                            disco_mode.set_green()
                            sensor_manager.set_buzzer(True)
                            ble.send_message(readings.azimuth, readings.inclination, readings.distance)

                            # Append stable reading to stable buffers
                            stable_azimuth_buffer.append(readings.azimuth)
                            stable_inclination_buffer.append(readings.inclination)

                            # Keep only last 3 stable readings
                            if len(stable_azimuth_buffer) > 3:
                                stable_azimuth_buffer.pop(0)
                                stable_inclination_buffer.pop(0)

                            # Check if last 3 stable readings are consistent
                            if len(stable_azimuth_buffer) == 3:
                                if is_consistent(stable_azimuth_buffer) and is_consistent(stable_inclination_buffer):
                                    for _ in range(3):
                                        sensor_manager.set_buzzer(True)
                                        disco_mode.set_white()
                                        await asyncio.sleep(0.1)
                                        disco_mode.turn_off()

                                    stable_azimuth_buffer.clear()
                                    stable_inclination_buffer.clear()

                        device.measurement_taken = True

                        # Update display regardless of success or failure
                        display.update_sensor_readings(readings.distance, readings.azimuth, readings.inclination)

                        await asyncio.sleep(0.02)
                        disco_mode.turn_off()
                        current_disco_color = None
                        device.current_state = SystemState.DISPLAYING

        if device.current_state == SystemState.IDLE:
            await asyncio.sleep(0.5)  # faster loop for active measuring
        elif device.current_state == SystemState.DISPLAYING:
            await asyncio.sleep(0.5)  # faster loop for active measuring


async def watch_for_button_presses(device):
    calibrate_button_start = None
    disco_on = False

    while True:
        button_manager.update()

        #Button 1 logic:
        if button_manager.was_pressed("Button 1"):
            print("Fire Button pressed!")
            signal_activity()
            if device.current_state == "IDLE":
                device.current_state = "TAKING_MEASURMENT"
                device.measurement_taken = False
            else:
                print("System busy, ignoring button press.")
                device.current_state = "IDLE"

        #Button 2 logic:
        if button_manager.was_pressed("Button 2"):
            signal_activity()
            if disco_on:
                disco_mode.turn_off()
                disco_on = False
                print("Disco mode OFF (Button 2 pressed)")
            else:
                disco_mode.start_disco()
                disco_on = True
                print("Disco mode ON (Button 2 pressed)")

        #Button 3 logic:
        if button_manager.is_pressed("Button 3"):
            signal_activity()
            if calibrate_button_start is None:
                calibrate_button_start = time.monotonic()
            else:
                held_time = time.monotonic() - calibrate_button_start
                if held_time >= 2.0 and device.current_state == SystemState.IDLE:
                    print("Entering calibration mode (Button 3 held 2s)")
                    display.show_starting_calibration()
                    device.current_state = SystemState.CALIBRATING
                    calibrate_button_start = None  # reset to prevent repeated triggers
        elif calibrate_button_start is not None:
            calibrate_button_start = None  # Reset when button released

        await asyncio.sleep(0.001)

def update_readings(readings,):
    try:
        readings.azimuth, readings.inclination, readings.roll = readings.calib_updated.get_angles(
            sensor_manager.get_mag(), sensor_manager.get_grav()
        )
        #print(f"({readings.azimuth})")

    except Exception as e:
        print(e)

async def check_battery_sensor(readings):
    while True:
        readings.battery_level = sensor_manager.get_bat()
        display.update_battery(readings.battery_level)
        await asyncio.sleep(30)


async def monitor_ble_pin(device):
    last_value = None
    while True:
        current_value = ble_status_pin.value
        if current_value != last_value:
            if current_value:
                print("🔵 BLE Status Pin: CONNECTED")
                device.ble_connected = True
                display.update_BT_label(True)
            else:
                print("🔴 BLE Status Pin: DISCONNECTED")
                device.ble_connected = False
                display.update_BT_label(False)
            last_value = current_value
        await asyncio.sleep(0.1)


def handle_command(cmd_byte, device, sensor_manager):
    if cmd_byte == 0x31:  # START_CAL
        device.current_state = "CALIBRATING"
    elif cmd_byte == 0x30:  # STOP_CAL
        device.current_state = "IDLE"
    elif cmd_byte == 0x36:  # LASER_ON
        sensor_manager.set_laser(True)
    elif cmd_byte == 0x37:  # LASER_OFF
        sensor_manager.set_laser(False)
    elif cmd_byte == 0x38:  # TAKE_SHOT
        device.current_state = "TAKING_MEASURMENT"
        device.measurement_taken = False
    elif cmd_byte == "Shutting_Down":  # DEVICE_OFF
        print("🛑 Device OFF command received")
    elif cmd_byte in (0x55, 0x56):  # ACKs
        print("✅ ACK received:", cmd_byte)


async def monitor_ble_uart(ble_manager, device, sensor_manager):
    global ble_ack_received
    while True:
        msg = ble_manager.read_message()
        if msg:
            print(f"📥 UART message from slave: {msg}")
            if msg == "85" or msg == "86":
                disco_mode.set_blue()
                await asyncio.sleep(0.2)
                disco_mode.turn_off()
                await asyncio.sleep(0.1)
                disco_mode.set_blue()
                await asyncio.sleep(0.2)
                disco_mode.turn_off()
                ble_ack_received = False
            elif msg in COMMANDS:
                cmd_byte = COMMANDS[msg]
                handle_command(cmd_byte, device, sensor_manager)
        await asyncio.sleep(0.01)


last_activity_time = 0
ACTIVITY_TIMEOUT = auto_shutdown_delay  # seconds

def signal_activity():
    global last_activity_time
    last_activity_time = time.monotonic()

async def send_keep_alive_periodically(ble_manager):
    global last_activity_time
    while True:
        now = time.monotonic()
        if now - last_activity_time > ACTIVITY_TIMEOUT:
            try:
                #insert command that pulls the switch in pin low
                print("inactivity timeout, shutting down device")
                print(now,last_activity_time, ACTIVITY_TIMEOUT)
            except Exception as e:
                print(f"Error shutting down device: {e}")
        await asyncio.sleep(5)  #check every 5 seconds


async def main():

    # Create and start background tasks once
    calibration = PerformCalibration(sensor_manager, button_manager, calib)
    sensor_reading = asyncio.create_task(sensor_read_display_update(readings, device))
    watch_for_buttons = asyncio.create_task(watch_for_button_presses(device))
    check_battery = asyncio.create_task(check_battery_sensor(readings))
    monitor_ble = asyncio.create_task(monitor_ble_pin(device))
    monitor_ble_uart_task = asyncio.create_task(monitor_ble_uart(ble, device, sensor_manager))
    keep_alive_task = asyncio.create_task(send_keep_alive_periodically(ble))

    while True:
        # Wait until calibration is triggered
        while device.current_state != SystemState.CALIBRATING:
            await asyncio.sleep(0.1)

        print("Entering Calibration Mode...")
        print("")
        print("")

        if "calibration_mode.txt" not in os.listdir("/"):
            try:
                with open("/calibration_mode.txt", "w") as f:
                    f.write("1")
                microcontroller.reset()
            except OSError as e:
                print(f"Failed to write calibration_mode.txt: {e}")
                microcontroller.reset()


        microcontroller.reset()

asyncio.run(main())
