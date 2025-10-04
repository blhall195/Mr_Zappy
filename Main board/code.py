from display_manager import DisplayManager
display = DisplayManager()
print("Loading\nPlease wait...")

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
stability_tolerance = float(os.getenv("stability_tolerance") or 0.5) #sets the noise tolerance before a reading is registered, there must be 3 consistent readings within this range for it to accept the reading
leg_angle_tolerance = float(os.getenv("leg_angle_tolerance") or 1.7) #degrees - same as default setting on sexytopo
leg_distance_tolerance = float(os.getenv("leg_distance_tolerance") or 0.05) #m - same as default setting on sexytopo
laser_distance_offset = float(os.getenv("laser_distance_offset") or 0.14) # in m accounts for length of device

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

    def is_consistent(buffer, threshold = stability_tolerance):
        base = buffer[0]
        for other in buffer[1:]:
            if abs(base - other) > threshold:
                return False
        return True

    def is_consistent_leg_angle(buffer, threshold = leg_angle_tolerance):
        base = buffer[0]
        for other in buffer[1:]:
            if abs(base - other) > threshold:
                return False
        return True

    def is_consistent_leg_distance(buffer, threshold = leg_distance_tolerance):
        base = buffer[0]
        for other in buffer[1:]:
            if abs(base - other) > threshold:
                return False
        return True

    azimuth_buffer = []
    inclination_buffer = []
    waiting_for_stable_measurement = False
    laser_enabled = False
    buzzer_enabled = False
    prev_azimuth = None
    current_disco_color = None
    stable_azimuth_buffer = []
    stable_inclination_buffer = []
    stable_distance_buffer = []
    ble_disconnection_counter = 0 #counts the number of pending readings and reports them on the display
    display.update_BT_number(ble_disconnection_counter)

    while True:

        if device.current_state == SystemState.IDLE:
            await asyncio.sleep(0.2)  # Slower loop for Idle

        elif device.current_state == SystemState.TAKING_MEASURMENT:
            if not buzzer_enabled:
                sensor_manager.set_buzzer(True)
                sensor_manager.set_buzzer(False)
                buzzer_enabled = True

            if current_disco_color != "red":
                disco_mode.turn_off()
                disco_mode.set_red()
                current_disco_color = "red"

            if not laser_enabled:
                sensor_manager.set_laser(True)
                laser_enabled = True

            if not device.measurement_taken:
                if not waiting_for_stable_measurement:
                    waiting_for_stable_measurement = True

                update_readings(readings)

                # Buffer azimuth and inclination only
                azimuth_buffer.append(readings.azimuth)
                inclination_buffer.append(readings.inclination)

                if len(azimuth_buffer) > 5:
                    azimuth_buffer.pop(0)
                    inclination_buffer.pop(0)

                if len(azimuth_buffer) == 5:
                    if is_consistent(azimuth_buffer) and is_consistent(inclination_buffer):

                        # ✅ Compute average azimuth & inclination
                        readings.azimuth = sum(azimuth_buffer) / len(azimuth_buffer)
                        readings.inclination = sum(inclination_buffer) / len(inclination_buffer)

                        try:
                            # Read distance
                            distance_cm = sensor_manager.get_distance()
                            if distance_cm is None or not isinstance(distance_cm, (int, float)):
                                raise ValueError("Invalid distance reading")
                            readings.distance = (distance_cm / 100) + laser_distance_offset

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
                            sensor_manager.set_buzzer(True)
                            buzzer_enabled = False
                            sensor_manager.set_laser(True)
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
                            sensor_manager.set_buzzer(True)
                            buzzer_enabled = False
                            sensor_manager.set_laser(True)
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
                            sensor_manager.set_buzzer(True)
                            buzzer_enabled = False
                            sensor_manager.set_laser(True)
                            device.measurement_taken = True

                        except Exception as e:
                            print(f"❌ Distance read failed or other error: {e}")
                            sensor_manager.set_buzzer(True)
                            buzzer_enabled = False
                            sensor_manager.set_laser(True)
                            readings.distance = "ERR"

                        else:
                            # No anomaly detected: send BLE message
                            disco_mode.set_green()
                            sensor_manager.set_laser(True)
                            await asyncio.sleep(0.1)
                            sensor_manager.set_buzzer(True)
                            sensor_manager.set_laser(False)
                            await asyncio.sleep(0.1)

                            sensor_manager.set_laser(True)
                            buzzer_enabled = False
                            ble.send_message(readings.azimuth, readings.inclination, readings.distance)
                            if not device.ble_connected:
                                ble_disconnection_counter += 1
                                display.update_BT_number(ble_disconnection_counter)
                            else:
                                ble_disconnection_counter = 0
                                display.update_BT_number(ble_disconnection_counter)

                            # Append stable reading to stable buffers
                            stable_azimuth_buffer.append(readings.azimuth)
                            stable_inclination_buffer.append(readings.inclination)
                            stable_distance_buffer.append(readings.distance)

                            # Keep only last 3 stable readings
                            if len(stable_azimuth_buffer) > 3:
                                stable_azimuth_buffer.pop(0)
                                stable_inclination_buffer.pop(0)
                                stable_distance_buffer.pop(0)

                            # Check if last 3 stable readings are consistent
                            if len(stable_azimuth_buffer) == 3:
                                if is_consistent_leg_angle(stable_azimuth_buffer) and is_consistent_leg_angle(stable_inclination_buffer) and is_consistent_leg_distance(stable_distance_buffer):
                                    for _ in range(3):
                                        sensor_manager.set_buzzer(True)
                                        disco_mode.set_white()
                                        await asyncio.sleep(0.1)
                                        disco_mode.turn_off()

                                    stable_azimuth_buffer.clear()
                                    stable_inclination_buffer.clear()
                                    stable_distance_buffer.clear()

                        device.measurement_taken = True

                        # Update display regardless of success or failure
                        display.update_sensor_readings(readings.distance, readings.azimuth, readings.inclination)

                        disco_mode.turn_off()
                        current_disco_color = None
                        device.current_state = SystemState.IDLE


async def watch_for_button_presses(device):
    calibrate_button_start = None
    disco_on = False
    laser_flag = False

    while True:
        button_manager.update()

        #Button 1 logic:
        if button_manager.was_pressed("Button 1"):
            signal_activity()
            if laser_flag:
                sensor_manager.set_buzzer(True)
                sensor_manager.set_laser(True)
                sensor_manager.set_buzzer(False)
                laser_flag = False
            elif device.current_state == "IDLE":
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
                sensor_manager.set_buzzer(False)
                sensor_manager.set_laser(False)
                laser_flag = True
                laser_enabled = False
                print("Disco mode OFF (Button 2 pressed)")
            else:
                sensor_manager.set_buzzer(False)
                disco_mode.start_disco()
                disco_on = True
                sensor_manager.set_laser(False)
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

        await asyncio.sleep(0.0005)

def update_readings(readings,):
    try:
        readings.azimuth, readings.inclination, readings.roll = readings.calib_updated.get_angles(
            sensor_manager.get_mag(), sensor_manager.get_grav()
        )
        #print((readings.azimuth,))
        #print((readings.inclination,))

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
        await asyncio.sleep(0.3)


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
                display.update_BT_number(0)
            elif msg in COMMANDS:
                cmd_byte = COMMANDS[msg]
                handle_command(cmd_byte, device, sensor_manager)
        await asyncio.sleep(0.01)


last_activity_time = 0
ACTIVITY_TIMEOUT = auto_shutdown_delay  # seconds

def signal_activity():
    global last_activity_time
    last_activity_time = time.monotonic()

async def auto_switch_off_timount():
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
    auto_shutdown_task = asyncio.create_task(auto_switch_off_timount())

    while True:
        # Wait until calibration is triggered
        while device.current_state != SystemState.CALIBRATING:
            await asyncio.sleep(0.2)

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
