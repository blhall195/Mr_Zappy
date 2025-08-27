import asyncio
import board
import digitalio
import time
import math
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
from calibration_manager import PerformCalibration
from mag_cal.calibration import Calibration
from ble_manager import BleManager
from disco_manager import DiscoMode
import json
from calibrate_roll import align_sensor_roll

try:
    from calibrate_roll import align_sensor_roll
    if not hasattr(Calibration, "align_sensor_roll"):
        Calibration.align_sensor_roll = align_sensor_roll
        print("✅ align_sensor_roll successfully patched into Calibration")
except Exception as e:
    print(f"❌ Failed to patch align_sensor_roll: {e}")

# Create instances
calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")
sensor_manager = SensorManager()
button_manager = ButtonManager()
display = DisplayManager()
ble = BleManager()
disco_mode = DiscoMode()

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

with open("/calibration_dict.json", "r") as f:
    calibration_dict = json.load(f)

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

# Update calib from the dictionary
readings.calib_updated = calib.from_dict(calibration_dict)

#class for storing system states
class SystemState():
    IDLE = "IDLE"
    TAKING_MEASURMENT = "TAKING_MEASURMENT"
    CALIBRATING = "CALIBRATING"

system_state = SystemState()

#class for updating system states
class DeviceContext:
    def __init__(self):
        self.current_state = SystemState.IDLE
        self.readings = Readings()
        self.measurement_taken = False  # NEW FLAG

device = DeviceContext()
device.current_state = SystemState.IDLE


async def sensor_read_display_update(readings, device):
    prev_azimuth = None
    laser_enabled = False  # Track laser state manually

    def safe_number(val):
        return isinstance(val, (int, float)) and math.isfinite(val)

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

        elif device.current_state == SystemState.TAKING_MEASURMENT:
            if not laser_enabled:
                sensor_manager.set_laser(True)
                laser_enabled = True

            if not device.measurement_taken:
                update_readings(readings)
                readings.distance = sensor_manager.get_distance() / 100
                display.update_sensor_readings(readings.distance, readings.azimuth, readings.inclination)
                ble.send_message(readings.azimuth, readings.inclination, readings.distance)


                device.measurement_taken = True

        await asyncio.sleep(0.25)


async def watch_for_button_presses(device):
    both_pressed_start = None
    disco_on = False

    while True:
        button_manager.update()

        # Detect if both buttons are pressed simultaneously
        button1_pressed = button_manager.is_pressed("Button 1")  # Assuming you have is_pressed method
        button2_pressed = button_manager.is_pressed("Button 2")

        if button1_pressed and button2_pressed:
            if both_pressed_start is None:
                both_pressed_start = time.monotonic()
            else:
                held_time = time.monotonic() - both_pressed_start
                if held_time >= 3.0:
                    # Toggle disco mode
                    if disco_on:
                        disco_mode.turn_off()
                        disco_on = False
                        print("Disco mode OFF (buttons held 3s)")
                    else:
                        disco_mode.start_disco()
                        disco_on = True
                        print("Disco mode ON (buttons held 3s)")
                    both_pressed_start = None  # Reset to prevent repeated toggling
        else:
            both_pressed_start = None  # Reset timer if buttons not pressed together

        # Your existing single button logic:
        if button_manager.was_pressed("Button 1"):
            print("Fire Button pressed!")
            if device.current_state == "IDLE":
                device.current_state = "TAKING_MEASURMENT"
                device.measurement_taken = False
            else:
                print("System busy, ignoring button press.")
                device.current_state = "IDLE"

        elif button_manager.was_pressed("Button 2"):
            if device.current_state == "IDLE":
                print("Entering calibration mode")
                device.current_state = "CALIBRATING"
            else:
                print("System busy, can't calibrate now.")

        await asyncio.sleep(0.01)



def update_readings(readings,):
    try:
        readings.azimuth, readings.inclination, readings.roll = readings.calib_updated.get_angles(
            sensor_manager.get_mag(), sensor_manager.get_grav()
        )
        print(f"({readings.azimuth})")

    except Exception as e:
        print(e)

async def check_battery_sensor(readings):
    while True:
        readings.battery_level = sensor_manager.get_bat()
        display.update_battery(readings.battery_level)
        print(readings.battery_level)
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
    elif cmd_byte == 0x34:  # DEVICE_OFF
        print("🛑 Device OFF command received")
        # Add shutdown code here
    elif cmd_byte in (0x55, 0x56):  # ACKs
        print("✅ ACK received:", cmd_byte)

async def monitor_ble_uart(ble_manager, device, sensor_manager):
    while True:
        msg = ble_manager.read_message()
        if msg:
            print(f"📥 UART message from slave: {msg}")

            if msg in COMMANDS:
                cmd_byte = COMMANDS[msg]
                handle_command(cmd_byte, device, sensor_manager)

            elif msg.isdigit():
                cmd_byte = int(msg)
                handle_command(cmd_byte, device, sensor_manager)

        await asyncio.sleep(0.01)

async def main():
    calibration = PerformCalibration(sensor_manager, button_manager, calib)

    # Create and start background tasks once
    sensor_reading = asyncio.create_task(sensor_read_display_update(readings, device))
    watch_for_buttons = asyncio.create_task(watch_for_button_presses(device))
    check_battery = asyncio.create_task(check_battery_sensor(readings))
    monitor_ble = asyncio.create_task(monitor_ble_pin(device))
    monitor_ble_uart_task = asyncio.create_task(monitor_ble_uart(ble, device, sensor_manager))

    while True:
        # Wait until calibration is triggered
        while device.current_state != SystemState.CALIBRATING:
            await asyncio.sleep(0.1)

        # Calibration triggered: stop background tasks
        sensor_reading.cancel()
        watch_for_buttons.cancel()
        check_battery.cancel()
        try:
            await asyncio.gather(sensor_reading, watch_for_buttons, check_battery)
        except asyncio.CancelledError:
            pass  # Expected on cancel

        # Re-initialize display if needed
        global display
        display = DisplayManager()

        print("Calibration starting...")

        # Perform calibration (blocking)
        await calibration.start_calibration(device, disco_mode)

        # Update calibration data
        readings.calib_updated = calib
        device.current_state = SystemState.IDLE

        print("Calibration complete, resuming normal tasks.")
        display.display_screen_initialise()

        # Restart background tasks after calibration
        sensor_reading = asyncio.create_task(sensor_read_display_update(readings, device))
        watch_for_buttons = asyncio.create_task(watch_for_button_presses(device))
        check_battery = asyncio.create_task(check_battery_sensor(readings))
        monitor_ble = asyncio.create_task(monitor_ble_pin(device))
        monitor_ble_uart_task = asyncio.create_task(monitor_ble_uart(ble, device, sensor_manager))


asyncio.run(main())

# # Print calibration data to confirm
# calibration_dict = calib_updated.as_dict()
# print(calibration_dict)
#display.blank_screen()

