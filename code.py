import asyncio
import board
import digitalio
import time
from sensor_manager import SensorManager
from button_manager import ButtonManager
from display_manager import DisplayManager
from calibration_manager import PerformCalibration
from mag_cal.calibration import Calibration
from ble_manager import BleManager

# Create instances
calib = Calibration(mag_axes="-X-Y-Z", grav_axes="-Y-X+Z")
sensor_manager = SensorManager()
button_manager = ButtonManager()
display = DisplayManager()
ble = BleManager()

print("Loading modules")
time.sleep(0.02)
print("...Display Settings")
time.sleep(0.01)
print("...Calibration")
print("...Bluetooth")
time.sleep(0.01)
print("...Sensors")
time.sleep(0.1)
print("...Config.")
time.sleep(0.01)
print("Starting Device")
time.sleep(0.1)

# Temp calibration dict for development/testing
calibration_dict = {'mag': {'axes': '-X-Y-Z', 'transform': [[0.0231683, -4.50966e-05, -0.000208465], [-4.50968e-05, 0.0233006, -2.46289e-05], [-0.000208464, -2.46296e-05, 0.0231333]], 'centre': [0.407859, -1.9058, 2.11295], 'rbfs': [], 'field_avg': None, 'field_std': None}, 'dip_avg': None, 'grav': {'axes': '-Y-X+Z', 'transform': [[0.101454, 0.00155312, -0.000734401], [0.00155312, 0.101232, 0.00149594], [-0.000734397, 0.00149594, 0.0987455]], 'centre': [0.364566, -0.0656354, 0.193454], 'rbfs': [], 'field_avg': None, 'field_std': None}}

# A class for storing device readings
class Readings:
    def __init__(self):
        self.azimuth = 0
        self.inclination = 0
        self.roll = 0
        self.distance = 0
        self.calib_updated = None

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
    while True:
        if device.current_state == SystemState.IDLE:
            update_readings(readings)
            display.update_sensor_readings(0, readings.azimuth, readings.inclination)

        elif device.current_state == SystemState.TAKING_MEASURMENT:
            # Only take reading once when entering the state
            if not device.measurement_taken:
                update_readings(readings)
                readings.distance = sensor_manager.get_distance()/100
                display.update_distance(readings.distance)
                print(readings.distance)
                ble.send_message(readings.azimuth, readings.inclination, readings.distance)
                device.measurement_taken = True

            # While TAKING_MEASURMENT, everything else is paused (no updates)

        await asyncio.sleep(0.5)


async def watch_for_button_presses(readings, device):
    while True:
        button_manager.update()

        if button_manager.was_pressed("Button 1"):
            print("Fire Button pressed!")

            if device.current_state == SystemState.IDLE:
                device.current_state = SystemState.TAKING_MEASURMENT
                device.measurement_taken = False  # reset flag
            elif device.current_state == SystemState.TAKING_MEASURMENT:
                device.current_state = SystemState.IDLE
                sensor_manager.set_laser(True)

        await asyncio.sleep(0.01)


def update_readings(readings):
    try:
        readings.azimuth, readings.inclination, readings.roll = readings.calib_updated.get_angles(
            sensor_manager.get_mag(), sensor_manager.get_grav()
        )
        print(f"({readings.azimuth})")

    except Exception as e:
        print(e)

async def main():

    # ----Initialize the PerformCalibration with both sensor and button managers
    #calibration = PerformCalibration(sensor_manager, button_manager, calib)
    #await calibration.start_calibration()

    # Display initialization
    display.display_screen_initialise()
    display.update_battery(sensor_manager.get_bat())

    sensor_reading = asyncio.create_task(sensor_read_display_update(readings, device))
    watch_for_buttons = asyncio.create_task(watch_for_button_presses(readings, device))
    # # Print calibration data to confirm
    # calibration_dict = calib_updated.as_dict()
    # print(calibration_dict)



    await asyncio.gather(sensor_reading,watch_for_buttons)

# Run the main async event loop
asyncio.run(main())
