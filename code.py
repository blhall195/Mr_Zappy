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


# Monitor states of things between async functions
class States:
    def __init__(self):
        self.azimuth = 0
        self.inclination = 0
        self.roll = 0
        self.distance = 0
        self.calib_updated = None


device_states = States()


def update_az_inc(states):
    try:
        states.azimuth, states.inclination, states.roll = states.calib_updated.get_angles(
            sensor_manager.get_mag(), sensor_manager.get_grav()
        )
        print(f"({states.azimuth})")

    except Exception as e:
        print(e)


async def sensor_read_display_update(states):
    # Use a new variable name for the updated calib data
    calibration_dict = {'mag': {'axes': '-X-Y-Z', 'transform': [[0.0231683, -4.50966e-05, -0.000208465], [-4.50968e-05, 0.0233006, -2.46289e-05], [-0.000208464, -2.46296e-05, 0.0231333]], 'centre': [0.407859, -1.9058, 2.11295], 'rbfs': [], 'field_avg': None, 'field_std': None}, 'dip_avg': None, 'grav': {'axes': '-Y-X+Z', 'transform': [[0.101454, 0.00155312, -0.000734401], [0.00155312, 0.101232, 0.00149594], [-0.000734397, 0.00149594, 0.0987455]], 'centre': [0.364566, -0.0656354, 0.193454], 'rbfs': [], 'field_avg': None, 'field_std': None}}

    # Update calib from the dictionary
    states.calib_updated = calib.from_dict(calibration_dict)
    while True:
        update_az_inc(states)
        display.update_sensor_readings(0, states.azimuth, states.inclination)
        await asyncio.sleep(.5)


async def watch_for_button_presses(states):
    while True:
        button_manager.update()

        # If "Fire Button" is pressed, perform a special action
        if button_manager.was_pressed("Button 1"):
            print("Fire Button pressed!")
            states.distance = sensor_manager.get_distance()
            update_az_inc(states)
            print(states.distance)
            ble.send_message(states.azimuth,states.inclination,states.distance)

        await asyncio.sleep(0)


async def main():

    # ----Initialize the PerformCalibration with both sensor and button managers
    #calibration = PerformCalibration(sensor_manager, button_manager, calib)
    #await calibration.start_calibration()

    # Display initialization
    display.display_screen_initialise()
    display.update_battery(sensor_manager.get_bat())

    sensor_reading = asyncio.create_task(sensor_read_display_update(device_states))
    watch_for_buttons = asyncio.create_task(watch_for_button_presses(device_states))
    # # Print calibration data to confirm
    # calibration_dict = calib_updated.as_dict()
    # print(calibration_dict)



    await asyncio.gather(sensor_reading,watch_for_buttons)

# Run the main async event loop
asyncio.run(main())
