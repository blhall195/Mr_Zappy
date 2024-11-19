import time

from hardware_functions import MrZappy
from screen_hardware import ZappyScreen
import digitalio
import asyncio
import keypad

my_hardware = MrZappy()
my_screen = ZappyScreen()

import board

# perform calibration before screen loads
print(
    "Carry out the initial device calibration by pressing the fire button while rotating the device"
    " around as many different positions as possible."
)

def take_calibration_readings(button, mag_sensor, grav_sensor, mag_array: list, grav_array: list):
    previous_state = button.value
    iteration = 0
    while iteration < 20:
        current_state = not button.value
        if current_state and current_state != previous_state:
            time.sleep(0.01)
            iteration += 1
            # Collect readings from sensors
            mag_array.append(mag_sensor.magnetic)
            grav_array.append(grav_sensor.acceleration)
            print(iteration)
        previous_state = current_state
        time.sleep(0.1)


mag_array = []
grav_array = []

# take_calibration_readings(fire_button, mag_sensor, grav_sensor, mag_array, grav_array)
# mag_accuracy, grav_accuracy = calib.fit_ellipsoid(mag_array, grav_array)
# runs = calib.find_similar_shots(mag_array, grav_array)
# paired_data = [(mag_array[a:b], grav_array[a:b]) for a, b in runs]
# calib.fit_to_axis(paired_data)
# print(runs)

# print('mag_acc: ', mag_accuracy)
# print('grav_acc: ', grav_accuracy)
# paired_data = list(zip(mag_array, grav_array))
# calibration_dict = calib.as_dict()
# print(calibration_dict)



class RollingValue:
    def __init__(self, num_samples):
        self.values = []
        self.num_samples = num_samples
    def add_value(self, value : float):
        self.values.append(value)
        if len(self.values) > self.num_samples:
            self.values = self.values[1:]
    def get_rolling_average(self):
        if len(self.values)> 0:
            total = 0
            for value in self.values:
                total += value
            return total/len(self.values)
        else:
            return 0


class SystemStates:
    def __init__(self):
        self.counter = 0
        self.fire_button_press = False
        self.paused = False
        self.azimuth = RollingValue(10)
        self.inclination = RollingValue(10)


async def laser_firing(buttons : SystemStates):  # Don't forget the async!
    while True:
        if buttons.fire_button_press:

            if not buttons.paused:
                my_hardware.laser.set_laser(True)
                distance = my_hardware.laser.distance / 100
                my_screen.distance_label.text = f"{distance}m"
            else:
                my_screen.distance_label.text = ""

            buttons.fire_button_press = False
            buttons.paused = not buttons.paused

        await asyncio.sleep(0.1)

async def monitor_buttons(button_states : SystemStates, hardware : MrZappy):
    """Monitor buttons that reverse direction and change animation speed.
    Assume buttons are active low.
    """
    with keypad.Keys(
        (hardware.fire_button,
         hardware.button_1,
         hardware.button_2,
         hardware.button_3,
         hardware.button_4), value_when_pressed=False, pull=True
    ) as keys:
        while True:
            key_event = keys.events.get()
            if key_event is not None:
                print(key_event)
            if key_event and key_event.pressed:
                key_number = key_event.key_number
                if key_number == 0:
                    print("pressed the button")
                    button_states.fire_button_press = not button_states.fire_button_press
                if key_number == 1:
                    print("pressed the button")
                    button_states.counter =3
                if key_number == 2:
                    print("pressed the button")
                    button_states.counter =4

            # Let another task run.
            await asyncio.sleep(0)



async def calc_angles(button_states:SystemStates):

    while True:
        angles = my_hardware.get_calibrated_angles()
        button_states.azimuth.add_value(angles.azimuth)
        button_states.inclination.add_value(angles.inclination)

        await asyncio.sleep(0)


async def display_updates(button_states:SystemStates):
    while True:
        if not button_states.paused:

            my_screen.azimuth_label.text = f"{round(button_states.azimuth.get_rolling_average(), 1)}°"
            my_screen.inclination_label.text = f"{round(button_states.inclination.get_rolling_average(), 1)}°"

        await asyncio.sleep(0.2)


async def main():  # Don't forget the async!

    buttons = SystemStates()

    led_task = asyncio.create_task(laser_firing(buttons))
    buttons_task = asyncio.create_task(monitor_buttons(buttons, my_hardware))
    screen_task = asyncio.create_task(display_updates(buttons))
    calc_angles_task = asyncio.create_task(calc_angles(buttons))
    await asyncio.gather(led_task, buttons_task, screen_task,calc_angles_task)

    print("done")


asyncio.run(main())