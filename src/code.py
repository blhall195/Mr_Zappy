import asyncio
import time

import keypad

from Angles import CustomAngleClass
from main_board import MrZappy

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


class SystemStates:
    def __init__(self):
        self.counter = 0
        self.fire_button_press = False
        self.paused = False
        self.angles = CustomAngleClass()


async def laser_firing(system_state: SystemStates, my_hardware: MrZappy):  # Don't forget the async!
    while True:
        if system_state.fire_button_press:
            if not system_state.paused:
                my_hardware.laser.set_laser(True)
                distance = my_hardware.laser.distance / 100
                my_hardware.screen.turn_off_screen() # Turns off the screen to prevent mag sensor interference

                azimuth, inclination, _ = system_state.angles.get_avg_value()

                my_hardware.screen.turn_on_screen() #Turns the screen back on after taking azimuth, inclination readings
                my_hardware.screen.distance_label.text = f"{distance}m"
                my_hardware.screen.update_angles(azimuth, inclination, True)

            else:
                my_hardware.screen.distance_label.text = ""

            system_state.fire_button_press = False
            system_state.paused = not system_state.paused

        await asyncio.sleep(0.1)

async def monitor_battery(my_hardware: MrZappy):
    while True:
        my_hardware.screen.update_battery_display()
        await asyncio.sleep(0.5)

async def monitor_buttons(button_states: SystemStates, hardware: MrZappy):
    """Monitor buttons that reverse direction and change animation speed.
    Assume buttons are active low.
    """
    with keypad.Keys(
        (
            hardware.fire_button,
            hardware.button_1,
            hardware.button_2,
            hardware.button_3,
            hardware.button_4,
        ),
        value_when_pressed=False,
        pull=True,
    ) as keys:
        while True:
            key_event = keys.events.get()
            if key_event is not None:
                print(key_event)
            if key_event and key_event.pressed:
                key_number = key_event.key_number
                if key_number == 0:
                    # Fire button
                    button_states.fire_button_press = not button_states.fire_button_press
                if key_number == 1:
                    #hardware.get_uncalibrated_angles()
                    print("pressed button 1")
                if key_number == 2:
                    print("pressed button 2")
                    button_states.counter = 4
                if key_number == 3:
                    print("pressed button 3")
                    button_states.counter = 4
                if key_number == 4:
                    print("pressed button 4")
                    button_states.counter = 4

            # Let another task run.
            await asyncio.sleep(0)


async def calc_angles(system_state: SystemStates, my_hardware: MrZappy):
    while True:
        # azimuth, inclination, roll = my_hardware.get_calibrated_angles()

        x, y, z = my_hardware.get_mag_readings()
        system_state.angles.mag_raw.add_values(x, y, z)

        x, y, z = my_hardware.get_grav_readings()
        system_state.angles.grav_raw.add_values(x, y, z)

        # system_state.angles.azimuth.add_value(azimuth)
        # system_state.angles.inclination.add_value(inclination)

        await asyncio.sleep(0)


async def display_updates(system_state: SystemStates, my_hardware: MrZappy):
    while True:
        if not system_state.paused:
            # azimuth, inclination, roll = my_hardware.get_calibrated_angles()
            azimuth, inclination, _ = system_state.angles.get_last_value()
            my_hardware.screen.update_angles(azimuth, inclination, False)

        await asyncio.sleep(0.2)


async def main():  # Don't forget the async!
    buttons = SystemStates()

    my_hardware = MrZappy()

    led_task = asyncio.create_task(laser_firing(buttons, my_hardware))
    buttons_task = asyncio.create_task(monitor_buttons(buttons, my_hardware))
    screen_task = asyncio.create_task(display_updates(buttons, my_hardware))
    monitor_battery_task = asyncio.create_task(monitor_battery(my_hardware))
    calc_angles_task = asyncio.create_task(calc_angles(buttons, my_hardware))

    await asyncio.gather(led_task, buttons_task, screen_task, calc_angles_task,monitor_battery_task)

    print("done")


asyncio.run(main())
