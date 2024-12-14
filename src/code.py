import asyncio
import time
import board
import neopixel
import keypad

from Angles import CustomAngleClass
from main_board import MrZappy
#from mag_cal.calibration import Calibration

pixel_pin = board.NEOPIXEL  # use `board.D6` for external NeoPixel
num_pixels = 1 # Define the number of NeoPixels (1 if built-in single pixel)
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=1, auto_write=False)# Create the NeoPixel object

def take_calibration_readings(button, mag_sensor, grav_sensor, mag_array: list, grav_array: list, my_hardware: MrZappy):
    my_hardware.screen.release_display()
    my_hardware.laser.set_laser(False)
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

# Helper function to convert HSV to RGB
def hsv_to_rgb(h, s, v):
    i = int(h * 6)  # assuming h is in [0, 1), and s, v are in [0, 1]
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    i = i % 6
    if i == 0:
        return int(v * 255), int(t * 255), int(p * 255)
    if i == 1:
        return int(q * 255), int(v * 255), int(p * 255)
    if i == 2:
        return int(p * 255), int(v * 255), int(t * 255)
    if i == 3:
        return int(p * 255), int(q * 255), int(v * 255)
    if i == 4:
        return int(t * 255), int(p * 255), int(v * 255)
    if i == 5:
        return int(v * 255), int(p * 255), int(q * 255)


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
        self.button_1_press = False
        self.button_2_press = False
        self.button_3_press = False
        self.paused = False
        self.calibration_active = False
        self.rainbow_active = False
        self.angles = CustomAngleClass()


# Async rainbow effect function
async def rainbow_effect(system_state: SystemStates,pixels):
    try:
        while True:
            for j in range(0, 360, 10):  # Go through 360 hue values
                hue = j / 360.0  # Normalize to 0-1
                color = hsv_to_rgb(hue, 1.0, 1.0)  # Full saturation and brightness
                pixels.fill(color)
                pixels.show()
                await asyncio.sleep(0.01)  # Non-blocking delay for smooth transition
    except asyncio.CancelledError:
        # Handle task cancellation gracefully
        pixels.fill((0, 0, 0))
        pixels.show()


# Async function to monitor the button and toggle the rainbow effect
async def monitor_disco_button(system_state: SystemStates, pixels):
    rainbow_task = None
    while True:
        if system_state.button_3_press:  # Check if button 2 is pressed
            system_state.button_3_press = False  # Reset button press state
            system_state.rainbow_active = not system_state.rainbow_active  # Toggle the rainbow state
            if system_state.rainbow_active:
                print("Starting rainbow effect...")
                if rainbow_task:  # Ensure any previous task is canceled
                    rainbow_task.cancel()
                    try:
                        await rainbow_task
                    except asyncio.CancelledError:
                        pass
                rainbow_task = asyncio.create_task(rainbow_effect(system_state, pixels))
            else:
                print("Stopping rainbow effect...")
                if rainbow_task:
                    rainbow_task.cancel()
                    try:
                        await rainbow_task
                    except asyncio.CancelledError:
                        pass
        await asyncio.sleep(0.05)  # Debounce delay


async def preform_calibration(system_state: SystemStates, my_hardware: MrZappy):  # Don't forget the async!
    while True:
        if system_state.button_1_press:
            # Reset the button state to avoid re-triggering immediately
            system_state.button_1_press = False  # Reset button press state
            my_hardware.laser.set_laser(False)
            system_state.calibration_active = True
            my_hardware.screen.release_display()
            mag_array = []
            grav_array = []
            print("Carry out the initial device calibration by pressing the fire button while rotating the device"
                  " around as many different positions as possible.")
            iteration = 0

            while iteration < 20:
                # Check if button 1 is pressed again to exit the calibration loop
                if system_state.button_1_press:
                    print("Calibration process cancelled.")
                    system_state.calibration_active = False  # Set calibration state to inactive
                    system_state.button_1_press = False  # Reset the button state
                    my_hardware.screen.initialise_screen()
                    break  # Exit the function

                if system_state.fire_button_press:  # Check for a transition from not-pressed to pressed
                    iteration += 1
                    # Collect readings from sensors
                    mag_array.append(my_hardware.get_mag_readings())
                    grav_array.append(my_hardware.get_grav_readings())
                    print(iteration)
                    system_state.fire_button_press = False
                await asyncio.sleep(0.1)

            # Calibration process is complete
            # mag_accuracy, grav_accuracy = calib.fit_ellipsoid(mag_array, grav_array)
            # print(mag_accuracy, grav_accuracy)
            system_state.calibration_active = False

        await asyncio.sleep(0.1)



async def monitor_battery(my_hardware: MrZappy):
    while True:
        my_hardware.screen.update_battery_display()
        await asyncio.sleep(30)


async def laser_firing(system_state: SystemStates, my_hardware: MrZappy):  # Don't forget the async!
    while True:
        # Skip laser firing if calibration is active
        if system_state.calibration_active:
            await asyncio.sleep(0.1)
            continue
        if system_state.fire_button_press:
            if not system_state.paused:

                distance = my_hardware.laser.distance / 100
                my_hardware.screen.turn_off_screen() # Turns off the screen to prevent mag sensor interference

                azimuth, inclination, _ = system_state.angles.get_avg_value()

                my_hardware.screen.turn_on_screen() #Turns the screen back on after taking azimuth, inclination readings
                my_hardware.screen.distance_label.text = f"{distance}m"
                my_hardware.screen.update_angles(azimuth, inclination, True)
                #my_hardware.laser.set_laser(False)


            else:
                my_hardware.screen.distance_label.text = ""
                my_hardware.laser.set_laser(True)
            system_state.fire_button_press = False
            system_state.paused = not system_state.paused

        await asyncio.sleep(0.1)


async def monitor_buttons(button_states: SystemStates, hardware: MrZappy):
    """Monitor buttons for single press and long press.
    Assume buttons are active low.
    """
    long_press_threshold = 2  # Time in seconds for a long press

    fire_button_last_press_time = None
    fire_button_pressed = False
    fire_button_long_pressed = False

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
                time.sleep(0.001)
            if key_event and key_event.pressed:
                key_number = key_event.key_number
                current_time = time.monotonic()

                if key_number == 0:  # Fire button
                    fire_button_last_press_time = current_time
                    fire_button_pressed = True
                    fire_button_long_pressed = False

                elif key_number == 1:
                    # Button 1
                    print("Pressed button 1")
                    button_states.counter = 1
                    button_states.button_1_press = not button_states.button_1_press
                elif key_number == 2:
                    print("Pressed button 2")
                    button_states.counter = 2
                    button_states.button_2_press = not button_states.button_2_press
                elif key_number == 3:
                    print("Pressed button 3")
                    button_states.counter = 3
                    button_states.button_3_press = not button_states.button_3_press

            # Handle fire button states
            if fire_button_pressed:
                elapsed_time = time.monotonic() - fire_button_last_press_time

                if elapsed_time >= long_press_threshold and not fire_button_long_pressed:
                    # Long press detected
                    fire_button_long_pressed = True
                    fire_button_pressed = False  # Reset pressed state
                    button_states.fire_button_long_press = True
                    button_states.fire_button_press = False  # Clear single press
                    print("Fire button long press detected!")

                elif key_event and key_event.released:
                    # Button released
                    if elapsed_time < long_press_threshold:
                        # Single press detected
                        button_states.fire_button_press = True
                        button_states.fire_button_long_press = False
                        print("Fire button single press detected!")

                    fire_button_pressed = False

            # Let another task run
            await asyncio.sleep(0.1)


async def calc_angles(system_state: SystemStates, my_hardware: MrZappy):
    while True:
        # azimuth, inclination, roll = my_hardware.get_calibrated_angles()

        x, y, z = my_hardware.get_mag_readings()
        system_state.angles.mag_raw.add_values(x, y, z)#

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
    preform_calibration_task = asyncio.create_task(preform_calibration(buttons, my_hardware))
    monitor_disco_button_task = asyncio.create_task(monitor_disco_button(buttons, pixels))


    await asyncio.gather(led_task, buttons_task, screen_task, calc_angles_task,monitor_battery_task,preform_calibration_task,monitor_disco_button_task)

    print("done")


asyncio.run(main())
