#delete the 3 apostrophies above if you want to enable the boot out code to allow
#the microcontroller to write to the devices memory when not plugged into a PC

import board
import digitalio
import storage
import usb_cdc
import usb_midi
import usb_hid

# Set up A4 as input with internal pull-up
trigger_pin = digitalio.DigitalInOut(board.A3)
trigger_pin.direction = digitalio.Direction.INPUT
trigger_pin.pull = digitalio.Pull.UP

if trigger_pin.value:
    # A4 is HIGH (default) — read-only flash, normal USB
    storage.remount("/", readonly=False)
    usb_midi.disable()
    usb_hid.disable()
else:
    # A4 is pulled LOW (e.g. jumper to GND) — disable write access and disable USB
    storage.remount("/", readonly=True)
    usb_midi.disable()
    usb_hid.disable()
