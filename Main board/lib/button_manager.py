import board
import digitalio
from adafruit_debouncer import Debouncer

class ButtonManager:
    def __init__(self):
        """Initializes the default set of buttons with debouncing."""
        pin_map = {
            "Button 1": board.A3,
            "Button 2": board.A4,
            "Button 3": board.A0,
            "Button 4": board.D25,
            "Fire Button": board.D4,
        }

        self.buttons = {}

        for name, pin in pin_map.items():
            io = digitalio.DigitalInOut(pin)
            io.direction = digitalio.Direction.INPUT
            io.pull = digitalio.Pull.UP
            debounced = Debouncer(io, interval=0.01)
            self.buttons[name] = debounced

    def update(self):
        """Call this once per loop to update all debouncer states."""
        for button in self.buttons.values():
            button.update()

    def is_pressed(self, name):
        """Returns True if the button is currently pressed."""
        return self.buttons[name].value == False  # Active low

    def was_pressed(self, name):
        """Returns True if the button was just pressed (fell)."""
        return self.buttons[name].fell

    def was_released(self, name):
        """Returns True if the button was just released (rose)."""
        return self.buttons[name].rose

    def all_buttons(self):
        """Returns a list of all button names."""
        return list(self.buttons.keys())

