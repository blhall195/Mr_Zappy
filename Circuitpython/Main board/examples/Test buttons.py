import board
import digitalio
import time

# Set up buttons as inputs
button_pins = {
    "Button 1": digitalio.DigitalInOut(board.D24),
    "Button 2": digitalio.DigitalInOut(board.D23),
    "Button 3": digitalio.DigitalInOut(board.D19),
    "Button 4": digitalio.DigitalInOut(board.D25),
    "Fire Button": digitalio.DigitalInOut(board.D13),
}

# Initialize buttons and store their previous states
for button in button_pins.values():
    button.direction = digitalio.Direction.INPUT
    button.pull = digitalio.Pull.UP  # Use pull-up resistor

# Create a dictionary to keep track of button states
button_states = {name: True for name in button_pins.keys()}  # Start with all buttons released

while True:
    # Check the state of each button
    for button_name, button in button_pins.items():
        current_state = not button.value  # Active low: pressed = True, not pressed = False
        
        # Check if the state has changed
        if current_state != button_states[button_name]:
            button_states[button_name] = current_state  # Update the state
            
            if current_state:  # If the button is pressed
                print(f"{button_name} is ON (pressed)")
            else:  # If the button is released
                print(f"{button_name} is OFF (not pressed)")
    
    time.sleep(0.1)  # Delay to avoid flooding the output

