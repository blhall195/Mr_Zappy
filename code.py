import board
import digitalio
import time

# Define pin 13
pin_to_test = board.D13

# Set up pin 13 as an input with a pull-up resistor
button = digitalio.DigitalInOut(pin_to_test)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

# Main loop to check the button state
while True:
    # Check if the button is grounded
    if not button.value:
        status = "Grounded"
    else:
        status = "Not grounded"
    
    # Print the status on a single line, overwriting the previous line
    print(f"Pin 13: {status}", end='\r')
    time.sleep(0.5)
