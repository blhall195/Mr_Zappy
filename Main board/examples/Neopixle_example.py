import time
import board
import neopixel

# Define which pin the NeoPixel is connected to
# For built-in NeoPixel on Feather, use `board.NEOPIXEL`
# If you're using an external NeoPixel, connect it to a pin like `D6`
pixel_pin = board.NEOPIXEL  # or use `board.D6` for external

# Define the number of NeoPixels
num_pixels = 1

# Create the NeoPixel object
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.3, auto_write=False)

# Define a function to set the NeoPixel color
def set_color(color):
    pixels.fill(color)
    pixels.show()

# Main loop to change colors
try:
    while True:
        set_color((255, 0, 0))  # Red
        time.sleep(1)

        set_color((0, 255, 0))  # Green
        time.sleep(1)

        set_color((0, 0, 255))  # Blue
        time.sleep(1)

        set_color((255, 255, 0))  # Yellow
        time.sleep(1)

        set_color((0, 255, 255))  # Cyan
        time.sleep(1)

        set_color((255, 0, 255))  # Magenta
        time.sleep(1)

        set_color((255, 255, 255))  # White
        time.sleep(1)

        set_color((0, 0, 0))  # Off
        time.sleep(1)

except KeyboardInterrupt:
    # Turn off NeoPixel when exiting
    set_color((0, 0, 0))
