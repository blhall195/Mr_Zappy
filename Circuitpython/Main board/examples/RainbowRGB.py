import time
import board
import neopixel
import math

# Define which pin the NeoPixel is connected to
pixel_pin = board.NEOPIXEL  # use `board.D6` for external NeoPixel

# Define the number of NeoPixels (1 if built-in single pixel)
num_pixels = 1

# Create the NeoPixel object
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=1, auto_write=False)

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

# Main loop for smooth rainbow effect
try:
    while True:
        for j in range(360):  # Go through 360 hue values
            hue = j / 360.0  # Normalize to 0-1
            color = hsv_to_rgb(hue, 1.0, 1.0)  # Full saturation and brightness
            pixels.fill(color)
            pixels.show()
            time.sleep(0.01)  # Small delay for smooth transition

except KeyboardInterrupt:
    # Turn off NeoPixel when exiting
    pixels.fill((0, 0, 0))
    pixels.show()

