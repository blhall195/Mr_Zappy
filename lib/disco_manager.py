import time
import board
import neopixel
import math
import asyncio

class DiscoMode:
    def __init__(self, pixel_pin=board.NEOPIXEL, num_pixels=1, brightness=1.0):
        self.pixel_pin = pixel_pin
        self.num_pixels = num_pixels
        self.pixels = neopixel.NeoPixel(self.pixel_pin, self.num_pixels, brightness=brightness, auto_write=False)
        self.is_active = False
        self.task = None  # To hold the background task
        self.loop = asyncio.get_event_loop()

    def hsv_to_rgb(self, h, s, v):
        """Helper function to convert HSV to RGB."""
        i = int(h * 6)
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

    def on(self):
        """Start the disco mode synchronously."""
        if not self.is_active:
            self.is_active = True
            print("Disco Mode ON")
            self.task = self.loop.create_task(self._run_disco_effect())

    def off(self):
        """Stop the disco mode synchronously."""
        if self.is_active:
            self.is_active = False
            # Wait for the background task to finish (block until done)
            if self.task is not None:
                self.loop.run_until_complete(self.task)
            self.pixels.fill((0, 0, 0))
            self.pixels.show()
            print("Disco Mode OFF")

    async def _run_disco_effect(self):
        """Run the rainbow effect in a smooth loop asynchronously."""
        while self.is_active:
            for j in range(0, 360, 10):
                if not self.is_active:
                    break
                hue = j / 360.0  # Normalize to 0-1
                color = self.hsv_to_rgb(hue, 1.0, 1.0)
                self.pixels.fill(color)
                self.pixels.show()
                await asyncio.sleep(0.1)
