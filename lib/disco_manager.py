import time
import board
import neopixel
import math
import asyncio

class DiscoMode:
    def __init__(self, pixel_pin=board.NEOPIXEL, num_pixels=1, brightness=1.0):
        self.pixel_pin = pixel_pin
        self.num_pixels = num_pixels
        self.brightness = max(0.0, min(1.0, brightness))
        self.pixels = neopixel.NeoPixel(
            self.pixel_pin,
            self.num_pixels,
            brightness=1.0,  # Hardware brightness stays at max
            auto_write=False
        )
        self.is_active = False
        self.task = None
        self.loop = asyncio.get_event_loop()

    def _apply_brightness(self, r, g, b):
        """Apply software brightness scaling."""
        return (
            int(r * self.brightness),
            int(g * self.brightness),
            int(b * self.brightness)
        )

    def set_green(self):
        """Set LED to green."""
        self._stop_disco()
        self.pixels.fill(self._apply_brightness(0, 255, 0))
        self.pixels.show()

    def set_red(self):
        """Set LED to red."""
        self._stop_disco()
        self.pixels.fill(self._apply_brightness(255, 0, 0))
        self.pixels.show()

    def turn_off(self):
        """Turn off the LED."""
        self._stop_disco()
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def _stop_disco(self):
        """Stop the disco effect if running."""
        if self.is_active:
            self.is_active = False
            if self.task is not None:
                self.loop.run_until_complete(self.task)

    def start_disco(self):
        """Start the disco (rainbow) effect."""
        if not self.is_active:
            self.is_active = True
            print("Disco Mode ON")
            self.task = self.loop.create_task(self._run_disco_effect())

    async def _run_disco_effect(self):
        """Run the rainbow effect."""
        while self.is_active:
            for j in range(0, 360, 10):
                if not self.is_active:
                    break
                hue = j / 360.0
                color = self.hsv_to_rgb(hue, 1.0, 1.0)
                color = self._apply_brightness(*color)
                self.pixels.fill(color)
                self.pixels.show()
                await asyncio.sleep(0.1)
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB."""
        i = int(h * 6)
        f = h * 6 - i
        p = v * (1 - s)
        q = v * (1 - f * s)
        t = v * (1 - (1 - f) * s)
        i = i % 6
        if i == 0: return int(v * 255), int(t * 255), int(p * 255)
        if i == 1: return int(q * 255), int(v * 255), int(p * 255)
        if i == 2: return int(p * 255), int(v * 255), int(t * 255)
        if i == 3: return int(p * 255), int(q * 255), int(v * 255)
        if i == 4: return int(t * 255), int(p * 255), int(v * 255)
        if i == 5: return int(v * 255), int(p * 255), int(q * 255)
