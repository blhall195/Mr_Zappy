import time
import board
import neopixel
import math
import asyncio
import random

class DiscoMode:
    def __init__(self, sensor_manager, pixel_pin=board.NEOPIXEL, num_pixels=1, brightness=1.0):
        self.sensor_manager = sensor_manager
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
        if self.is_active:
            self.is_active = False
            if self.task is not None:
                self.task.cancel()
                self.task = None

    def start_disco(self):
        """Start the disco (rainbow) effect."""
        if not self.is_active:
            self.is_active = True
            print("Disco Mode ON")
            self.task = self.loop.create_task(self._run_disco_effect())

    async def _run_disco_effect(self):
        """Run the rainbow effect with latched wild mode based on gravity."""
        base_sleep = 0.2
        wild_sleep = 0.1
        color_step_base = 20

        j = 0
        self.wild_latched = False  # latch flag

        while self.is_active:
            # Continuously sample gravity
            grav = self.sensor_manager.get_grav()
            gx, gy, gz = grav if grav else (0, 0, 0)

            # Trigger wild mode once if any axis exceeds ~1.1g
            if not self.wild_latched and (abs(gx) > 11 or abs(gy) > 11 or abs(gz) > 11):
                self.wild_latched = True
                print("🔥 Wild mode latched ON!")

            is_wild = self.wild_latched
            sleep_time = wild_sleep if is_wild else base_sleep

            # Choose color
            if is_wild:
                hue = random.random()  # random hue between 0.0–1.0
            else:
                hue = (j % 360) / 360.0
                j += color_step_base

            # Apply color
            color = self.hsv_to_rgb(hue, 1.0, 1.0)
            color = self._apply_brightness(*color)
            self.pixels.fill(color)
            self.pixels.show()

            #print(f"Gravity: ({gx:.2f}, {gy:.2f}, {gz:.2f}) | Wild Mode: {is_wild}")

            await asyncio.sleep(sleep_time)

        # Turn off LED when done
        self.pixels.fill((0, 0, 0))
        self.pixels.show()
        self.wild_latched = False  # Reset for next run

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
