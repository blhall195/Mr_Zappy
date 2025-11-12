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

        # Brightness handling
        self.base_brightness = 0.3
        self.wild_brightness = 1.0
        self.brightness = self.base_brightness
        self._last_shake_time = 0
        self._shake_timeout = 5  # seconds to return to dim after shake

        self.pixels = neopixel.NeoPixel(
            self.pixel_pin,
            self.num_pixels,
            brightness=1.0,  # hardware brightness stays at max
            auto_write=False
        )

        # State flags
        self.is_active = False
        self.pulse_active = False
        self.disco_active = False  # <-- Added for compatibility with external references

        self.task = None
        self.loop = asyncio.get_event_loop()

    def _apply_brightness(self, r, g, b):
        return (
            int(r * self.brightness),
            int(g * self.brightness),
            int(b * self.brightness)
        )

    # ----- Static color setters -----
    def set_red(self):
        self._stop_all()
        self.pixels.fill(self._apply_brightness(255, 0, 0))
        self.pixels.show()

    def set_green(self):
        self._stop_all()
        self.pixels.fill(self._apply_brightness(0, 255, 0))
        self.pixels.show()

    def set_blue(self):
        self._stop_all()
        self.pixels.fill(self._apply_brightness(0, 0, 255))
        self.pixels.show()

    def set_white(self):
        self._stop_all()
        self.pixels.fill(self._apply_brightness(255, 255, 255))
        self.pixels.show()

    def turn_off(self):
        self._stop_all()
        self.pixels.fill((0, 0, 0))
        self.pixels.show()

    def _stop_all(self):
        self.is_active = False
        self.pulse_active = False
        self.disco_active = False
        if self.task:
            self.task.cancel()
            self.task = None

    # ----- Disco Mode -----
    def start_disco(self):
        if not self.is_active:
            self._stop_all()
            self.is_active = True
            self.disco_active = True
            self.brightness = self.base_brightness
            self.wild_latched = False
            self._last_shake_time = 0
            print("Disco Mode ON (starting dim)")
            self.task = self.loop.create_task(self._run_disco_effect())

    # ----- Purple Pulse -----
    def set_purple(self):
        self._stop_all()
        self.is_active = True
        self.pulse_active = True
        self.task = self.loop.create_task(self._run_purple_pulse())

    def hsv_to_rgb(self, h, s, v):
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

    async def _run_disco_effect(self):
        base_sleep = 0.2
        wild_sleep = 0.1
        color_step_base = 20
        j = 0
        self.wild_latched = False

        while self.is_active and self.disco_active:
            grav = self.sensor_manager.get_grav()
            gx, gy, gz = grav if grav else (0, 0, 0)
            now = time.monotonic()

            # Detect shake for wild mode
            if abs(gx) > 11 or abs(gy) > 11 or abs(gz) > 11:
                self.wild_latched = True
                self._last_shake_time = now
                self.brightness = self.wild_brightness
                print("🔥 Wild mode latched ON! Brightness increased.")

            # Return to dim after timeout
            if self.wild_latched and now - self._last_shake_time > self._shake_timeout:
                self.brightness = self.base_brightness
                print("🌙 Calm mode. Brightness reduced.")

            is_wild = self.brightness == self.wild_brightness
            sleep_time = wild_sleep if is_wild else base_sleep

            # Choose color
            hue = random.random() if is_wild else (j % 360) / 360.0
            j += color_step_base

            color = self.hsv_to_rgb(hue, 1.0, 1.0)
            color = self._apply_brightness(*color)
            self.pixels.fill(color)
            self.pixels.show()

            await asyncio.sleep(sleep_time)

        self.turn_off()

    async def _run_purple_pulse(self):
        base_color = (255, 0, 255)
        t = 0
        while self.pulse_active:
            pulse_brightness = 0.55 + 0.45 * math.sin(t)
            t += 0.3
            r = int(base_color[0] * pulse_brightness)
            g = int(base_color[1] * pulse_brightness)
            b = int(base_color[2] * pulse_brightness)
            self.pixels.fill((r, g, b))
            self.pixels.show()
            await asyncio.sleep(0.03)
        self.turn_off()
