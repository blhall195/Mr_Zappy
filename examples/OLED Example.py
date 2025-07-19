# SPDX-FileCopyrightText: 2021 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
"""
Based on example by Mark Roberts (mdroberts1243).

This example writes text to the display, and draws a series of squares and a rectangle.
"""

import board
import displayio
import terminalio
from adafruit_display_text import bitmap_label as label
from adafruit_displayio_sh1107 import SH1107, DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297
from adafruit_bitmap_font import bitmap_font
import random

font = bitmap_font.load_font("lib/fonts/terminal.bdf")

displayio.release_displays()
i2c = board.I2C()
display_bus = displayio.I2CDisplay(i2c, device_address=0x3d)

# Width, height and rotation for Monochrome 1.12" 128x128 OLED
WIDTH = 128
HEIGHT = 128
ROTATION = 0
BORDER = 2

display = SH1107(
    display_bus,
    width=WIDTH,
    height=HEIGHT,
    display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297,
    rotation=ROTATION,
)

color_bitmap = displayio.Bitmap(WIDTH, HEIGHT, 1)
white = displayio.Palette(1)
white[0] = 0xFFFFFF  # White
black = displayio.Palette(1)
black[0] = 0x000000  # Black


FONTSCALE = 3
while True:
    s= "1.13m"
    a = label.Label(font, text=s, color=0xFFFFFF,x=0,y=44,scale=FONTSCALE)
    b = random.random()
    b = b*150
    b = round(b, 1)
    b = str(b)
    b = f"{b}°"
    b = label.Label(font, text=b, color=0xFFFFFF,x=0,y=78,scale=FONTSCALE)
    c = random.random()
    c=c*150
    c = round(c, 1)
    c = str(c)
    c = f"{c}°"
    c = label.Label(font, text=c, color=0xFFFFFF,x=0,y=112,scale=FONTSCALE)

    #code for displaying battery
    small_bitmap_1 = displayio.Bitmap(32, 15, 1)
    small_square_1 = displayio.TileGrid(small_bitmap_1, pixel_shader=white, x=90, y=0)
    small_bitmap_2 = displayio.Bitmap(30, 13, 1)
    small_square_2 = displayio.TileGrid(small_bitmap_2, pixel_shader=black, x=91, y=1)
    small_bitmap_3 = displayio.Bitmap(3, 6, 1)
    small_square_3 = displayio.TileGrid(small_bitmap_3, pixel_shader=white, x=122, y=4)
    small_bitmap_4 = displayio.Bitmap(15, 13, 1)
    small_square_4 = displayio.TileGrid(small_bitmap_4, pixel_shader=white, x=91, y=1)
    battery_percentage = 51
    battery_percentage = str(battery_percentage)
    battery_percentage = f"{battery_percentage}%"
    battery_percentage = label.Label(font, text=battery_percentage, color=0xFFFFFF,x=50,y=6,scale=2)

    splash = displayio.Group()
    display.root_group = splash  # Set the splash group as the root group
    splash.append(a)
    splash.append(c)
    splash.append(b)
    splash.append(small_square_1)
    splash.append(small_square_2)
    splash.append(small_square_3)
    splash.append(small_square_4)
    splash.append(battery_percentage)





