"""Coverage bar display for calibration - loaded only when needed."""
import math


class CoverageBar:
    """
    Lightweight coverage bar with point counter for calibration.
    8 columns × 4 rows = 32 zones, shown as a small bar.
    """
    COLS = 8
    ROWS = 4

    def __init__(self, display, font):
        import displayio
        self.display = display
        self.font = font
        self.zones = [[False] * self.COLS for _ in range(self.ROWS)]
        self.point_count = 0
        # Small bitmap: 112 wide × 16 tall
        self.bitmap = displayio.Bitmap(112, 16, 2)
        self.palette = displayio.Palette(2)
        self.palette[0] = 0x000000
        self.palette[1] = 0xFFFFFF
        self.tilegrid = displayio.TileGrid(self.bitmap, pixel_shader=self.palette, x=8, y=80)

    def show(self):
        """Create calibration screen with counter and bar."""
        import displayio
        from adafruit_display_text import bitmap_label as label

        # Draw bar outline
        for x in range(112):
            self.bitmap[x, 0] = 1
            self.bitmap[x, 15] = 1
        for y in range(16):
            self.bitmap[0, y] = 1
            self.bitmap[111, y] = 1
        # Draw column dividers
        for col in range(1, self.COLS):
            x = col * 14
            for y in range(16):
                self.bitmap[x, y] = 1

        # Create labels
        self.title1_label = label.Label(self.font, text="Ellipsoid", scale=1, x=32, y=6)
        self.title2_label = label.Label(self.font, text="Calibration", scale=1, x=24, y=18)
        self.count_label = label.Label(self.font, text="0/56", scale=3, x=30, y=50)

        # Create display group
        self.group = displayio.Group()
        self.group.append(self.title1_label)
        self.group.append(self.title2_label)
        self.group.append(self.count_label)
        self.group.append(self.tilegrid)
        self.display.root_group = self.group

    def add_point(self, x, y, z):
        """Add point and update bar and counter."""
        self.point_count += 1
        self.count_label.text = f"{self.point_count}/56"

        mag = math.sqrt(x*x + y*y + z*z)
        if mag < 0.001:
            return
        nz = z / mag
        elevation = math.degrees(math.asin(max(-1.0, min(1.0, nz))))
        azimuth = math.degrees(math.atan2(y, x)) % 360
        row = min(self.ROWS - 1, max(0, int((elevation + 90) / 45)))
        col = min(self.COLS - 1, int(azimuth / 45))
        if not self.zones[row][col]:
            self.zones[row][col] = True
            self._update_col(col)

    def _update_col(self, col):
        """Update a column's fill based on how many rows are filled."""
        count = sum(1 for r in range(self.ROWS) if self.zones[r][col])
        start_x = col * 14 + 2
        for py in range(12):
            fill = 1 if py < (count * 3) else 0
            for px in range(10):
                self.bitmap[start_x + px, 13 - py] = fill
