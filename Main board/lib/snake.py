import time
import asyncio
import random
import gc
import displayio
import microcontroller


# Packed 5x7 pixel font — 7 bytes per char, stored in flash (not heap)
# Index string maps character to its position in the packed data
_FONT_CHARS = "GAMEOVRSC:0123456789 "
_FONT_DATA = (
    b'\x0e\x11\x01\x1d\x11\x11\x0e'  # G
    b'\x0e\x11\x11\x1f\x11\x11\x11'  # A
    b'\x11\x1b\x15\x11\x11\x11\x11'  # M
    b'\x1f\x01\x01\x0f\x01\x01\x1f'  # E
    b'\x0e\x11\x11\x11\x11\x11\x0e'  # O
    b'\x11\x11\x11\x11\x0a\x0a\x04'  # V
    b'\x0f\x11\x11\x0f\x05\x09\x11'  # R
    b'\x0e\x11\x01\x0e\x10\x11\x0e'  # S
    b'\x0e\x11\x01\x01\x01\x11\x0e'  # C
    b'\x00\x04\x04\x00\x04\x04\x00'  # :
    b'\x0e\x11\x19\x15\x13\x11\x0e'  # 0
    b'\x04\x06\x04\x04\x04\x04\x0e'  # 1
    b'\x0e\x11\x10\x0e\x01\x01\x1f'  # 2
    b'\x0e\x11\x10\x0c\x10\x11\x0e'  # 3
    b'\x08\x0c\x0a\x09\x1f\x08\x08'  # 4
    b'\x1f\x01\x0f\x10\x10\x11\x0e'  # 5
    b'\x0e\x01\x01\x0f\x11\x11\x0e'  # 6
    b'\x1f\x10\x08\x04\x02\x02\x02'  # 7
    b'\x0e\x11\x11\x0e\x11\x11\x0e'  # 8
    b'\x0e\x11\x11\x1e\x10\x10\x0e'  # 9
    b'\x00\x00\x00\x00\x00\x00\x00'  # (space)
)


class SnakeGame:
    def __init__(self, display_manager):
        self.display = display_manager.display
        self.cell_size = 8
        self.grid_width = 16
        self.grid_height = 16

        # Game state — snake body as flat bytearray: [x0,y0, x1,y1, ...]
        self.snake = bytearray([8, 8, 7, 8, 6, 8])
        self.direction = (1, 0)
        self.food = None
        self.score = 0
        self.game_over = False
        self.just_ate = False

        # Create display elements
        self.palette = displayio.Palette(2)
        self.palette[0] = 0x000000
        self.palette[1] = 0xFFFFFF
        self.bitmap = displayio.Bitmap(128, 128, 2)
        self.tilegrid = displayio.TileGrid(self.bitmap, pixel_shader=self.palette)

        self.spawn_food()

    def _in_snake(self, x, y, skip_last=False):
        end = len(self.snake) - (2 if skip_last else 0)
        for i in range(0, end, 2):
            if self.snake[i] == x and self.snake[i + 1] == y:
                return True
        return False

    def draw_cell(self, cell_x, cell_y, color):
        for py in range(self.cell_size):
            for px in range(self.cell_size):
                x = cell_x * self.cell_size + px
                y = cell_y * self.cell_size + py
                self.bitmap[x, y] = color

    def clear_cell(self, cell_x, cell_y):
        self.draw_cell(cell_x, cell_y, 0)

    def spawn_food(self):
        while True:
            x = random.randint(0, self.grid_width - 1)
            y = random.randint(0, self.grid_height - 1)
            if not self._in_snake(x, y):
                self.food = (x, y)
                self.draw_cell(x, y, 1)
                break

    def turn_left(self):
        dx, dy = self.direction
        self.direction = (dy, -dx)

    def turn_right(self):
        dx, dy = self.direction
        self.direction = (-dy, dx)

    def update(self):
        if self.game_over:
            return

        self.just_ate = False

        head_x = self.snake[0]
        head_y = self.snake[1]
        dx, dy = self.direction
        new_x = (head_x + dx) % self.grid_width
        new_y = (head_y + dy) % self.grid_height

        # Check collision with body (exclude tail — it will move away)
        if self._in_snake(new_x, new_y, skip_last=True):
            self.game_over = True
            return

        # Prepend new head
        self.snake = bytearray([new_x, new_y]) + self.snake
        self.draw_cell(new_x, new_y, 1)

        if new_x == self.food[0] and new_y == self.food[1]:
            self.score += 1
            self.just_ate = True
            self.spawn_food()
        else:
            # Remove tail
            tail_x = self.snake[-2]
            tail_y = self.snake[-1]
            self.snake = self.snake[:-2]
            self.clear_cell(tail_x, tail_y)

    def draw_initial(self):
        for y in range(128):
            for x in range(128):
                self.bitmap[x, y] = 0

        for i in range(0, len(self.snake), 2):
            self.draw_cell(self.snake[i], self.snake[i + 1], 1)

        if self.food:
            self.draw_cell(self.food[0], self.food[1], 1)

    def show(self):
        splash = displayio.Group()
        splash.append(self.tilegrid)
        self.display.root_group = splash
        self.draw_initial()

    def show_game_over(self):
        """Display the game over splash screen with score using pixel art."""
        def draw_char(ch, x, y, scale):
            idx = _FONT_CHARS.find(ch)
            if idx < 0:
                return
            offset = idx * 7
            for row in range(7):
                byte = _FONT_DATA[offset + row]
                for col in range(5):
                    if byte & (1 << col):
                        for sy in range(scale):
                            for sx in range(scale):
                                px = x + col * scale + sx
                                py = y + row * scale + sy
                                if 0 <= px < 128 and 0 <= py < 128:
                                    self.bitmap[px, py] = 1

        def draw_text(text, y, scale):
            char_w = 5 * scale + scale  # char width + spacing
            total_w = len(text) * char_w - scale
            start_x = (128 - total_w) // 2
            for i, ch in enumerate(text):
                draw_char(ch, start_x + i * char_w, y, scale)

        # Clear screen
        for y in range(128):
            for x in range(128):
                self.bitmap[x, y] = 0

        # Draw "GAME" and "OVER" with scale 3
        draw_text("GAME", 15, 3)
        draw_text("OVER", 45, 3)

        # Draw score with scale 2
        draw_text("SCORE:" + str(self.score), 95, 2)


async def start_snake_game(display_manager, button_manager, disco_mode=None, pwr_pin=None):
    """
    Entry point to start the snake game.
    Call this from code.py when button 2 is held for 5 seconds.
    Press button 3 to exit and return to normal device mode.

    Args:
        display_manager: The DisplayManager instance from code.py
        button_manager: The ButtonManager instance from code.py
        disco_mode: The DiscoMode instance from code.py (optional, for light effects)
        pwr_pin: The power control pin for LTC2952 shutdown (optional)
    """
    print("\n" + "="*30)
    print("SNAKE GAME")
    print("="*30)
    print("Button 1 = Turn Left")
    print("Button 2 = Turn Right")
    print("Button 3 = Exit Game")
    print("="*30 + "\n")

    gc.collect()
    game = SnakeGame(display_manager)
    game.show()

    move_delay = 0.2
    last_move_time = time.monotonic()

    while not game.game_over:
        button_manager.update()

        if button_manager.was_pressed("Button 1"):
            game.turn_left()

        if button_manager.was_pressed("Button 2"):
            game.turn_right()

        if button_manager.was_pressed("Button 3"):
            print("Exiting Snake Game...")
            if disco_mode:
                disco_mode.turn_off()
            display_manager.display_screen_initialise()
            return

        if button_manager.was_pressed("Button 4") and pwr_pin is not None:
            print("Power off requested during snake game")
            pwr_pin.value = False

        current_time = time.monotonic()
        if current_time - last_move_time >= move_delay:
            game.update()
            last_move_time = current_time

            # Flash green when food is eaten
            if game.just_ate:
                if disco_mode:
                    disco_mode.set_green()
            else:
                if disco_mode:
                    disco_mode.turn_off()

        await asyncio.sleep(0.01)  # Poll buttons at 100Hz for better responsiveness

    # Game Over - show splash screen and flash red 3 times
    print(f"Game Over! Score: {game.score}")
    game.show_game_over()

    if disco_mode:
        for _ in range(3):
            disco_mode.set_red()
            await asyncio.sleep(0.3)
            disco_mode.turn_off()
            await asyncio.sleep(0.2)

    # Wait for user to see the game over screen
    await asyncio.sleep(3)
    display_manager.display_screen_initialise()
