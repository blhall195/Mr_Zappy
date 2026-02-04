import time
import asyncio
import random
import displayio
import microcontroller


class SnakeGame:
    def __init__(self, display_manager):
        self.display = display_manager.display
        self.cell_size = 8
        self.grid_width = 16
        self.grid_height = 16

        # Game state
        self.snake = [(8, 8), (7, 8), (6, 8)]
        self.direction = (1, 0)
        self.food = None
        self.score = 0
        self.game_over = False
        self.just_ate = False  # Flag to track when food is eaten

        # Create display elements
        self.palette = displayio.Palette(2)
        self.palette[0] = 0x000000
        self.palette[1] = 0xFFFFFF
        self.bitmap = displayio.Bitmap(128, 128, 2)
        self.tilegrid = displayio.TileGrid(self.bitmap, pixel_shader=self.palette)

        self.spawn_food()

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
            if (x, y) not in self.snake:
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

        self.just_ate = False  # Reset flag

        head_x, head_y = self.snake[0]
        dx, dy = self.direction
        new_head = (
            (head_x + dx) % self.grid_width,
            (head_y + dy) % self.grid_height
        )

        if new_head in self.snake[:-1]:
            self.game_over = True
            return

        self.snake.insert(0, new_head)
        self.draw_cell(new_head[0], new_head[1], 1)

        if new_head == self.food:
            self.score += 1
            self.just_ate = True  # Set flag when food is eaten
            self.spawn_food()
        else:
            old_tail = self.snake.pop()
            self.clear_cell(old_tail[0], old_tail[1])

    def draw_initial(self):
        for y in range(128):
            for x in range(128):
                self.bitmap[x, y] = 0

        for seg_x, seg_y in self.snake:
            self.draw_cell(seg_x, seg_y, 1)

        if self.food:
            self.draw_cell(self.food[0], self.food[1], 1)

    def show(self):
        splash = displayio.Group()
        splash.append(self.tilegrid)
        self.display.root_group = splash
        self.draw_initial()


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

    # Game Over - flash red 3 times
    print(f"Game Over! Score: {game.score}")
    if disco_mode:
        for _ in range(3):
            disco_mode.set_red()
            await asyncio.sleep(0.3)
            disco_mode.turn_off()
            await asyncio.sleep(0.2)

    await asyncio.sleep(1)
    display_manager.display_screen_initialise()
