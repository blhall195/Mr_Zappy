import time
import board
import busio
import digitalio

class BleManager:
    def __init__(self, uart_tx=board.D5, uart_rx=board.D6, drdy_pin=board.D12):
        # UART setup
        self.uart = busio.UART(uart_tx, uart_rx, baudrate=9600, timeout=0.1)

        # DRDY pin setup
        self.drdy = digitalio.DigitalInOut(drdy_pin)
        self.drdy.direction = digitalio.Direction.OUTPUT
        self.drdy.value = False

    def send_message(self, compass: float, clino: float, dist: float):
        """Format and send a survey data message."""
        message = f"COMPASS:{compass:.1f},CLINO:{clino:.1f},DIST:{dist:.1f}\n".encode("utf-8")
        self.drdy.value = True
        self.uart.write(message)
        time.sleep(0.1)
        self.drdy.value = False
        print("wrote message:", message.decode().strip())
