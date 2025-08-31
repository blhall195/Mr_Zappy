import board
import time
import digitalio
import alarm
import asyncio
import busio
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
import caveble  # Assumed to contain SurveyProtocolService
from button_manager import ButtonManager
import microcontroller

# Wake detection
if alarm.wake_alarm:
    print("🌙 Woke from sleep.")
else:
    print("🌀 First boot or reset.")

#Turn on M4 microcontroller + sensors
M4_en_pin = digitalio.DigitalInOut(board.D12)
M4_en_pin.direction = digitalio.Direction.OUTPUT
M4_en_pin.value = False
#
print("hello world")

# BLE Setup
ble = BLERadio()
ble.name = "SAP6_BH"
print(f"BLE Name: {ble.name}")
survey_protocol = caveble.SurveyProtocolService()
advertisement = ProvideServicesAdvertisement(survey_protocol)
ble.start_advertising(advertisement)

# UART Setup
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=0.1)

# DRDY Pin Setup
drdy = digitalio.DigitalInOut(board.D7)
drdy.direction = digitalio.Direction.INPUT
drdy.pull = digitalio.Pull.DOWN

#BLE_Connected Pin Setup
ble_connected = digitalio.DigitalInOut(board.D5)
ble_connected.direction = digitalio.Direction.OUTPUT
ble_connected.value = False  # Start LOW (disconnected)

#BLE_Connected Pin Setup
LZR_power = digitalio.DigitalInOut(board.A2)
LZR_power.direction = digitalio.Direction.OUTPUT
LZR_power.value = True  # Start HIGH power on

# UART Parsing
line_buffer = b''

def parse_uart_line(line):
    try:
        decoded = line.decode("utf-8").strip()
        parts = decoded.split(',')
        if len(parts) != 3:
            raise ValueError("Invalid message format")
        compass = float(parts[0].split(':')[1])
        clino = float(parts[1].split(':')[1])
        distance = float(parts[2].split(':')[1])
        return compass, clino, distance
    except Exception as e:
        print(f"Parse error: {e}")
        return None

async def read_uart_loop():
    global line_buffer
    while True:
        if drdy.value:
            await asyncio.sleep(0.01)
            if not drdy.value:
                await asyncio.sleep(0.01)
                continue

            data = uart.read(64)
            if data:
                line_buffer += data
                while b'\n' in line_buffer:
                    line, line_buffer = line_buffer.split(b'\n', 1)
                    parsed = parse_uart_line(line)

                    # Decode the line for keep-alive check
                    decoded_line = line.decode("utf-8").strip()

                    #Detect keep-alive message here
                    if decoded_line == "ALIVE":
                        print("Keep-alive received via UART")
                        # Update timestamp or trigger logic here
                        global last_activity_time
                        last_activity_time = time.monotonic()
                        continue  # optionally skip further processing

                    if parsed:
                        compass, clino, distance = parsed
                        print(f"Received: compass={compass}, clino={clino}, distance={distance}")

                        if getattr(ble, "connected", False):
                            survey_protocol.send_data(compass, clino, distance)
                            print("Data sent via Bluetooth")

            while drdy.value:
                await asyncio.sleep(0.01)
        else:
            await asyncio.sleep(0.01)

async def poll_ble_loop():
    while True:
        message = survey_protocol.poll()
        if message:
            print(f"Message received: {message}")
            try:
                # Convert message to bytes if not already
                if isinstance(message, str):
                    uart.write((message + "\n").encode("utf-8"))
                elif isinstance(message, bytes):
                    uart.write(message + b"\n")
                else:
                    # Fallback for other types (e.g., dicts or objects)
                    uart.write((str(message) + "\n").encode("utf-8"))

                print("📤 Message forwarded via UART")
            except Exception as e:
                print(f"⚠️ UART send failed: {e}")
        await asyncio.sleep(0.01)

async def monitor_ble_connection(device):
    last_state = ble.connected
    while True:
        current_state = ble.connected
        if current_state != last_state:
            if current_state:
                print("🔵 BLE Connected")
                device.change_state("Active")
                ble_connected.value = True  # Set HIGH when connected
            else:
                print("🔴 BLE Disconnected")
                device.change_state("Idle")
                ble_connected.value = False  # Set LOW when disconnected
            last_state = current_state
        await asyncio.sleep(0.1)

async def blink_led():
    await asyncio.sleep(0.1)
    led.value = False

# Device class
class Device:
    def __init__(self, buttons):
        self.device_state = "Idle"
        self.buttons = buttons
        self.sleep_requested = False

    def change_state(self, new_state):
        print(f"Device state changed to: {new_state}")
        self.device_state = new_state

        if new_state == "Sleep":
            self.sleep_requested = True
        elif new_state == "Active":
            print("✅ Device is now Active.")

# Monitor button presses
async def monitor_buttons(buttons, device):
    while True:
        buttons.update()

        if buttons.was_pressed("Button 1"):
            print("Button 1 pressed!")
            device.change_state("Sleep")
            print(f"Current state: {device.device_state}")

        await asyncio.sleep(0.1)

last_activity_time = 0

async def keep_alive_watchdog(device, timeout=60):
    global last_activity_time
    check_interval = 1  # how often to check (in seconds)

    while True:
        now = time.monotonic()
        if now - last_activity_time > timeout:
            print("⚠️ Keep-alive timeout! Going to sleep...")
            device.change_state("Sleep")
            break  # Exit loop to allow main() to sleep
        await asyncio.sleep(check_interval)
        print(now)

# Main entry
async def main():
    buttons = ButtonManager()
    device = Device(buttons)

    tasks = [
        asyncio.create_task(read_uart_loop()),
        asyncio.create_task(poll_ble_loop()),
        asyncio.create_task(monitor_buttons(buttons, device)),
        asyncio.create_task(monitor_ble_connection(device)),
        asyncio.create_task(keep_alive_watchdog(device)),
    ]

    while True:
        await asyncio.sleep(0.1)
        if device.sleep_requested:
            print("🔁 Cancelling async tasks before sleep...")
            for t in tasks:
                t.cancel()
            break  # Exit to go to sleep

    # Let tasks cancel cleanly
    await asyncio.sleep(0.1)

    # Prepare for deep sleep
    print("💤 Preparing for deep sleep...")

    print("⏳ Waiting for button release before sleeping...")
    while True:
        buttons.update()
        if not buttons.is_pressed("Button 1"):
            break

    pin_alarm = alarm.pin.PinAlarm(pin=board.A0, value=False, pull=True)

    M4_en_pin.value = True
    LZR_power.value = False

    alarm.exit_and_deep_sleep_until_alarms(pin_alarm)

# Start the loop
asyncio.run(main())
