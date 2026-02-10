import board
import time
import busio
import asyncio
import digitalio
import _bleio
import microcontroller
from adafruit_ble import BLERadio, Advertisement
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
import caveble  # Custom BLE service

# --- NVM Name Storage ---
DEFAULT_NAME = "SAP6_Basic"
NVM_MAGIC = 0xBE
MAX_NAME_LEN = 20

def read_name_from_nvm():
    if microcontroller.nvm[0] != NVM_MAGIC:
        return DEFAULT_NAME
    length = microcontroller.nvm[1]
    if length == 0 or length > MAX_NAME_LEN:
        return DEFAULT_NAME
    try:
        return bytes(microcontroller.nvm[2:2 + length]).decode("utf-8")
    except Exception:
        return DEFAULT_NAME

def save_name_to_nvm(name):
    encoded = name.encode("utf-8")[:MAX_NAME_LEN]
    microcontroller.nvm[0] = NVM_MAGIC
    microcontroller.nvm[1] = len(encoded)
    microcontroller.nvm[2:2 + len(encoded)] = encoded

# BLE Setup
ble = BLERadio()
ble.name = read_name_from_nvm()
print(f"BLE Name: {ble.name}")

# Clear any stored bonds so the device always advertises with its
# public address. No characteristics require encryption, so bonding
# is unnecessary and stale bonds can make the device undiscoverable
# on some phones after unpairing.
_bleio.adapter.erase_bonding()
print("Cleared BLE bonds")

survey_protocol = caveble.SurveyProtocolService()
advertisement = ProvideServicesAdvertisement(survey_protocol)

# Create a scan response to include complete device name and tx power
scan_response = Advertisement()
scan_response.complete_name = ble.name
scan_response.tx_power = 8  # Boost to +8 dBm

# Start advertising with boosted power
ble._adapter.start_advertising(
    bytes(advertisement),
    scan_response=bytes(scan_response),
    tx_power=8  # Boost TX power to +8 dBm
)

# UART Setup
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=0.1)

# DRDY Pin Setup (used to signal new data from master board?)
drdy = digitalio.DigitalInOut(board.D7)
drdy.direction = digitalio.Direction.INPUT
drdy.pull = digitalio.Pull.DOWN

# BLE_Connected Pin Setup (used as an output indicator for BLE status)
ble_connected = digitalio.DigitalInOut(board.D5)
ble_connected.direction = digitalio.Direction.OUTPUT
ble_connected.value = False  # Start LOW (disconnected)

#BLE_Connected Pin Setup
LZR_power = digitalio.DigitalInOut(board.A2)
LZR_power.direction = digitalio.Direction.OUTPUT
LZR_power.value = True  # Start HIGH power on

def restart_advertising_with_name(new_name):
    global advertisement, scan_response
    if ble.advertising:
        ble.stop_advertising()
    ble.name = new_name
    advertisement = ProvideServicesAdvertisement(survey_protocol)
    scan_response = Advertisement()
    scan_response.complete_name = new_name
    scan_response.tx_power = 8
    if not ble.connected:
        ble._adapter.start_advertising(
            bytes(advertisement),
            scan_response=bytes(scan_response),
            tx_power=8
        )

# UART Parsing Helper (only if your BLE send expects float data)
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

# BLE Send: Triggered when DRDY indicates new UART data is ready
async def read_uart_loop():
    line_buffer = b''
    while True:
        if drdy.value:  # Master is signaling new data is ready
            await asyncio.sleep(0.01)  # Small debounce delay

            data = uart.read(64)
            if data:
                line_buffer += data
                while b'\n' in line_buffer:
                    line, line_buffer = line_buffer.split(b'\n', 1)
                    decoded_line = line.decode("utf-8").strip()

                    if decoded_line == "ALIVE":
                        print("Keep-alive received via UART")
                        continue

                    if decoded_line.startswith("NAME:"):
                        new_name = decoded_line[5:].strip()
                        if 1 <= len(new_name) <= MAX_NAME_LEN:
                            save_name_to_nvm(new_name)
                            restart_advertising_with_name(new_name)
                            print(f"BLE name changed to: {new_name}")
                        else:
                            print(f"Invalid name length: {len(new_name)}")
                        continue

                    parsed = parse_uart_line(line)
                    if parsed:
                        compass, clino, distance = parsed
                        print(f"Sending over BLE: {parsed}")
                        survey_protocol.send_data(compass, clino, distance)

            # Wait for DRDY to go low again
            while drdy.value:
                await asyncio.sleep(0.01)
        else:
            await asyncio.sleep(0.01)


async def monitor_ble_messages():
    while True:
        message = survey_protocol.poll()
        if message is not None:
            print(f"Received from BLE: {message}")
            try:
                if isinstance(message, str):
                    uart.write((message + "\n").encode("utf-8"))
                elif isinstance(message, bytes):
                    uart.write(message + b"\n")
                else:
                    uart.write((str(message) + "\n").encode("utf-8"))
            except Exception as e:
                print(f"UART write error: {e}")
        await asyncio.sleep(0.01)


async def poll_ble_loop():
    while True:
        message = survey_protocol.poll()
        if message is not None:
            print(f"Received from BLE: {message}")
            try:
                if message == survey_protocol.ACK0:
                    uart.write(b"ACK_RECEIVED\n")
                elif message == survey_protocol.ACK1:
                    uart.write(b"ACK_RECEIVED\n")
                else:
                    # Forward other commands as strings
                    if isinstance(message, str):
                        uart.write((message + "\n").encode("utf-8"))
                    elif isinstance(message, bytes):
                        uart.write(message + b"\n")
                    else:
                        uart.write((str(message) + "\n").encode("utf-8"))
            except Exception as e:
                print(f"UART write error: {e}")
        await asyncio.sleep(0.01)



# BLE connection monitor: updates the ble_connected pin
async def monitor_ble_connection():
    last_state = ble.connected
    while True:
        current_state = ble.connected
        if current_state != last_state:
            if current_state:
                print("🔵 BLE Connected")
                ble_connected.value = True
                # Boost TX power for all active connections
                for conn in ble.connections:
                    try:
                        conn.tx_power = 8
                        print("📶 Set connection TX power to +8 dBm")
                    except Exception as e:
                        print(f"⚠️ Failed to set TX power: {e}")
            else:
                print("🔴 BLE Disconnected")
                ble_connected.value = False
                # Wipe any bonds created during the session so the
                # device always advertises on its public address.
                try:
                    _bleio.adapter.erase_bonding()
                except Exception as e:
                    print(f"⚠️ Failed to erase bonding: {e}")
                if not ble.connected and not ble.advertising:
                    try:
                        print("📣 Restarting BLE advertising at +8 dBm")
                        ble._adapter.start_advertising(
                            bytes(advertisement),
                            scan_response=bytes(scan_response),
                            tx_power=8
                        )
                    except Exception as e:
                        print(f"⚠️ Failed to restart advertising: {e}")
            last_state = current_state
        await asyncio.sleep(0.1)

# Main loop
async def main():
    tasks = [
        asyncio.create_task(read_uart_loop()),
        asyncio.create_task(monitor_ble_messages()),
        asyncio.create_task(monitor_ble_connection()),
    ]
    while True:
        await asyncio.sleep(0.1)


# Run everything
asyncio.run(main())
