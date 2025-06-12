# Write your code here :-)
import asyncio
from button_manager import ButtonManager
from ble_manager import BleManager

async def main():
    buttons = ButtonManager()
    ble = BleManager()

    async def monitor_buttons():
        while True:
            buttons.update()

            if buttons.was_pressed("Fire Button"):
                print("Fire Button pressed!")
                # Replace these with live sensor values if needed
                ble.send_message(123.4, 56.7, 8.9)

            await asyncio.sleep(0.1)

    await monitor_buttons()

# Start the event loop
asyncio.run(main())
