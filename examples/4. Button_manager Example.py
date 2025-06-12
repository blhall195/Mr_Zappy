import asyncio
import time
from button_manager import ButtonManager

# This class could represent some device, system, or functionality
class Device:
    def __init__(self):
        self.device_state = "Idle"

    def change_state(self, new_state):
        """Simulate changing device state."""
        print(f"Device state changed to: {new_state}")
        self.device_state = new_state

async def main():
    # Initialize the ButtonManager
    buttons = ButtonManager()

    # Initialize the device or system you want to control
    device = Device()

    # Asynchronous task for monitoring buttons
    async def monitor():
        while True:
            # Check button states asynchronously
            buttons.update()  # Update the button states

            # Example: If Button 1 is pressed, change the device state to 'Active'
            if buttons.was_pressed("Button 1"):
                print("Button 1 pressed!")
                device.change_state("Active")

            # Example: If Button 2 is pressed, change the device state to 'Sleep'
            if buttons.was_pressed("Button 2"):
                print("Button 2 pressed!")
                device.change_state("Sleep")

            # If "Fire Button" is pressed, perform a special action
            if buttons.was_pressed("Fire Button"):
                print("Fire Button pressed!")
                device.change_state("Firing")

            await asyncio.sleep(0.1)  # Yield control back to asyncio loop

    # Run the monitoring function
    await monitor()

# Run the async event loop
asyncio.run(main())
