import asyncio
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
import caveble

class BLEService:
    def __init__(self, name="SAP6_AB", protocol=None):
        self.ble = BLERadio()
        self.ble.name = name
        print(f"BLE Name: {self.ble.name}")
        
        self.survey_protocol = protocol if protocol else caveble.SurveyProtocolService()
        self.advertisement = ProvideServicesAdvertisement(self.survey_protocol)
        
    def start_advertising(self):
        print("Starting BLE advertising...")
        self.ble.start_advertising(self.advertisement)
        
    def stop_advertising(self):
        print("Stopping BLE advertising...")
        self.ble.stop_advertising()

    async def wait_for_connection(self):
        print("Waiting for BLE connection...")
        while not self.ble.connected:
            await asyncio.sleep(0.2)  # Keep checking for a connection
            
        print("BLE Connected!")

    async def wait_for_disconnection(self):
        print("Waiting for BLE disconnection...")
        while self.ble.connected:
            await asyncio.sleep(0.2)  # Keep checking for disconnection
            
        print("BLE Disconnected!")

    async def manage_connection(self):
        while True:
            try:
                # Only start advertising if BLE is not connected and not advertising
                if not self.ble.connected and not self.ble.advertising:
                    self.start_advertising()

                await self.wait_for_connection()

                # Maintain connection while BLE is connected
                while self.ble.connected:
                    await asyncio.sleep(0.2)  # Keep the connection alive

                # Once disconnected, stop advertising
                self.stop_advertising()
                await self.wait_for_disconnection()

            except Exception as e:
                print(f"BLE Error: {e}")

            await asyncio.sleep(1)  # Restart BLE operation in case of error

        
    def send_survey_data(self, compass, clino, distance):
        """
        This method sends the survey data using the protocol.
        """
        try:
            # Send the data over BLE using the survey protocol
            self.survey_protocol.send_data(compass, clino, distance)
            print(f"Sent survey data: Compass={compass}, Clino={clino}, Distance={distance}")
        except Exception as e:
            print(f"Error sending survey data: {e}")



