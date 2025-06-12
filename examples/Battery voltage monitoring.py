import board
import analogio
import time

# Define the pin for battery voltage measurement
VBAT_PIN = board.VOLTAGE_MONITOR  # Adjust based on your board, if needed

# Initialize the analog pin
vbat_pin = analogio.AnalogIn(VBAT_PIN)


while True:
    raw_value = vbat_pin.value # Read the voltage
    voltage = ((raw_value / 65535) * 3.3) * 2
    bat_percentage = ((voltage)/4.2)*100 # Convert to bat %
    print(f"{bat_percentage:.0f}%") # Print the result in volts 
    time.sleep(1) # Wait a second before reading again

