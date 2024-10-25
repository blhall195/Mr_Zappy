import board
import analogio
import time

# Define the pin for battery voltage measurement
VBAT_PIN = board.VOLTAGE_MONITOR  # Adjust based on your board, if needed

# Initialize the analog pin
vbat_pin = analogio.AnalogIn(VBAT_PIN)

# Function to convert the raw ADC value to a voltage
def get_voltage(pin):
    # Get the raw ADC value (0 to 65535)
    raw_value = pin.value
    # Convert to voltage based on board's reference voltage (typically 3.3V or 5V)
    # The full range (65535) corresponds to the reference voltage
    voltage = (raw_value / 65535) * 3.3  # Adjust 3.3 if your reference voltage is different
    return voltage

while True:
    # Read the voltage
    measured_vbat = get_voltage(vbat_pin)
    
    # The voltage divider divides the battery voltage by 2, so multiply back
    measured_vbat *= 2
    
    # Print the result in volts
    print("VBat: {:.2f} V".format(measured_vbat))
    
    # Wait a second before reading again
    time.sleep(1)

