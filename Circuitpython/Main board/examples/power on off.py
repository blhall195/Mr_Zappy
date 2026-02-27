# from board_definitions import adafruit_feather_esp32_v2a as board
import board
import digitalio
import asyncio
import countio
import alarm
import time

i2c_power_pin = digitalio.DigitalInOut(board.NEOPIXEL_I2C_POWER)
# button = digitalio.DigitalInOut(board.BUTTON)

i2c_power_pin.switch_to_output(value=True)

# pin12 = digitalio.DigitalInOut(board.D12)
# pin12.switch_to_input(pull=True)
# pin12.direction = digitalio.Direction.INPUT
# pin12.pull = digitalio.Pull.UP



SLEEP_STATE = False
button_alarm = alarm.pin.PinAlarm(pin=board.D13, value=False, pull=True)


async def catch_interrupt(pin):
    global SLEEP_STATE

    with countio.Counter(pin, pull=digitalio.Pull.UP) as interrupt:
        while True:
            if interrupt.count > 0:
                print("interupted")
                interrupt.count = 0
                if SLEEP_STATE:
                    print("sleep off")
                else:
                    print("sleep on")
                    time.sleep(0.5)
                    alarm.exit_and_deep_sleep_until_alarms(button_alarm)
                SLEEP_STATE = not SLEEP_STATE

            # Let another task run.
            await asyncio.sleep(0)


async def main():
    interrupt_task = asyncio.create_task(catch_interrupt(board.D13))

    await asyncio.gather(interrupt_task)


asyncio.run(main())