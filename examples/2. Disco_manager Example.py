import asyncio
from disco_manager import DiscoMode

disco_mode = DiscoMode()

async def main():
    disco_mode.on()
    while True:
        await asyncio.sleep(1)  # Keep the loop alive

asyncio.run(main())
