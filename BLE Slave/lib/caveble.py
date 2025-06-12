# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: MIT
"""
`caveble`
================================================================================

Cave Surveying Bluetooth Protocol - communicate via BLE with paperless cave surveying tools
e.g. `TopoDroid <https://github.com/marcocorvi/topodroid>`_


* Author(s): Phil Underwood

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

"""
__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/furbrain/CircuitPython_CaveBLE.git"

import time
from collections import deque

from adafruit_ble import Service
from adafruit_ble.attributes import Attribute
from adafruit_ble.characteristics import StructCharacteristic, Characteristic
from adafruit_ble.characteristics.int import Uint8Characteristic
from adafruit_ble.characteristics.string import FixedStringCharacteristic
from adafruit_ble.uuid import VendorUUID

try:
    from typing import Deque, Callable, Optional, Coroutine, Tuple
except ImportError:
    pass


class SurveyProtocolService(Service):
    """
    This service provide a BLE style interface to access data from the device
    """

    ACK = [0x56, 0x55]
    ACK0 = 0x55  #: Acknowledge a leg with sequence bit 0
    ACK1 = 0x56  #: Acknowledge a leg with sequence bit 1
    START_CAL = 0x31  #: Start Calibration
    STOP_CAL = 0x30  #: Finish calibration
    LASER_ON = 0x36  #: Turn laser on
    LASER_OFF = 0x37  #: Turn laser off
    DEVICE_OFF = 0x34  #: Turn device off
    TAKE_SHOT = 0x38  #: Take a reading

    uuid = VendorUUID("137c4435-8a64-4bcb-93f1-3792c6bdc965")
    protocol_name = FixedStringCharacteristic(
        uuid=VendorUUID("137c4435-8a64-4bcb-93f1-3792c6bdc966"),
    )
    command = Uint8Characteristic(
        uuid=VendorUUID("137c4435-8a64-4bcb-93f1-3792c6bdc967"),
        properties=Characteristic.WRITE,
        read_perm=Attribute.NO_ACCESS,
    )
    leg = StructCharacteristic(
        struct_format="<Bffff",
        uuid=VendorUUID("137c4435-8a64-4bcb-93f1-3792c6bdc968"),
        properties=Characteristic.READ | Characteristic.NOTIFY,
        write_perm=Attribute.NO_ACCESS,
    )

    def __init__(self):
        super().__init__(protocol_name="SAP6")
        self.last_send_time: float = 0
        # pylint: disable=too-many-function-args
        self.send_queue: Deque[Tuple[float, float, float, float]] = deque((), 20)
        self.sent_packet: bytes = b""
        self.last_sent_bit = 0
        self.waiting_for_ack: bool = False
        self.incomplete_packets: bytes = b""

    def send_data(self, azimuth, inclination, distance, roll=0):
        """
        Add a reading to the list to be sent

        :param float azimuth: Compass bearing in degrees
        :param float inclination: Inclination in degrees
        :param float distance: Distance in metres
        """
        self.send_queue.append((azimuth, inclination, roll, distance))
        self._poll_out()

    def pending(self) -> int:
        """
        How many readings are waiting to be sent

        :return: Number of readings queued
        """
        if self.waiting_for_ack:
            return len(self.send_queue) + 1
        else:
            return len(self.send_queue)

    def _poll_out(self):
        if self.waiting_for_ack:
            # resend if last packet sent more than 5s ago
            if time.monotonic() - self.last_send_time > 5.0:
                self.leg = self.leg  # resend notify with unchanged data
                self.last_send_time = time.monotonic()
        else:
            if self.send_queue:
                azimuth, inclination, roll, distance = self.send_queue.popleft()
                self.leg = (self.last_sent_bit, azimuth, inclination, roll, distance)
                self.last_sent_bit ^= 1
                self.last_send_time = time.monotonic()
                self.waiting_for_ack = True

    def _poll_in(self):
        cmd = self.command
        if cmd != 0:
            print(f"got cmd: {cmd}")
            # process command
            self.command = 0
            expected_ack = self.ACK[self.last_sent_bit]
            unexpected_ack = self.ACK[self.last_sent_bit ^ 1]
            if cmd == expected_ack:
                print("ACK received")
                self.waiting_for_ack = False
                return None
            elif cmd == unexpected_ack:
                print("Wrong ack received: expecting ", expected_ack)
                return None
            else:
                return cmd
        return None

    def poll(self) -> Optional[int]:
        """
        Check to see if any messages need sending or re-sending. This should be called
        every 100ms or so. It will send any data due to be sent and will resend data if no
        acknowledgement has been received. It will also return any messages received
        (one of ``START_CAL``, ``STOP_CAL``, ``START_SILENT``, ``STOP_SILENT`` etc). Note the memory
        access commands are currently unsupported and will be ignored.

        :return: None (if no command received) or an int of one of the above constants
        """
        result = self._poll_in()
        self._poll_out()
        return result

    async def background_task(
        self, callback: Optional[Callable[[int], Coroutine]] = None
    ):
        # pylint: disable=import-outside-toplevel
        """
        You can use this as a background async task instead of doing regular polls. It
        will check for new messages and if there are any outstanding outgoing messages
        every 100ms or so.

        :param Callable callback: Async function to call with any messages received, function
          must take one argument, which will be an integer matching one of the above constants
        :return: None
        """
        import asyncio

        cb_tasks = []
        try:
            while True:
                await asyncio.sleep(0.1)
                cmd = self._poll_in()
                if cmd is not None and callback is not None:
                    res = callback(cmd)
                    if hasattr(res, "__await__"):
                        cb_tasks.append(asyncio.create_task(res))
                self._poll_out()
                cb_tasks = [x for x in cb_tasks if not x.done]
        finally:
            for task in cb_tasks:
                task.cancel()