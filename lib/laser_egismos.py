# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: MIT
"""
`laser_egismos`
================================================================================

Device driver for the egismos series of lasers, available at
https://www.egismos.com/laser-measuring-optoelectronics-module


* Author(s): Phil Underwood

Implementation Notes
--------------------

**Hardware:**

  *  `Egismos laser <https://www.egismos.com/laser-measuring-optoelectronics-module>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
"""
__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/furbrain/CircuitPython_laser_egismos.git"

import time

try:
    from typing import Sequence, Tuple
except ImportError:
    pass

import busio

DEFAULT_TIMEOUT = 5.0


class LaserError(RuntimeError):
    """
    An error from the laser module
    """


class LaserCommandFailedError(LaserError):
    """
    Laser command not recognised or acknowledged
    """


class BadReadingError(LaserError):
    """
    Error while making a reading
    """


class TooDimError(LaserError):
    """
    Laser return was too dim to be interpreted
    """


class TooBrightError(LaserError):
    """
    Laser was too bright to get accurate reading
    """


class LaserTimeOutError(LaserError):
    """
    Laser took too long to respond
    """


class _LaserBase:
    # pylint: disable=too-few-public-methods
    FRAME_END = 0xA8
    FRAME_START = 0xAA
    BUZZER_CONTROL = 0x47
    STOP_CONTINUOUS_MEASURE = 0x46
    CONTINUOUS_MEASURE = 0x45
    SINGLE_MEASURE = 0x44
    LASER_OFF = 0x43
    LASER_ON = 0x42
    READ_DEVICE_ERR = 0x08
    SET_SLAVE_ADDRESS = 0x41
    READ_SLAVE_ADDRESS = 0x04
    READ_DEV_TYPE = 0x02
    READ_SW_VERSION = 0x01

    def __init__(self, uart: busio.UART, address=0x01, timeout=DEFAULT_TIMEOUT):
        """
        Access an Egismos Laser distance module v2

        :param ~busio.UART uart: uart to use to connect. Should have baud rate set to 9600
        :param address: address to use, default is 0x01; you should only change this if
          using multiple devices
        :param timeout: timeout to wait for a response from the device
        """
        self.uart: busio.UART = uart
        self.address: int = address
        self.timeout: float = timeout

    def _build_frame(
        self, command: int, address=None, data: Sequence[int] = None
    ) -> bytes:
        """
        Build a frame that represents the given command

        :param command: Command to send
        :param address: address to send to, default is 1
        :param data: int or list of ints, represents data parts to send
        :return:
        """
        if address is None:
            address = self.address
        if data is None:
            data = []
        if isinstance(data, int):
            data = [data]
        checksum = (command + address + sum(data)) & 0x7F
        frame = [self.FRAME_START, address, command] + data + [checksum, self.FRAME_END]
        return bytes(frame)

    def _parse_frame(self, frame: bytes) -> Tuple[int, int, bytes]:
        """
        Parse a frame and return the contained data. Raises a value error if incorrect
        start or end bytes, or if the checksum is incorrect
        :param bytes frame: The frame to be parsed
        :return: a tuple containing the command
        :raises: `ValueError` if an error is encountered
        """
        if frame[0] != self.FRAME_START:
            raise LaserCommandFailedError(
                f"Frame does not start with {self.FRAME_START}"
            )
        if frame[-1] != self.FRAME_END:
            raise LaserCommandFailedError(f"Frame does not end with {self.FRAME_END}")
        checksum = sum(frame[1:-2]) & 0x7F
        if frame[-2] != checksum:
            raise LaserCommandFailedError(
                f"Checksum should be {checksum}, was {frame[-2]}"
            )
        command = frame[2]
        address = frame[1]
        data = frame[3:-2]
        return command, address, data

    def _process_frame(self, address, command, frame):
        if address is None:
            address = self.address
        read_command, read_address, read_data = self._parse_frame(frame)
        if command != read_command:
            raise LaserCommandFailedError(
                f"Received command {read_command} does not match"
                + f" sent command {command}"
            )
        if address != read_address:
            raise LaserCommandFailedError(
                f"Received address {read_address} does not match"
                + f" sent address {address}"
            )
        return read_data

    @staticmethod
    def _check_measurement_for_errors(result):
        if result == b"ERR256":
            raise TooBrightError("Too much ambient light, or laser too close")
        if result == b"ERR255":
            raise TooDimError(
                "Laser spot too dim. Use reflective tape or shorter distance"
            )
        if result == b"ERR204":
            raise BadReadingError("Unable to measure - is the target moving?")
        try:
            result = int(result)
        except ValueError as exc:
            raise LaserCommandFailedError("Unexpected response from read") from exc
        return result


class Laser(_LaserBase):
    """
    This is a driver for the Laser Module 2, produced by Egismos
    """

    def _read_frame(self):
        # wait for an AA to start
        timeout_due = time.monotonic() + self.timeout
        buffer = b"\x00"
        while buffer[0] != self.FRAME_START:
            buffer = self.uart.read(1) or b"\x00"
            if time.monotonic() > timeout_due:
                raise LaserTimeOutError("Timed Out waiting for FRAME_START")
        while buffer[-1] != self.FRAME_END:
            buffer += self.uart.read(1) or b""
            if time.monotonic() > timeout_due:
                raise LaserTimeOutError("Timed Out waiting for FRAME_END")
        return buffer

    def _send_and_receive(
        self, command: int, data: int = None, address: int = None
    ) -> bytes:
        frame = self._build_frame(command, address, data)
        self.uart.reset_input_buffer()  # clear input before writing
        self.uart.write(frame)
        frame = self._read_frame()
        read_data = self._process_frame(address, command, frame)
        return read_data

    def _send_command_and_raise_on_failure(self, command, data=None):
        """
        Send a command, and raise `LaserCommandFailedError` if it does not succeed
        :param int command:
        :param data: Optional data byte or sequence for the command
        """
        result = self._send_and_receive(command, data)
        if not result or result[0] != 0x01:
            raise LaserCommandFailedError(f"Tried to send {command} but it failed")

    def set_laser(self, value: bool):
        """
        Turn the laser pointer on

        :param bool value: If ``True``, turn on laser, turn off if ``False``
        """
        if value:
            self._send_command_and_raise_on_failure(self.LASER_ON)
        else:
            self._send_command_and_raise_on_failure(self.LASER_OFF)

    def stop_measuring(self):
        """
        Stop measuring when in continuous mode

        :return:
        """
        self._send_command_and_raise_on_failure(self.STOP_CONTINUOUS_MEASURE)

    def set_buzzer(self, value: bool):
        """
        Turn on or off beeps when receiving commands

        :param bool value: If ``True``, turn on beeps, turn off if ``False``
        :return:
        """
        self._send_command_and_raise_on_failure(self.BUZZER_CONTROL, int(value))

    def set_slave_address(self, address):
        """
        Set the address of the laser pointer

        :param int address: Address to use - between 1 and 255
        """
        self._send_command_and_raise_on_failure(self.SET_SLAVE_ADDRESS, address)
        self.address = address

    def measure(self) -> int:
        """
        Make a single reading.

        :return: distance in mm
        :raises: `LaserError`; can be one of

          `TooDimError`
            Can't see the laser spot properly. Use reflective tape or a shorter distance
          `TooBrightError`
            Laser spot is too bright (maybe too close to the device or there may be too much
            ambient light)
          `BadReadingError`
            Measurement failed, often due to movement
          `LaserCommandFailedError`
            Return value from laser was garbled
        """
        result = self._send_and_receive(self.SINGLE_MEASURE)
        result = self._check_measurement_for_errors(result)
        return result

    @property
    def distance(self) -> float:
        """
        Get the distance in cm

        :return: Distance in cm
        :raises: Same as `measure`
        """
        return self.measure() / 10.0


class AsyncLaser(_LaserBase):
    """
    Same as `Laser`, but with async methods, requires the `asyncio` module
    """

    def __init__(self, uart: busio.UART, address=0x01, timeout=DEFAULT_TIMEOUT):
        # pylint: disable=import-outside-toplevel
        import asyncio

        uart.timeout = 0
        super().__init__(uart, address, timeout)
        self.async_reader = asyncio.StreamReader(uart)

    async def _read_frame(self):
        buffer = b"\x00"
        while buffer[0] != self.FRAME_START:
            buffer = await self.async_reader.read(1) or b""
        while buffer[-1] != self.FRAME_END:
            buffer += await self.async_reader.read(1) or b""
        return buffer

    async def _send_and_receive(
        self, command: int, data: int = None, address: int = None
    ) -> bytes:
        # pylint: disable=import-outside-toplevel
        import asyncio

        frame = self._build_frame(command, address, data)
        self.uart.write(frame)
        try:
            frame = await asyncio.wait_for(self._read_frame(), self.timeout)
        except asyncio.TimeoutError as exc:
            raise LaserTimeOutError("Did not receive response within timeout") from exc
        read_data = self._process_frame(address, command, frame)
        return read_data

    async def _send_command_and_raise_on_failure(self, command, data=None):
        """
        Send a command, and raise `LaserCommandFailedError` if it does not succeed
        :param int command:
        :param data: Optional data byte or sequence for the command
        """
        result = await self._send_and_receive(command, data)
        if not result or result[0] != 0x01:
            raise LaserCommandFailedError(f"Tried to send {command} but it failed")

    async def set_laser(self, value: bool):
        """
        Turn the laser pointer on or off

        :param bool value: If ``True``, turn on laser, turn off if ``False``
        """
        if value:
            await self._send_command_and_raise_on_failure(self.LASER_ON)
        else:
            await self._send_command_and_raise_on_failure(self.LASER_OFF)

    async def stop_measuring(self):
        """
        Stop measuring when in continuous mode

        :return:
        """
        await self._send_command_and_raise_on_failure(self.STOP_CONTINUOUS_MEASURE)

    async def set_buzzer(self, value: bool):
        """
        Turn on or off beeps when receiving commands

        :param bool value: If ``True``, turn on beeps, turn off if ``False``
        :return:
        """
        await self._send_command_and_raise_on_failure(self.BUZZER_CONTROL, int(value))

    async def set_slave_address(self, address):
        """
        Set the address of the laser pointer

        :param int address: Address to use - between 1 and 255
        """
        await self._send_command_and_raise_on_failure(self.SET_SLAVE_ADDRESS, address)
        self.address = address

    async def measure(self) -> int:
        """
        Make a single reading.

        :return: distance in mm
        :raises: `LaserError`; can be one of

          `TooDimError`
            Can't see the laser spot properly. Use reflective tape or a shorter distance
          `TooBrightError`
            Laser spot is too bright (maybe too close to the device or there may be too much
            ambient light)
          `BadReadingError`
            Measurement failed, often due to movement
          `LaserCommandFailedError`
            Return value from laser was garbled
        """
        result = await self._send_and_receive(self.SINGLE_MEASURE)
        result = self._check_measurement_for_errors(result)
        return result
