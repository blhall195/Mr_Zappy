# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Phil Underwood for Underwood Underground
#
# SPDX-License-Identifier: MIT
"""
`laser_ldj100`
================================================================================

Device driver for the Meskernel LDJ100-689 laser distance sensor

* Author(s): Claude (based on laser_egismos by Phil Underwood)

Implementation Notes
--------------------

**Hardware:**

  *  `Meskernel LDJ100-689 <https://www.meskernel.com>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
"""
__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/furbrain/CircuitPython_laser_ldj100.git"

import time

try:
    from typing import Sequence, Tuple, Optional
except ImportError:
    pass

import busio

DEFAULT_TIMEOUT = 5.0
DEFAULT_BAUD_RATE = 115200


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


class TargetOutOfRangeError(LaserError):
    """
    Target is out of measurement range
    """


class TemperatureError(LaserError):
    """
    Module temperature is outside operating range
    """


class LowVoltageError(LaserError):
    """
    Input voltage is too low
    """


class HardwareError(LaserError):
    """
    Hardware error in the module
    """


class _LaserBase:
    # pylint: disable=too-few-public-methods
    FRAME_START = 0xAA

    # Register addresses
    REG_ERR_CODE = 0x0000
    REG_HW_VERSION = 0x000A
    REG_SW_VERSION = 0x000C
    REG_SERIAL_NUM = 0x000E
    REG_BAT_VLTG = 0x0006
    REG_ADDRESS = 0x0010
    REG_OFFSET = 0x0012
    REG_MEA_START = 0x0020
    REG_MEA_RESULT = 0x0022
    REG_CTRL_LD = 0x01BE

    # Measurement modes
    MODE_SINGLE_AUTO = 0x0000
    MODE_SINGLE_LOW_SPEED = 0x0001
    MODE_SINGLE_HIGH_SPEED = 0x0002
    MODE_CONTINUOUS_AUTO = 0x0004
    MODE_CONTINUOUS_LOW_SPEED = 0x0005
    MODE_CONTINUOUS_HIGH_SPEED = 0x0006

    # Laser control
    LASER_OFF = 0x0000
    LASER_ON = 0x0001

    # Exit continuous mode byte
    EXIT_CONTINUOUS = 0x58  # ASCII 'X'

    def __init__(self, uart: busio.UART, address=0x00, timeout=DEFAULT_TIMEOUT):
        """
        Access a Meskernel LDJ100-689 Laser distance module

        :param ~busio.UART uart: uart to use to connect. Should have baud rate set to 115200
        :param address: address to use, default is 0x00; change if using multiple devices
        :param timeout: timeout to wait for a response from the device
        """
        self.uart: busio.UART = uart
        self.address: int = address
        self.timeout: float = timeout

    def _build_frame(
        self, register: int, write: bool = False, address: Optional[int] = None,
        data: Optional[Sequence[int]] = None
    ) -> bytes:
        """
        Build a frame for the LDJ100 protocol

        Frame format:
        [Header][R/W+Address][Reg High][Reg Low][Count High][Count Low][Data...][Checksum]

        :param register: Register address (16-bit)
        :param write: True for write operation, False for read
        :param address: device address (7-bit), uses self.address if None
        :param data: data bytes to send (for write operations)
        :return: complete frame as bytes
        """
        if address is None:
            address = self.address
        if data is None:
            data = []
        if isinstance(data, int):
            data = [data]

        # Build R/W + Address byte (bit 7 = R/W, bits 6-0 = address)
        rw_address = (0x00 if write else 0x80) | (address & 0x7F)

        # Register address (16-bit, big-endian)
        reg_high = (register >> 8) & 0xFF
        reg_low = register & 0xFF

        # Payload count (16-bit, big-endian) - number of 16-bit words
        # For reads, this is typically 0x0001
        # For writes, this depends on the data length
        if write:
            payload_count = len(data) // 2 + (len(data) % 2)
        else:
            payload_count = 1
        count_high = (payload_count >> 8) & 0xFF
        count_low = payload_count & 0xFF

        # Build frame without checksum
        frame = [self.FRAME_START, rw_address, reg_high, reg_low,
                count_high, count_low] + list(data)

        # Calculate checksum (sum of all bytes except header and checksum)
        checksum = sum(frame[1:]) & 0xFF

        frame.append(checksum)
        return bytes(frame)

    def _parse_frame(self, frame: bytes) -> Tuple[int, int, bytes]:
        """
        Parse a frame and return the contained data

        :param bytes frame: The frame to be parsed
        :return: tuple of (register, address, data)
        :raises: `LaserCommandFailedError` if frame is invalid
        """
        if len(frame) < 7:
            raise LaserCommandFailedError(f"Frame too short: {len(frame)} bytes")

        if frame[0] != self.FRAME_START:
            raise LaserCommandFailedError(
                f"Frame does not start with 0x{self.FRAME_START:02X}"
            )

        # Verify checksum
        expected_checksum = sum(frame[1:-1]) & 0xFF
        if frame[-1] != expected_checksum:
            raise LaserCommandFailedError(
                f"Checksum mismatch: expected 0x{expected_checksum:02X}, "
                f"got 0x{frame[-1]:02X}"
            )

        # Parse header
        rw_address = frame[1]
        address = rw_address & 0x7F
        reg_high = frame[2]
        reg_low = frame[3]
        register = (reg_high << 8) | reg_low

        # Data starts at byte 6
        data = frame[6:-1]

        return register, address, data

    def _process_frame(self, expected_register: int, expected_address: Optional[int],
                      frame: bytes) -> bytes:
        """
        Process a received frame and validate it matches expectations

        :param expected_register: The register we expect in the response
        :param expected_address: The address we expect (or None to use self.address)
        :param frame: The received frame
        :return: The data portion of the frame
        """
        if expected_address is None:
            expected_address = self.address

        register, address, data = self._parse_frame(frame)

        if register != expected_register:
            raise LaserCommandFailedError(
                f"Register mismatch: expected 0x{expected_register:04X}, "
                f"got 0x{register:04X}"
            )

        if address != expected_address:
            raise LaserCommandFailedError(
                f"Address mismatch: expected 0x{expected_address:02X}, "
                f"got 0x{address:02X}"
            )

        return data

    @staticmethod
    def _check_status_code(status_code: int):
        """
        Check status code and raise appropriate exception if there's an error

        :param status_code: 16-bit status code from the device
        :raises: Various LaserError subclasses based on the error
        """
        if status_code == 0x0000:
            return  # No error

        error_messages = {
            0x0001: ("Input voltage too low (should be ≥ 2.0V)", LowVoltageError),
            0x0002: ("Network error (can be ignored)", LaserError),
            0x0003: ("Module temperature too low (<-20°C)", TemperatureError),
            0x0004: ("Module temperature too high (>+60°C)", TemperatureError),
            0x0005: ("Target out of range", TargetOutOfRangeError),
            0x0006: ("Invalid measurement value", BadReadingError),
            0x0007: ("Excessive ambient light", TooBrightError),
            0x0008: ("Weak laser signal - check for contamination or increase reflectivity", TooDimError),
            0x0009: ("Strong laser signal - reduce target reflectivity", TooBrightError),
            0x000A: ("Hardware error", HardwareError),
            0x000F: ("Laser signal instability - stabilize device or check power", BadReadingError),
            0x0081: ("Invalid communication format", LaserCommandFailedError),
        }

        if status_code in error_messages:
            message, error_class = error_messages[status_code]
            raise error_class(f"Error 0x{status_code:04X}: {message}")
        else:
            raise LaserError(f"Unknown error code: 0x{status_code:04X}")


class Laser(_LaserBase):
    """
    This is a driver for the LDJ100-689 Laser Distance Sensor by Meskernel
    """

    def _read_frame(self) -> bytes:
        """
        Read a complete frame from the UART

        :return: Complete frame including header and checksum
        :raises: LaserTimeOutError if timeout occurs
        """
        timeout_due = time.monotonic() + self.timeout
        buffer = b""

        # Wait for frame start
        while True:
            byte = self.uart.read(1)
            if byte and byte[0] == self.FRAME_START:
                buffer = byte
                break
            if time.monotonic() > timeout_due:
                raise LaserTimeOutError("Timed out waiting for FRAME_START")

        # Read at least the header (7 bytes minimum)
        while len(buffer) < 7:
            byte = self.uart.read(1)
            if byte:
                buffer += byte
            if time.monotonic() > timeout_due:
                raise LaserTimeOutError("Timed out reading frame header")

        # Determine expected frame length from payload count
        count_high = buffer[4]
        count_low = buffer[5]
        payload_count = (count_high << 8) | count_low

        # Total length = header(6) + data(payload_count * 2) + checksum(1)
        expected_length = 7 + (payload_count * 2)

        # Read remaining bytes
        while len(buffer) < expected_length:
            byte = self.uart.read(1)
            if byte:
                buffer += byte
            if time.monotonic() > timeout_due:
                raise LaserTimeOutError("Timed out reading frame data")

        return buffer

    def _send_and_receive(
        self, register: int, write: bool = False, data: Optional[Sequence[int]] = None,
        address: Optional[int] = None
    ) -> bytes:
        """
        Send a command and receive the response

        :param register: Register to read/write
        :param write: True for write, False for read
        :param data: Data to write (for write operations)
        :param address: Device address (uses self.address if None)
        :return: Data portion of the response
        """
        frame = self._build_frame(register, write, address, data)
        self.uart.reset_input_buffer()
        self.uart.write(frame)

        response = self._read_frame()
        return self._process_frame(register, address, response)

    def _read_register(self, register: int, address: Optional[int] = None) -> bytes:
        """
        Read a register from the device

        :param register: Register address
        :param address: Device address
        :return: Register data as bytes
        """
        return self._send_and_receive(register, write=False, address=address)

    def _write_register(self, register: int, data: Sequence[int],
                       address: Optional[int] = None):
        """
        Write data to a register

        :param register: Register address
        :param data: Data to write
        :param address: Device address
        """
        self._send_and_receive(register, write=True, data=data, address=address)

    def get_status(self) -> int:
        """
        Read the current status code from the device

        :return: Status code (0x0000 = no error)
        """
        data = self._read_register(self.REG_ERR_CODE)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    def check_status(self):
        """
        Check device status and raise exception if there's an error

        :raises: Various LaserError subclasses if there's an error
        """
        status = self.get_status()
        self._check_status_code(status)

    def get_hardware_version(self) -> int:
        """
        Read the hardware version

        :return: Hardware version as 16-bit integer
        """
        data = self._read_register(self.REG_HW_VERSION)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    def get_software_version(self) -> int:
        """
        Read the software version

        :return: Software version as 16-bit integer
        """
        data = self._read_register(self.REG_SW_VERSION)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    def get_serial_number(self) -> int:
        """
        Read the module serial number

        :return: Serial number as 32-bit integer
        """
        data = self._read_register(self.REG_SERIAL_NUM)
        if len(data) >= 4:
            return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        return 0

    def get_voltage(self) -> int:
        """
        Read the input voltage in millivolts

        :return: Voltage in mV
        """
        data = self._read_register(self.REG_BAT_VLTG)
        if len(data) >= 2:
            # Voltage is returned in BCD format (e.g., 0x3219 = 3219 mV)
            high = data[0]
            low = data[1]
            return (high << 8) | low
        return 0

    def set_laser(self, value: bool):
        """
        Turn the laser pointer on or off

        :param bool value: If ``True``, turn on laser, turn off if ``False``
        """
        laser_value = self.LASER_ON if value else self.LASER_OFF
        data = [0x00, 0x01, 0x00, laser_value & 0xFF]
        self._write_register(self.REG_CTRL_LD, data)

    def set_slave_address(self, address: int):
        """
        Set the device address (0x00-0x7E, do not use 0x7F which is broadcast)

        :param int address: Address to use (0-126)
        """
        if address < 0 or address > 0x7E:
            raise ValueError("Address must be between 0 and 0x7E (126)")

        data = [0x00, 0x01, 0x00, address]
        self._write_register(self.REG_ADDRESS, data)
        self.address = address

    def set_offset(self, offset_mm: int):
        """
        Set the measurement offset in millimeters

        The offset adjusts the zero point of measurements. Positive values increase
        the reading, negative values decrease it.

        :param int offset_mm: Offset in millimeters (can be negative)
        """
        # Convert to 16-bit signed value
        if offset_mm < 0:
            offset_value = (1 << 16) + offset_mm  # Two's complement
        else:
            offset_value = offset_mm

        data = [0x00, 0x01, (offset_value >> 8) & 0xFF, offset_value & 0xFF]
        self._write_register(self.REG_OFFSET, data)

    def get_offset(self) -> int:
        """
        Read the current measurement offset

        :return: Offset in millimeters (signed)
        """
        data = self._read_register(self.REG_OFFSET)
        if len(data) >= 4:
            # Data format: [0x00, 0x01, offset_high, offset_low]
            offset = (data[2] << 8) | data[3]
            # Convert from unsigned to signed
            if offset & 0x8000:
                offset = offset - (1 << 16)
            return offset
        return 0

    def stop_measuring(self):
        """
        Stop continuous measurement mode
        """
        self.uart.write(bytes([self.EXIT_CONTINUOUS]))

    def _measure_internal(self, mode: int) -> Tuple[int, int]:
        """
        Internal method to perform a measurement

        :param mode: Measurement mode
        :return: Tuple of (distance_mm, signal_quality)
        """
        # Start measurement
        data = [0x00, 0x01, (mode >> 8) & 0xFF, mode & 0xFF]
        self._write_register(self.REG_MEA_START, data)

        # Read result
        result = self._read_register(self.REG_MEA_RESULT)

        # Parse result: [count_high, count_low, dist[3], dist[2], dist[1], dist[0],
        #                 sig[1], sig[0]]
        # We expect count = 0x0003 (3 words = 6 bytes)
        if len(result) < 8:
            raise LaserCommandFailedError("Invalid measurement result length")

        # Distance is bytes 2-5 (32-bit, big-endian)
        distance = (result[2] << 24) | (result[3] << 16) | (result[4] << 8) | result[5]

        # Signal quality is bytes 6-7 (16-bit, big-endian)
        signal_quality = (result[6] << 8) | result[7]

        return distance, signal_quality

    def measure(self, mode: str = "auto") -> int:
        """
        Make a single distance measurement

        :param str mode: Measurement mode - "auto", "low_speed", or "high_speed"
                        auto: balanced speed and accuracy
                        low_speed: highest accuracy
                        high_speed: fastest measurement
        :return: distance in mm
        :raises: Various LaserError subclasses on error
        """
        mode_map = {
            "auto": self.MODE_SINGLE_AUTO,
            "low_speed": self.MODE_SINGLE_LOW_SPEED,
            "high_speed": self.MODE_SINGLE_HIGH_SPEED,
        }

        if mode not in mode_map:
            raise ValueError(f"Invalid mode: {mode}. Use 'auto', 'low_speed', or 'high_speed'")

        distance, _ = self._measure_internal(mode_map[mode])

        # Check for errors after measurement
        self.check_status()

        return distance

    @property
    def distance(self) -> float:
        """
        Get the distance in centimeters (using auto mode)

        :return: Distance in cm
        :raises: Various LaserError subclasses on error
        """
        return self.measure(mode="auto") / 10.0

    def measure_with_quality(self, mode: str = "auto") -> Tuple[int, int]:
        """
        Make a measurement and return both distance and signal quality

        Lower signal quality values indicate stronger signals and more reliable readings.

        :param str mode: Measurement mode - "auto", "low_speed", or "high_speed"
        :return: Tuple of (distance_mm, signal_quality)
        """
        mode_map = {
            "auto": self.MODE_SINGLE_AUTO,
            "low_speed": self.MODE_SINGLE_LOW_SPEED,
            "high_speed": self.MODE_SINGLE_HIGH_SPEED,
        }

        if mode not in mode_map:
            raise ValueError(f"Invalid mode: {mode}. Use 'auto', 'low_speed', or 'high_speed'")

        distance, quality = self._measure_internal(mode_map[mode])
        self.check_status()

        return distance, quality


class AsyncLaser(_LaserBase):
    """
    Same as `Laser`, but with async methods, requires the `asyncio` module
    """

    def __init__(self, uart: busio.UART, address=0x00, timeout=DEFAULT_TIMEOUT):
        # pylint: disable=import-outside-toplevel
        import asyncio

        uart.timeout = 0
        super().__init__(uart, address, timeout)
        self.async_reader = asyncio.StreamReader(uart)

    async def _read_frame(self) -> bytes:
        """
        Asynchronously read a complete frame from the UART

        :return: Complete frame including header and checksum
        """
        buffer = b""

        # Wait for frame start
        while True:
            byte = await self.async_reader.read(1)
            if byte and byte[0] == self.FRAME_START:
                buffer = byte
                break

        # Read header (at least 7 bytes)
        while len(buffer) < 7:
            byte = await self.async_reader.read(1)
            if byte:
                buffer += byte

        # Determine expected frame length
        count_high = buffer[4]
        count_low = buffer[5]
        payload_count = (count_high << 8) | count_low
        expected_length = 7 + (payload_count * 2)

        # Read remaining bytes
        while len(buffer) < expected_length:
            byte = await self.async_reader.read(1)
            if byte:
                buffer += byte

        return buffer

    async def _send_and_receive(
        self, register: int, write: bool = False, data: Optional[Sequence[int]] = None,
        address: Optional[int] = None
    ) -> bytes:
        """
        Asynchronously send a command and receive the response
        """
        # pylint: disable=import-outside-toplevel
        import asyncio

        frame = self._build_frame(register, write, address, data)
        self.uart.write(frame)

        try:
            response = await asyncio.wait_for(self._read_frame(), self.timeout)
        except asyncio.TimeoutError as exc:
            raise LaserTimeOutError("Did not receive response within timeout") from exc

        return self._process_frame(register, address, response)

    async def _read_register(self, register: int, address: Optional[int] = None) -> bytes:
        """Async read register"""
        return await self._send_and_receive(register, write=False, address=address)

    async def _write_register(self, register: int, data: Sequence[int],
                             address: Optional[int] = None):
        """Async write register"""
        await self._send_and_receive(register, write=True, data=data, address=address)

    async def get_status(self) -> int:
        """Async get status"""
        data = await self._read_register(self.REG_ERR_CODE)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    async def check_status(self):
        """Async check status"""
        status = await self.get_status()
        self._check_status_code(status)

    async def get_hardware_version(self) -> int:
        """Async get hardware version"""
        data = await self._read_register(self.REG_HW_VERSION)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    async def get_software_version(self) -> int:
        """Async get software version"""
        data = await self._read_register(self.REG_SW_VERSION)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    async def get_serial_number(self) -> int:
        """Async get serial number"""
        data = await self._read_register(self.REG_SERIAL_NUM)
        if len(data) >= 4:
            return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
        return 0

    async def get_voltage(self) -> int:
        """Async get voltage"""
        data = await self._read_register(self.REG_BAT_VLTG)
        if len(data) >= 2:
            return (data[0] << 8) | data[1]
        return 0

    async def set_laser(self, value: bool):
        """
        Asynchronously turn the laser pointer on or off

        :param bool value: If ``True``, turn on laser, turn off if ``False``
        """
        laser_value = self.LASER_ON if value else self.LASER_OFF
        data = [0x00, 0x01, 0x00, laser_value & 0xFF]
        await self._write_register(self.REG_CTRL_LD, data)

    async def set_slave_address(self, address: int):
        """Async set slave address"""
        if address < 0 or address > 0x7E:
            raise ValueError("Address must be between 0 and 0x7E (126)")

        data = [0x00, 0x01, 0x00, address]
        await self._write_register(self.REG_ADDRESS, data)
        self.address = address

    async def set_offset(self, offset_mm: int):
        """Async set offset"""
        if offset_mm < 0:
            offset_value = (1 << 16) + offset_mm
        else:
            offset_value = offset_mm

        data = [0x00, 0x01, (offset_value >> 8) & 0xFF, offset_value & 0xFF]
        await self._write_register(self.REG_OFFSET, data)

    async def get_offset(self) -> int:
        """Async get offset"""
        data = await self._read_register(self.REG_OFFSET)
        if len(data) >= 4:
            offset = (data[2] << 8) | data[3]
            if offset & 0x8000:
                offset = offset - (1 << 16)
            return offset
        return 0

    async def stop_measuring(self):
        """
        Asynchronously stop continuous measurement mode
        """
        self.uart.write(bytes([self.EXIT_CONTINUOUS]))

    async def _measure_internal(self, mode: int) -> Tuple[int, int]:
        """Async internal measure"""
        data = [0x00, 0x01, (mode >> 8) & 0xFF, mode & 0xFF]
        await self._write_register(self.REG_MEA_START, data)

        result = await self._read_register(self.REG_MEA_RESULT)

        if len(result) < 8:
            raise LaserCommandFailedError("Invalid measurement result length")

        distance = (result[2] << 24) | (result[3] << 16) | (result[4] << 8) | result[5]
        signal_quality = (result[6] << 8) | result[7]

        return distance, signal_quality

    async def measure(self, mode: str = "auto") -> int:
        """
        Asynchronously make a single distance measurement

        :param str mode: Measurement mode - "auto", "low_speed", or "high_speed"
        :return: distance in mm
        :raises: Various LaserError subclasses on error
        """
        mode_map = {
            "auto": self.MODE_SINGLE_AUTO,
            "low_speed": self.MODE_SINGLE_LOW_SPEED,
            "high_speed": self.MODE_SINGLE_HIGH_SPEED,
        }

        if mode not in mode_map:
            raise ValueError(f"Invalid mode: {mode}. Use 'auto', 'low_speed', or 'high_speed'")

        distance, _ = await self._measure_internal(mode_map[mode])
        await self.check_status()

        return distance

    async def measure_with_quality(self, mode: str = "auto") -> Tuple[int, int]:
        """
        Asynchronously measure with signal quality

        :param str mode: Measurement mode
        :return: Tuple of (distance_mm, signal_quality)
        """
        mode_map = {
            "auto": self.MODE_SINGLE_AUTO,
            "low_speed": self.MODE_SINGLE_LOW_SPEED,
            "high_speed": self.MODE_SINGLE_HIGH_SPEED,
        }

        if mode not in mode_map:
            raise ValueError(f"Invalid mode: {mode}. Use 'auto', 'low_speed', or 'high_speed'")

        distance, quality = await self._measure_internal(mode_map[mode])
        await self.check_status()

        return distance, quality
