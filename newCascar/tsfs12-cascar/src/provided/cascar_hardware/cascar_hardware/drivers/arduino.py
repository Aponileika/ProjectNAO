"""Serial driver for the Arduino motor and wheel-encoder controller."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from numbers import Real
from pathlib import Path
from time import sleep

import serial


class SerialConnectionError(ConnectionError):
    """Raised when the Arduino cannot be reached or fails its handshake."""


@dataclass(frozen=True)
class CommandResult:
    """The normalized values successfully sent to the controller."""

    speed: float | None
    steer: float | None


@dataclass(frozen=True)
class EncoderReading:
    """One wheel-magnet event reported by the controller."""

    wheel: str
    delta_seconds: float


class ArduinoDriver:
    """Own the serial connection and the Arduino wire protocol."""

    def __init__(self, connection: serial.Serial) -> None:
        """Wrap an open, handshaken serial connection."""
        self._connection = connection
        self._connection.timeout = 0
        self._encoder_buffer = bytearray()
        self._discard_encoder_line = False

    @classmethod
    def connect(
        cls,
        port: str,
        baud_rate: int = 115200,
        timeout: float = 1,
        settle_time: float = 2,
    ) -> ArduinoDriver:
        """Open *port* and verify the device responds to PING."""
        if not Path(port).exists():
            msg = f"Serial port does not exist: {port}"
            raise SerialConnectionError(msg)

        try:
            connection = serial.Serial(port, baud_rate, timeout=timeout)
        except serial.SerialException as error:
            msg = f"Failed to open serial port: {port}"
            raise SerialConnectionError(msg) from error
        try:
            connection.flush()
            sleep(settle_time)
            connection.write(b"PING;\r")
            sleep(0.05)

            while connection.in_waiting > 0:
                message = connection.readline().decode("utf-8").split(";", 1)[0]
                if message == "PONG":
                    return cls(connection)
        except Exception:
            connection.close()
            raise

        connection.close()
        msg = f"No PONG response from serial port: {port}"
        raise SerialConnectionError(msg)

    @property
    def in_waiting(self) -> int:
        """Include complete lines already buffered by the driver."""
        buffered = len(self._encoder_buffer) if b"\n" in self._encoder_buffer else 0
        return self._connection.in_waiting + buffered

    @property
    def readable(self) -> bool:
        """Return whether the serial connection can be read."""
        return self._connection.readable()

    @property
    def writable(self) -> bool:
        """Return whether the serial connection can be written."""
        return self._connection.writable()

    @staticmethod
    def normalize_control_value(value: float) -> float | None:
        """Validate and clamp a controller percentage to [-100, 100]."""
        if isinstance(value, bool) or not isinstance(value, Real):
            return None
        numeric_value = float(value)
        if not isfinite(numeric_value):
            return None
        return max(min(numeric_value, 100.0), -100.0)

    @classmethod
    def speed_command(cls, speed: float) -> str:
        """Build a percentage throttle command, or an empty invalid command."""
        normalized = cls.normalize_control_value(speed)
        return "" if normalized is None else f"T;{int(normalized)}\r"

    @classmethod
    def steer_command(cls, steer: float) -> str:
        """Build a percentage steering command, or an empty invalid command."""
        normalized = cls.normalize_control_value(steer)
        return "" if normalized is None else f"S;{int(normalized)}\r"

    def send(
        self,
        speed: float | None = None,
        steer: float | None = None,
    ) -> CommandResult | None:
        """Send controls and return the values accepted by the controller."""
        normalized_speed = (
            self.normalize_control_value(speed) if speed is not None else None
        )
        normalized_steer = (
            self.normalize_control_value(steer) if steer is not None else None
        )
        command = ""
        if normalized_speed is not None:
            command += self.speed_command(normalized_speed)
        if normalized_steer is not None:
            command += self.steer_command(normalized_steer)

        if not command or not self.writable:
            return None

        self._connection.write(command.encode())
        return CommandResult(normalized_speed, normalized_steer)

    def read_encoder(self) -> EncoderReading | None:
        """Parse one complete event without waiting for more serial bytes."""
        if not self.readable:
            return None
        if b"\n" not in self._encoder_buffer:
            self._encoder_buffer.extend(
                self._connection.read(min(self._connection.in_waiting, 256)),
            )
        raw_message, separator, remainder = self._encoder_buffer.partition(b"\n")
        if not separator:
            # Bound memory even if a faulty device never terminates its lines.
            if len(self._encoder_buffer) > 256:
                self._encoder_buffer.clear()
                self._discard_encoder_line = True
            return None
        self._encoder_buffer = remainder
        if self._discard_encoder_line or len(raw_message) > 256:
            self._discard_encoder_line = False
            return None
        try:
            message = raw_message.decode("utf-8")
            wheel, raw_delta = message.split(";", 1)
            delta_seconds = float(raw_delta) / 1e6
        except (UnicodeDecodeError, ValueError):
            return None

        if wheel not in {"L", "R"} or not isfinite(delta_seconds) or delta_seconds <= 0:
            return None
        return EncoderReading(wheel, delta_seconds)

    def close(self) -> None:
        """Close the underlying serial connection."""
        self._connection.close()
