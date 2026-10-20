"""Minimal MPU-6050 accelerometer and gyroscope driver."""

from __future__ import annotations

from time import sleep
from typing import ClassVar, Protocol

import smbus2


class SMBus(Protocol):
    """Subset of the SMBus interface used by the driver."""

    def read_byte_data(self, address: int, register: int) -> int:
        """Read one byte from a device register."""
        ...

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        """Write one byte to a device register."""
        ...

    def close(self) -> None:
        """Close the bus."""
        ...


class MPU6050:
    """Read SI-unit acceleration and angular velocity from an MPU-6050."""

    GRAVITY_MS2 = 9.82

    ACCEL_SCALE_MODIFIERS: ClassVar[dict[int, float]] = {
        0x00: 16384.0,
        0x08: 8192.0,
        0x10: 4096.0,
        0x18: 2048.0,
    }
    GYRO_SCALE_MODIFIERS: ClassVar[dict[int, float]] = {
        0x00: 131.0,
        0x08: 65.5,
        0x10: 32.8,
        0x18: 16.4,
    }

    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18
    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F
    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    DEG2RAD = 0.0174532951

    def __init__(
        self,
        address: int,
        bus_number: int = 1,
        bus: SMBus | None = None,
    ) -> None:
        """Open the I2C bus and wake the device at *address*."""
        self.address = address
        self.bus = bus if bus is not None else smbus2.SMBus(bus_number)
        self.accel_x_bias = 0.0
        self.accel_y_bias = 0.0
        self.gyro_z_bias = 0.0
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def read_i2c_word(self, register: int) -> int:
        """Read a signed 16-bit value starting at *register*."""
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        return value - 65536 if value >= 0x8000 else value

    def calibrate(self, sample_count: int = 100) -> None:
        """Estimate planar acceleration and yaw-rate biases."""
        accel_x = accel_y = gyro_z = 0.0
        for _ in range(sample_count):
            ax, ay, _ = self.get_accel_data()
            _, _, gz = self.get_gyro_data()
            accel_x += ax
            accel_y += ay
            gyro_z += gz
            sleep(0.01)
        self.accel_x_bias = accel_x / sample_count
        self.accel_y_bias = accel_y / sample_count
        self.gyro_z_bias = gyro_z / sample_count

    def set_accel_range(self, accel_range: int) -> None:
        """Set the accelerometer range using an ``ACCEL_RANGE_*`` value."""
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw: bool = False) -> int:
        """Return the accelerometer range or its raw register value."""
        value = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        if raw:
            return value
        return {
            self.ACCEL_RANGE_2G: 2,
            self.ACCEL_RANGE_4G: 4,
            self.ACCEL_RANGE_8G: 8,
            self.ACCEL_RANGE_16G: 16,
        }.get(value, -1)

    def get_accel_data(self) -> tuple[float, float, float]:
        """Return acceleration along each axis in metres per second squared."""
        scale = self.ACCEL_SCALE_MODIFIERS.get(
            self.read_accel_range(raw=True),
            self.ACCEL_SCALE_MODIFIERS[self.ACCEL_RANGE_2G],
        )

        return (
            self.read_i2c_word(self.ACCEL_XOUT0) / scale * self.GRAVITY_MS2,
            self.read_i2c_word(self.ACCEL_YOUT0) / scale * self.GRAVITY_MS2,
            self.read_i2c_word(self.ACCEL_ZOUT0) / scale * self.GRAVITY_MS2,
        )

    def set_gyro_range(self, gyro_range: int) -> None:
        """Set the gyroscope range using a ``GYRO_RANGE_*`` value."""
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, *, raw: bool = False) -> int:
        """Return the configured gyroscope range or its raw register value."""
        value = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        if raw:
            return value
        return {
            self.GYRO_RANGE_250DEG: 250,
            self.GYRO_RANGE_500DEG: 500,
            self.GYRO_RANGE_1000DEG: 1000,
            self.GYRO_RANGE_2000DEG: 2000,
        }.get(value, -1)

    def get_gyro_data(self) -> tuple[float, float, float]:
        """Return angular velocity along each axis in radians per second."""
        scale = self.GYRO_SCALE_MODIFIERS.get(
            self.read_gyro_range(raw=True),
            self.GYRO_SCALE_MODIFIERS[self.GYRO_RANGE_250DEG],
        )

        return (
            self.read_i2c_word(self.GYRO_XOUT0) / scale * self.DEG2RAD,
            self.read_i2c_word(self.GYRO_YOUT0) / scale * self.DEG2RAD,
            self.read_i2c_word(self.GYRO_ZOUT0) / scale * self.DEG2RAD,
        )

    def close(self) -> None:
        """Close the underlying I2C bus."""
        self.bus.close()
