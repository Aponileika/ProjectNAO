"""ROS-independent planar extended Kalman filter."""

from __future__ import annotations

import numpy as np
from scipy.stats import chi2


class PlanarEkf:
    """Estimate planar pose, speed, and yaw-rate sensor bias."""

    STATE_SIZE = 6
    X = 0
    Y = 1
    QW = 2
    QZ = 3
    VELOCITY = 4
    GYRO_BIAS = 5

    def __init__(
        self,
        initial_covariance: np.ndarray,
        process_noise_rates: np.ndarray,
    ) -> None:
        """Initialize state as ``[x, y, qw, qz, velocity, gyro_bias]``."""
        self.state = np.zeros(self.STATE_SIZE, dtype=float)
        self.state[self.QW] = 1.0
        self.covariance = np.diag(
            self._validate_diagonal(initial_covariance, "initial_covariance"),
        )
        self.process_noise_rates = self._validate_diagonal(
            process_noise_rates,
            "process_noise_rates",
        )

    @classmethod
    def _validate_diagonal(cls, values: np.ndarray, name: str) -> np.ndarray:
        values = np.asarray(values, dtype=float)
        if values.shape != (cls.STATE_SIZE,):
            msg = f"{name} must contain {cls.STATE_SIZE} values"
            raise ValueError(msg)
        if not np.all(np.isfinite(values)) or np.any(values < 0.0):
            msg = f"{name} values must be finite and non-negative"
            raise ValueError(msg)
        return values

    @staticmethod
    def _stabilize_covariance(covariance: np.ndarray) -> np.ndarray:
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if eigenvalues[0] < 0.0:
            covariance = (eigenvectors * np.maximum(eigenvalues, 0.0)) @ (
                eigenvectors.T
            )
        return 0.5 * (covariance + covariance.T)

    @classmethod
    def _normalize(
        cls,
        state: np.ndarray,
        covariance: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        quaternion = state[cls.QW : cls.QZ + 1]
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("planar quaternion has zero or non-finite norm")

        normalized = quaternion / norm
        normalization_jacobian = (np.eye(2) - np.outer(normalized, normalized)) / norm
        transform = np.eye(cls.STATE_SIZE)
        transform[cls.QW : cls.QZ + 1, cls.QW : cls.QZ + 1] = normalization_jacobian
        state = state.copy()
        state[cls.QW : cls.QZ + 1] = normalized
        covariance = transform @ covariance @ transform.T
        return state, cls._stabilize_covariance(covariance)

    def predict(self, measured_yaw_rate: float, dt: float) -> None:
        """Advance the filter using a measured yaw rate and elapsed seconds."""
        if not np.isfinite(measured_yaw_rate):
            raise ValueError("measured_yaw_rate must be finite")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and greater than zero")

        x, y, qw, qz, velocity, gyro_bias = self.state
        yaw_rate = measured_yaw_rate - gyro_bias
        cos_yaw = qw * qw - qz * qz
        sin_yaw = 2.0 * qw * qz

        jacobian = np.eye(self.STATE_SIZE)
        jacobian[self.X, self.QW] = 2.0 * velocity * qw * dt
        jacobian[self.X, self.QZ] = -2.0 * velocity * qz * dt
        jacobian[self.X, self.VELOCITY] = cos_yaw * dt
        jacobian[self.Y, self.QW] = 2.0 * velocity * qz * dt
        jacobian[self.Y, self.QZ] = 2.0 * velocity * qw * dt
        jacobian[self.Y, self.VELOCITY] = sin_yaw * dt
        jacobian[self.QW, self.QZ] = -0.5 * yaw_rate * dt
        jacobian[self.QZ, self.QW] = 0.5 * yaw_rate * dt
        jacobian[self.QW, self.GYRO_BIAS] = 0.5 * qz * dt
        jacobian[self.QZ, self.GYRO_BIAS] = -0.5 * qw * dt

        predicted = self.state.copy()
        predicted[self.X] = x + velocity * cos_yaw * dt
        predicted[self.Y] = y + velocity * sin_yaw * dt
        predicted[self.QW] = qw - 0.5 * yaw_rate * qz * dt
        predicted[self.QZ] = qz + 0.5 * yaw_rate * qw * dt

        process_noise = np.diag(self.process_noise_rates * dt)
        covariance = jacobian @ self.covariance @ jacobian.T + process_noise
        self.state, self.covariance = self._normalize(predicted, covariance)

    def _measurement_update(
        self,
        measurement: np.ndarray,
        observation: np.ndarray,
        measurement_covariance: np.ndarray,
        gate_confidence: float,
    ) -> bool:
        innovation = measurement - observation @ self.state
        innovation_covariance = (
            observation @ self.covariance @ observation.T + measurement_covariance
        )
        try:
            scaled_innovation = np.linalg.solve(
                innovation_covariance,
                innovation,
            )
            distance_squared = float(innovation.T @ scaled_innovation)
            threshold = float(chi2.ppf(gate_confidence, measurement.size))
            if not np.isfinite(distance_squared) or distance_squared > threshold:
                return False

            covariance_observation = self.covariance @ observation.T
            gain = np.linalg.solve(
                innovation_covariance.T,
                covariance_observation.T,
            ).T
        except np.linalg.LinAlgError:
            return False

        updated_state = self.state + gain @ innovation
        identity = np.eye(self.STATE_SIZE)
        residual_transform = identity - gain @ observation
        updated_covariance = (
            residual_transform @ self.covariance @ residual_transform.T
            + gain @ measurement_covariance @ gain.T
        )
        if not (
            np.all(np.isfinite(updated_state))
            and np.all(np.isfinite(updated_covariance))
        ):
            return False

        try:
            self.state, self.covariance = self._normalize(
                updated_state,
                updated_covariance,
            )
        except ValueError:
            return False
        return True

    def update_pose(
        self,
        x: float,
        y: float,
        qw: float,
        qz: float,
        position_variance: float,
        orientation_variance: float,
        gate_confidence: float,
    ) -> bool:
        """Fuse a planar position and orientation measurement."""
        measurement = np.array([x, y, qw, qz], dtype=float)
        if not np.all(np.isfinite(measurement)):
            return False
        quaternion_norm = np.linalg.norm(measurement[2:4])
        if quaternion_norm < 1e-12:
            return False
        measurement[2:4] /= quaternion_norm

        observation = np.zeros((4, self.STATE_SIZE))
        observation[0:2, 0:2] = np.eye(2)
        observation[2:4, 2:4] = np.eye(2)
        if np.dot(measurement[2:4], self.state[2:4]) < 0.0:
            measurement[2:4] *= -1.0
        covariance = np.diag(
            [
                position_variance,
                position_variance,
                orientation_variance,
                orientation_variance,
            ],
        )
        return self._measurement_update(
            measurement,
            observation,
            covariance,
            gate_confidence,
        )

    def initialize_pose(
        self,
        x: float,
        y: float,
        qw: float,
        qz: float,
        position_variance: float,
        orientation_variance: float,
    ) -> bool:
        """Initialize pose from the first absolute measurement."""
        pose = np.array([x, y, qw, qz], dtype=float)
        if not np.all(np.isfinite(pose)):
            return False
        quaternion_norm = np.linalg.norm(pose[2:4])
        if quaternion_norm < 1e-12:
            return False

        state = self.state.copy()
        state[0:2] = pose[0:2]
        state[2:4] = pose[2:4] / quaternion_norm
        covariance = self.covariance.copy()
        pose_indices = (self.X, self.Y, self.QW, self.QZ)
        covariance[pose_indices, :] = 0.0
        covariance[:, pose_indices] = 0.0
        covariance[self.X, self.X] = position_variance
        covariance[self.Y, self.Y] = position_variance
        covariance[self.QW, self.QW] = orientation_variance
        covariance[self.QZ, self.QZ] = orientation_variance
        try:
            self.state, self.covariance = self._normalize(state, covariance)
        except ValueError:
            return False
        return True

    def update_velocity(
        self,
        velocity: float,
        variance: float,
        gate_confidence: float,
    ) -> bool:
        """Fuse a longitudinal velocity measurement."""
        if not np.isfinite(velocity) or not np.isfinite(variance) or variance <= 0.0:
            return False
        observation = np.zeros((1, self.STATE_SIZE))
        observation[0, self.VELOCITY] = 1.0
        return self._measurement_update(
            np.array([velocity]),
            observation,
            np.array([[variance]]),
            gate_confidence,
        )

    def constrain_velocity_to_zero(self, variance: float) -> None:
        """Apply the stop detector's zero-velocity constraint directly."""
        self.state[self.VELOCITY] = 0.0
        self.covariance[self.VELOCITY, :] = 0.0
        self.covariance[:, self.VELOCITY] = 0.0
        self.covariance[self.VELOCITY, self.VELOCITY] = variance
        self.covariance = self._stabilize_covariance(self.covariance)

    def initialize_velocity(self, velocity: float, variance: float) -> bool:
        """Initialize speed from the first valid wheel measurement."""
        if not np.isfinite(velocity) or not np.isfinite(variance) or variance <= 0.0:
            return False
        self.state[self.VELOCITY] = velocity
        self.covariance[self.VELOCITY, :] = 0.0
        self.covariance[:, self.VELOCITY] = 0.0
        self.covariance[self.VELOCITY, self.VELOCITY] = variance
        self.covariance = self._stabilize_covariance(self.covariance)
        return True

    def planar_pose_covariance(self) -> np.ndarray:
        """Return covariance for ``[x, y, yaw]``."""
        qw = self.state[self.QW]
        qz = self.state[self.QZ]
        norm_squared = qw * qw + qz * qz
        transform = np.zeros((3, self.STATE_SIZE))
        transform[0, self.X] = 1.0
        transform[1, self.Y] = 1.0
        transform[2, self.QW] = -2.0 * qz / norm_squared
        transform[2, self.QZ] = 2.0 * qw / norm_squared
        return transform @ self.covariance @ transform.T
