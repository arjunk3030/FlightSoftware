"""
Flight Controller Module

Provides PID-based attitude and altitude control for an autonomous aircraft.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import time


class FlightMode(Enum):
    """Operating modes of the flight controller."""
    GROUND = "GROUND"
    TAKEOFF = "TAKEOFF"
    CRUISE = "CRUISE"
    HOVER = "HOVER"
    LAND = "LAND"
    EMERGENCY = "EMERGENCY"


@dataclass
class AttitudeState:
    """Current attitude (orientation) of the aircraft."""
    roll: float = 0.0       # degrees, positive = right wing down
    pitch: float = 0.0      # degrees, positive = nose up
    yaw: float = 0.0        # degrees, 0 = north, clockwise positive
    roll_rate: float = 0.0  # degrees/sec
    pitch_rate: float = 0.0 # degrees/sec
    yaw_rate: float = 0.0   # degrees/sec


@dataclass
class ControlOutput:
    """Actuator commands produced by the controller."""
    throttle: float = 0.0   # 0.0 – 1.0
    aileron: float = 0.0    # -1.0 – 1.0 (roll)
    elevator: float = 0.0   # -1.0 – 1.0 (pitch)
    rudder: float = 0.0     # -1.0 – 1.0 (yaw)


@dataclass
class PIDState:
    """Running state for a single PID channel."""
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    previous_error: float = 0.0
    integral_limit: float = 100.0

    def compute(self, error: float, dt: float) -> float:
        """Compute PID output given error and time-step."""
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self.integral = 0.0
        self.previous_error = 0.0


class FlightController:
    """
    PID-based flight controller.

    Controls roll, pitch, yaw and altitude using four independent PID loops.
    """

    MAX_ROLL_DEG = 45.0
    MAX_PITCH_DEG = 30.0
    MAX_YAW_RATE = 60.0   # deg/sec
    MAX_CLIMB_RATE = 5.0  # m/s

    def __init__(self) -> None:
        self.mode: FlightMode = FlightMode.GROUND
        self.state: AttitudeState = AttitudeState()
        self.altitude: float = 0.0          # metres AGL
        self.target_altitude: float = 0.0   # metres AGL
        self.target_roll: float = 0.0
        self.target_pitch: float = 0.0
        self.target_yaw: float = 0.0

        self._roll_pid = PIDState(kp=1.2, ki=0.05, kd=0.3)
        self._pitch_pid = PIDState(kp=1.2, ki=0.05, kd=0.3)
        self._yaw_pid = PIDState(kp=0.8, ki=0.02, kd=0.1)
        self._altitude_pid = PIDState(kp=0.5, ki=0.01, kd=0.2)

        self._last_update: Optional[float] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def arm(self) -> None:
        """Transition from GROUND to ready-for-takeoff."""
        if self.mode != FlightMode.GROUND:
            raise RuntimeError(f"Cannot arm in mode {self.mode}")
        self._reset_pids()
        self.mode = FlightMode.TAKEOFF

    def disarm(self) -> None:
        """Return to GROUND state and zero all outputs."""
        self.mode = FlightMode.GROUND
        self._reset_pids()
        self._last_update = None

    def set_target_altitude(self, altitude_m: float) -> None:
        if altitude_m < 0:
            raise ValueError("Target altitude must be non-negative.")
        self.target_altitude = altitude_m

    def set_target_attitude(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self.target_roll = max(-self.MAX_ROLL_DEG, min(self.MAX_ROLL_DEG, roll))
        self.target_pitch = max(-self.MAX_PITCH_DEG, min(self.MAX_PITCH_DEG, pitch))
        self.target_yaw = yaw % 360.0

    def update(
        self,
        state: AttitudeState,
        altitude: float,
        timestamp: Optional[float] = None,
    ) -> ControlOutput:
        """
        Ingest new sensor data and produce actuator commands.

        Args:
            state: Current attitude from IMU.
            altitude: Current altitude in metres AGL.
            timestamp: Monotonic time in seconds (uses ``time.monotonic()`` if None).

        Returns:
            ControlOutput with throttle, aileron, elevator, rudder values.
        """
        if timestamp is None:
            timestamp = time.monotonic()

        dt = 0.0
        if self._last_update is not None:
            dt = timestamp - self._last_update
        self._last_update = timestamp

        self.state = state
        self.altitude = altitude

        if self.mode in (FlightMode.GROUND, FlightMode.EMERGENCY):
            return ControlOutput()

        output = ControlOutput()

        roll_error = self.target_roll - state.roll
        pitch_error = self.target_pitch - state.pitch
        yaw_error = self._yaw_error(self.target_yaw, state.yaw)
        alt_error = self.target_altitude - altitude

        output.aileron = self._clamp(self._roll_pid.compute(roll_error, dt), -1.0, 1.0)
        output.elevator = self._clamp(self._pitch_pid.compute(pitch_error, dt), -1.0, 1.0)
        output.rudder = self._clamp(self._yaw_pid.compute(yaw_error, dt), -1.0, 1.0)

        throttle_correction = self._altitude_pid.compute(alt_error, dt)
        base_throttle = 0.5 if self.mode != FlightMode.TAKEOFF else 0.7
        output.throttle = self._clamp(base_throttle + throttle_correction, 0.0, 1.0)

        self._update_mode(altitude)

        return output

    def emergency_stop(self) -> ControlOutput:
        """Immediately enter emergency mode and cut throttle."""
        self.mode = FlightMode.EMERGENCY
        self._reset_pids()
        return ControlOutput(throttle=0.0)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _yaw_error(target: float, current: float) -> float:
        """Shortest-path yaw error in degrees (-180 … +180)."""
        error = (target - current) % 360.0
        if error > 180.0:
            error -= 360.0
        return error

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _reset_pids(self) -> None:
        for pid in (self._roll_pid, self._pitch_pid, self._yaw_pid, self._altitude_pid):
            pid.reset()

    def _update_mode(self, altitude: float) -> None:
        """Automatically advance flight mode based on altitude."""
        if self.mode == FlightMode.TAKEOFF and altitude >= self.target_altitude * 0.95:
            self.mode = FlightMode.CRUISE
        elif self.mode == FlightMode.LAND and altitude <= 0.5:
            self.mode = FlightMode.GROUND
