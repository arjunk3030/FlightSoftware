"""
Safety Module

Pre-flight checks and in-flight safety monitoring.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class SafetyLevel(Enum):
    OK = "OK"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


@dataclass
class SafetyCheck:
    """Result of a single safety assessment."""
    name: str
    level: SafetyLevel
    message: str = ""

    def passed(self) -> bool:
        return self.level == SafetyLevel.OK


@dataclass
class SafetyReport:
    """Aggregated safety report from all checks."""
    checks: List[SafetyCheck] = field(default_factory=list)

    def add(self, check: SafetyCheck) -> None:
        self.checks.append(check)

    @property
    def overall_level(self) -> SafetyLevel:
        if any(c.level == SafetyLevel.CRITICAL for c in self.checks):
            return SafetyLevel.CRITICAL
        if any(c.level == SafetyLevel.WARNING for c in self.checks):
            return SafetyLevel.WARNING
        return SafetyLevel.OK

    @property
    def is_safe_to_fly(self) -> bool:
        return self.overall_level != SafetyLevel.CRITICAL

    def failed_checks(self) -> List[SafetyCheck]:
        return [c for c in self.checks if not c.passed()]

    def __str__(self) -> str:
        lines = [f"Safety Report — {self.overall_level.value}"]
        for check in self.checks:
            lines.append(f"  [{check.level.value:8s}] {check.name}: {check.message}")
        return "\n".join(lines)


class SafetyMonitor:
    """
    Performs pre-flight and in-flight safety checks.

    All thresholds can be customised at construction time.
    """

    def __init__(
        self,
        min_battery_voltage: float = 10.5,
        warn_battery_voltage: float = 11.0,
        max_roll_deg: float = 60.0,
        max_pitch_deg: float = 45.0,
        max_altitude_m: float = 400.0,
        warn_altitude_m: float = 350.0,
        min_gps_satellites: int = 6,
    ) -> None:
        self.min_battery_voltage = min_battery_voltage
        self.warn_battery_voltage = warn_battery_voltage
        self.max_roll_deg = max_roll_deg
        self.max_pitch_deg = max_pitch_deg
        self.max_altitude_m = max_altitude_m
        self.warn_altitude_m = warn_altitude_m
        self.min_gps_satellites = min_gps_satellites

    # ------------------------------------------------------------------
    # Pre-flight checks
    # ------------------------------------------------------------------

    def pre_flight(
        self,
        battery_voltage: float,
        gps_satellites: int,
        imu_ok: bool,
        radio_ok: bool,
    ) -> SafetyReport:
        """Run all pre-flight safety checks and return a SafetyReport."""
        report = SafetyReport()

        report.add(self._check_battery(battery_voltage))
        report.add(self._check_gps(gps_satellites))
        report.add(self._check_imu(imu_ok))
        report.add(self._check_radio(radio_ok))

        return report

    # ------------------------------------------------------------------
    # In-flight monitoring
    # ------------------------------------------------------------------

    def monitor(
        self,
        altitude: float,
        roll: float,
        pitch: float,
        battery_voltage: float,
    ) -> SafetyReport:
        """Evaluate in-flight safety constraints and return a SafetyReport."""
        report = SafetyReport()

        report.add(self._check_altitude(altitude))
        report.add(self._check_roll(roll))
        report.add(self._check_pitch(pitch))
        report.add(self._check_battery(battery_voltage))

        return report

    # ------------------------------------------------------------------
    # Individual checks
    # ------------------------------------------------------------------

    def _check_battery(self, voltage: float) -> SafetyCheck:
        if voltage < self.min_battery_voltage:
            return SafetyCheck(
                "Battery",
                SafetyLevel.CRITICAL,
                f"Voltage {voltage:.2f}V below minimum {self.min_battery_voltage:.2f}V — land immediately.",
            )
        if voltage < self.warn_battery_voltage:
            return SafetyCheck(
                "Battery",
                SafetyLevel.WARNING,
                f"Voltage {voltage:.2f}V low (warn threshold {self.warn_battery_voltage:.2f}V).",
            )
        return SafetyCheck("Battery", SafetyLevel.OK, f"Voltage {voltage:.2f}V nominal.")

    def _check_gps(self, satellites: int) -> SafetyCheck:
        if satellites < self.min_gps_satellites:
            return SafetyCheck(
                "GPS",
                SafetyLevel.CRITICAL,
                f"Only {satellites} satellites (need ≥ {self.min_gps_satellites}).",
            )
        return SafetyCheck("GPS", SafetyLevel.OK, f"{satellites} satellites acquired.")

    def _check_imu(self, imu_ok: bool) -> SafetyCheck:
        if not imu_ok:
            return SafetyCheck("IMU", SafetyLevel.CRITICAL, "IMU health check failed.")
        return SafetyCheck("IMU", SafetyLevel.OK, "IMU healthy.")

    def _check_radio(self, radio_ok: bool) -> SafetyCheck:
        if not radio_ok:
            return SafetyCheck("Radio", SafetyLevel.CRITICAL, "No RC / datalink signal.")
        return SafetyCheck("Radio", SafetyLevel.OK, "Radio link nominal.")

    def _check_altitude(self, altitude: float) -> SafetyCheck:
        if altitude > self.max_altitude_m:
            return SafetyCheck(
                "Altitude",
                SafetyLevel.CRITICAL,
                f"Altitude {altitude:.1f}m exceeds limit {self.max_altitude_m:.1f}m.",
            )
        if altitude > self.warn_altitude_m:
            return SafetyCheck(
                "Altitude",
                SafetyLevel.WARNING,
                f"Altitude {altitude:.1f}m approaching limit {self.max_altitude_m:.1f}m.",
            )
        return SafetyCheck("Altitude", SafetyLevel.OK, f"Altitude {altitude:.1f}m nominal.")

    def _check_roll(self, roll: float) -> SafetyCheck:
        if abs(roll) > self.max_roll_deg:
            return SafetyCheck(
                "Roll",
                SafetyLevel.CRITICAL,
                f"Roll {roll:.1f}° exceeds limit ±{self.max_roll_deg:.1f}°.",
            )
        return SafetyCheck("Roll", SafetyLevel.OK, f"Roll {roll:.1f}° nominal.")

    def _check_pitch(self, pitch: float) -> SafetyCheck:
        if abs(pitch) > self.max_pitch_deg:
            return SafetyCheck(
                "Pitch",
                SafetyLevel.CRITICAL,
                f"Pitch {pitch:.1f}° exceeds limit ±{self.max_pitch_deg:.1f}°.",
            )
        return SafetyCheck("Pitch", SafetyLevel.OK, f"Pitch {pitch:.1f}° nominal.")
