"""Tests for the SafetyMonitor."""

import pytest
from src.safety import SafetyCheck, SafetyLevel, SafetyMonitor, SafetyReport


class TestSafetyCheck:
    def test_ok_check_passes(self):
        check = SafetyCheck("Test", SafetyLevel.OK, "all good")
        assert check.passed()

    def test_warning_check_does_not_pass(self):
        check = SafetyCheck("Test", SafetyLevel.WARNING, "low voltage")
        assert not check.passed()

    def test_critical_check_does_not_pass(self):
        check = SafetyCheck("Test", SafetyLevel.CRITICAL, "critical failure")
        assert not check.passed()


class TestSafetyReport:
    def test_empty_report_is_ok(self):
        report = SafetyReport()
        assert report.overall_level == SafetyLevel.OK
        assert report.is_safe_to_fly

    def test_single_warning_makes_report_warning(self):
        report = SafetyReport()
        report.add(SafetyCheck("A", SafetyLevel.OK))
        report.add(SafetyCheck("B", SafetyLevel.WARNING))
        assert report.overall_level == SafetyLevel.WARNING
        assert report.is_safe_to_fly

    def test_single_critical_makes_report_critical(self):
        report = SafetyReport()
        report.add(SafetyCheck("A", SafetyLevel.WARNING))
        report.add(SafetyCheck("B", SafetyLevel.CRITICAL))
        assert report.overall_level == SafetyLevel.CRITICAL
        assert not report.is_safe_to_fly

    def test_failed_checks_lists_non_ok(self):
        report = SafetyReport()
        report.add(SafetyCheck("A", SafetyLevel.OK))
        report.add(SafetyCheck("B", SafetyLevel.WARNING))
        report.add(SafetyCheck("C", SafetyLevel.CRITICAL))
        failed = report.failed_checks()
        assert len(failed) == 2
        assert all(c.name in ("B", "C") for c in failed)

    def test_str_contains_check_names(self):
        report = SafetyReport()
        report.add(SafetyCheck("Battery", SafetyLevel.OK, "Nominal"))
        s = str(report)
        assert "Battery" in s


class TestSafetyMonitor:
    def _ok_monitor(self) -> SafetyMonitor:
        return SafetyMonitor()

    # ------------------------------------------------------------------ pre-flight

    def test_all_good_pre_flight(self):
        monitor = self._ok_monitor()
        report = monitor.pre_flight(
            battery_voltage=12.4,
            gps_satellites=10,
            imu_ok=True,
            radio_ok=True,
        )
        assert report.is_safe_to_fly
        assert report.overall_level == SafetyLevel.OK

    def test_low_battery_critical(self):
        monitor = self._ok_monitor()
        report = monitor.pre_flight(
            battery_voltage=9.0,
            gps_satellites=10,
            imu_ok=True,
            radio_ok=True,
        )
        assert not report.is_safe_to_fly

    def test_warn_battery(self):
        monitor = SafetyMonitor(min_battery_voltage=10.5, warn_battery_voltage=11.5)
        report = monitor.pre_flight(
            battery_voltage=11.0,
            gps_satellites=10,
            imu_ok=True,
            radio_ok=True,
        )
        assert report.overall_level == SafetyLevel.WARNING
        assert report.is_safe_to_fly

    def test_insufficient_gps_critical(self):
        monitor = self._ok_monitor()
        report = monitor.pre_flight(
            battery_voltage=12.4,
            gps_satellites=3,
            imu_ok=True,
            radio_ok=True,
        )
        assert not report.is_safe_to_fly

    def test_imu_fail_critical(self):
        monitor = self._ok_monitor()
        report = monitor.pre_flight(
            battery_voltage=12.4,
            gps_satellites=10,
            imu_ok=False,
            radio_ok=True,
        )
        assert not report.is_safe_to_fly

    def test_radio_fail_critical(self):
        monitor = self._ok_monitor()
        report = monitor.pre_flight(
            battery_voltage=12.4,
            gps_satellites=10,
            imu_ok=True,
            radio_ok=False,
        )
        assert not report.is_safe_to_fly

    # ------------------------------------------------------------------ in-flight

    def test_nominal_inflight(self):
        monitor = self._ok_monitor()
        report = monitor.monitor(altitude=100.0, roll=5.0, pitch=3.0, battery_voltage=12.0)
        assert report.is_safe_to_fly

    def test_excess_altitude_critical(self):
        monitor = SafetyMonitor(max_altitude_m=400.0)
        report = monitor.monitor(altitude=500.0, roll=0.0, pitch=0.0, battery_voltage=12.0)
        assert not report.is_safe_to_fly

    def test_approaching_altitude_warning(self):
        monitor = SafetyMonitor(max_altitude_m=400.0, warn_altitude_m=350.0)
        report = monitor.monitor(altitude=360.0, roll=0.0, pitch=0.0, battery_voltage=12.0)
        assert report.overall_level == SafetyLevel.WARNING

    def test_excess_roll_critical(self):
        monitor = SafetyMonitor(max_roll_deg=60.0)
        report = monitor.monitor(altitude=50.0, roll=75.0, pitch=0.0, battery_voltage=12.0)
        assert not report.is_safe_to_fly

    def test_excess_pitch_critical(self):
        monitor = SafetyMonitor(max_pitch_deg=45.0)
        report = monitor.monitor(altitude=50.0, roll=0.0, pitch=60.0, battery_voltage=12.0)
        assert not report.is_safe_to_fly
