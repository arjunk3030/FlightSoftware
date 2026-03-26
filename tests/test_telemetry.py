"""Tests for the TelemetryLogger."""

import os
import json
import tempfile
import pytest
from src.telemetry import TelemetryLogger, TelemetryRecord


def _make_record(ts: float = 0.0, altitude: float = 100.0) -> TelemetryRecord:
    return TelemetryRecord(
        timestamp=ts,
        altitude=altitude,
        roll=1.0,
        pitch=2.0,
        yaw=45.0,
        throttle=0.6,
        aileron=0.1,
        elevator=0.0,
        rudder=-0.05,
        latitude=37.77,
        longitude=-122.42,
        groundspeed=15.0,
        battery_voltage=12.3,
        mode="CRUISE",
    )


class TestTelemetryRecord:
    def test_to_dict_contains_all_fields(self):
        rec = _make_record()
        d = rec.to_dict()
        assert "timestamp" in d
        assert "altitude" in d
        assert "mode" in d


class TestTelemetryLogger:
    def test_empty_logger_has_no_records(self):
        logger = TelemetryLogger()
        assert logger.record_count == 0
        assert logger.latest() is None

    def test_record_increases_count(self):
        logger = TelemetryLogger()
        logger.record(_make_record(ts=1.0))
        assert logger.record_count == 1

    def test_latest_returns_last_record(self):
        logger = TelemetryLogger()
        logger.record(_make_record(ts=1.0, altitude=50.0))
        logger.record(_make_record(ts=2.0, altitude=100.0))
        assert logger.latest().altitude == 100.0

    def test_max_records_evicts_oldest(self):
        logger = TelemetryLogger(max_records=3)
        for i in range(5):
            logger.record(_make_record(ts=float(i)))
        assert logger.record_count == 3
        assert logger.all_records()[0].timestamp == 2.0

    def test_clear_removes_all(self):
        logger = TelemetryLogger()
        logger.record(_make_record())
        logger.clear()
        assert logger.record_count == 0

    def test_summary_returns_expected_keys(self):
        logger = TelemetryLogger()
        logger.record(_make_record(ts=0.0, altitude=10.0))
        logger.record(_make_record(ts=5.0, altitude=20.0))
        s = logger.summary()
        assert "max_altitude_m" in s
        assert "duration_s" in s
        assert s["duration_s"] == pytest.approx(5.0)

    def test_empty_summary(self):
        logger = TelemetryLogger()
        assert logger.summary() == {}

    def test_to_csv_and_from_csv_roundtrip(self):
        logger = TelemetryLogger()
        for i in range(5):
            logger.record(_make_record(ts=float(i), altitude=float(i * 10)))

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "telemetry.csv")
            logger.to_csv(filepath)
            assert os.path.exists(filepath)

            loaded = TelemetryLogger.from_csv(filepath)
            assert loaded.record_count == 5
            assert loaded.latest().altitude == pytest.approx(40.0)

    def test_to_json(self):
        logger = TelemetryLogger()
        logger.record(_make_record(ts=1.0))

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "telemetry.json")
            logger.to_json(filepath)
            with open(filepath) as fh:
                data = json.load(fh)
            assert len(data) == 1
            assert data[0]["altitude"] == pytest.approx(100.0)

    def test_all_records_returns_copy(self):
        logger = TelemetryLogger()
        logger.record(_make_record())
        records = logger.all_records()
        records.clear()
        assert logger.record_count == 1
