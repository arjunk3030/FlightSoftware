"""
Telemetry Module

Collects, stores and exports flight telemetry data.
"""

import csv
import json
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional


@dataclass
class TelemetryRecord:
    """A single snapshot of aircraft state at a given timestamp."""
    timestamp: float
    altitude: float              # metres AGL
    roll: float                  # degrees
    pitch: float                 # degrees
    yaw: float                   # degrees
    throttle: float              # 0.0 – 1.0
    aileron: float               # -1.0 – 1.0
    elevator: float              # -1.0 – 1.0
    rudder: float                # -1.0 – 1.0
    latitude: float = 0.0        # decimal degrees
    longitude: float = 0.0       # decimal degrees
    groundspeed: float = 0.0     # m/s
    battery_voltage: float = 0.0 # volts
    mode: str = "UNKNOWN"

    def to_dict(self) -> Dict:
        return asdict(self)


class TelemetryLogger:
    """
    Records telemetry data in memory and can flush it to disk as CSV or JSON.

    Args:
        max_records: Maximum number of records to keep in memory (oldest are dropped).
    """

    CSV_FIELDNAMES = [
        "timestamp", "altitude", "roll", "pitch", "yaw",
        "throttle", "aileron", "elevator", "rudder",
        "latitude", "longitude", "groundspeed", "battery_voltage", "mode",
    ]

    def __init__(self, max_records: int = 10_000) -> None:
        self._records: List[TelemetryRecord] = []
        self.max_records = max_records
        self._start_time: float = time.monotonic()

    # ------------------------------------------------------------------
    # Data ingestion
    # ------------------------------------------------------------------

    def record(self, record: TelemetryRecord) -> None:
        """Append a telemetry record; evict oldest if capacity exceeded."""
        if len(self._records) >= self.max_records:
            self._records.pop(0)
        self._records.append(record)

    def latest(self) -> Optional[TelemetryRecord]:
        """Return the most recently recorded entry, or None."""
        return self._records[-1] if self._records else None

    def all_records(self) -> List[TelemetryRecord]:
        """Return a copy of all stored records."""
        return list(self._records)

    def clear(self) -> None:
        """Remove all stored records."""
        self._records.clear()

    @property
    def record_count(self) -> int:
        return len(self._records)

    # ------------------------------------------------------------------
    # Export helpers
    # ------------------------------------------------------------------

    def to_csv(self, filepath: str) -> None:
        """Write all records to a CSV file."""
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(filepath, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=self.CSV_FIELDNAMES)
            writer.writeheader()
            for rec in self._records:
                writer.writerow(rec.to_dict())

    def to_json(self, filepath: str) -> None:
        """Write all records to a JSON file."""
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(filepath, "w", encoding="utf-8") as fh:
            json.dump([rec.to_dict() for rec in self._records], fh, indent=2)

    @classmethod
    def from_csv(cls, filepath: str, max_records: int = 10_000) -> "TelemetryLogger":
        """Load records from a previously saved CSV file."""
        logger = cls(max_records=max_records)
        with open(filepath, newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                rec = TelemetryRecord(
                    timestamp=float(row["timestamp"]),
                    altitude=float(row["altitude"]),
                    roll=float(row["roll"]),
                    pitch=float(row["pitch"]),
                    yaw=float(row["yaw"]),
                    throttle=float(row["throttle"]),
                    aileron=float(row["aileron"]),
                    elevator=float(row["elevator"]),
                    rudder=float(row["rudder"]),
                    latitude=float(row["latitude"]),
                    longitude=float(row["longitude"]),
                    groundspeed=float(row["groundspeed"]),
                    battery_voltage=float(row["battery_voltage"]),
                    mode=row["mode"],
                )
                logger.record(rec)
        return logger

    # ------------------------------------------------------------------
    # Summary statistics
    # ------------------------------------------------------------------

    def summary(self) -> Dict:
        """Return a basic statistics summary of recorded data."""
        if not self._records:
            return {}

        altitudes = [r.altitude for r in self._records]
        groundspeeds = [r.groundspeed for r in self._records]
        duration = self._records[-1].timestamp - self._records[0].timestamp

        return {
            "record_count": len(self._records),
            "duration_s": round(duration, 3),
            "max_altitude_m": round(max(altitudes), 3),
            "min_altitude_m": round(min(altitudes), 3),
            "avg_altitude_m": round(sum(altitudes) / len(altitudes), 3),
            "max_groundspeed_ms": round(max(groundspeeds), 3),
        }
