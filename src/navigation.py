"""
Navigation Module

Manages a sequence of waypoints and provides bearing/distance guidance.
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


EARTH_RADIUS_M = 6_371_000.0  # mean radius in metres


@dataclass
class Waypoint:
    """A geographic position with an optional altitude constraint."""
    latitude: float           # decimal degrees
    longitude: float          # decimal degrees
    altitude: float = 0.0     # metres AGL
    name: str = ""
    radius: float = 10.0      # acceptance radius in metres

    def __str__(self) -> str:
        label = self.name or f"({self.latitude:.6f}, {self.longitude:.6f})"
        return f"Waypoint[{label} alt={self.altitude:.1f}m]"


class NavigationStatus(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    REACHED = "REACHED"
    COMPLETED = "COMPLETED"


class Navigator:
    """
    Waypoint-based navigation manager.

    Maintains an ordered list of waypoints and tracks progress through them.
    As the vehicle reaches each waypoint (within its acceptance radius) the
    navigator automatically advances to the next one.
    """

    def __init__(self) -> None:
        self._waypoints: List[Waypoint] = []
        self._current_index: int = 0
        self.status: NavigationStatus = NavigationStatus.IDLE

    # ------------------------------------------------------------------
    # Mission management
    # ------------------------------------------------------------------

    def load_mission(self, waypoints: List[Waypoint]) -> None:
        """Load an ordered list of waypoints as the current mission."""
        if not waypoints:
            raise ValueError("Waypoint list must not be empty.")
        self._waypoints = list(waypoints)
        self._current_index = 0
        self.status = NavigationStatus.NAVIGATING

    def clear_mission(self) -> None:
        """Remove all waypoints and reset status."""
        self._waypoints.clear()
        self._current_index = 0
        self.status = NavigationStatus.IDLE

    def add_waypoint(self, waypoint: Waypoint) -> None:
        """Append a waypoint to the end of the current mission."""
        self._waypoints.append(waypoint)
        if self.status == NavigationStatus.IDLE:
            self.status = NavigationStatus.NAVIGATING

    @property
    def waypoints(self) -> List[Waypoint]:
        return list(self._waypoints)

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Return the active target waypoint, or None if mission is done."""
        if self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None

    @property
    def remaining_waypoints(self) -> int:
        return max(0, len(self._waypoints) - self._current_index)

    # ------------------------------------------------------------------
    # Guidance
    # ------------------------------------------------------------------

    def update(self, latitude: float, longitude: float, altitude: float) -> Optional["GuidanceCommand"]:
        """
        Update the navigator with the current position and return a
        guidance command toward the active waypoint (or None if idle/done).

        The navigator advances to the next waypoint when the vehicle enters
        the acceptance radius of the current one.
        """
        if self.status in (NavigationStatus.IDLE, NavigationStatus.COMPLETED):
            return None

        wp = self.current_waypoint
        if wp is None:
            self.status = NavigationStatus.COMPLETED
            return None

        dist = haversine_distance(latitude, longitude, wp.latitude, wp.longitude)
        bearing = initial_bearing(latitude, longitude, wp.latitude, wp.longitude)
        alt_error = wp.altitude - altitude

        if dist <= wp.radius:
            self._current_index += 1
            if self._current_index >= len(self._waypoints):
                self.status = NavigationStatus.COMPLETED
                return GuidanceCommand(
                    distance_m=0.0,
                    bearing_deg=0.0,
                    altitude_error_m=0.0,
                    waypoint_reached=True,
                    mission_complete=True,
                )
            return GuidanceCommand(
                distance_m=dist,
                bearing_deg=bearing,
                altitude_error_m=alt_error,
                waypoint_reached=True,
                mission_complete=False,
            )

        return GuidanceCommand(
            distance_m=dist,
            bearing_deg=bearing,
            altitude_error_m=alt_error,
            waypoint_reached=False,
            mission_complete=False,
        )


@dataclass
class GuidanceCommand:
    """Navigation guidance output for the current update cycle."""
    distance_m: float          # metres to active waypoint
    bearing_deg: float         # degrees true to active waypoint
    altitude_error_m: float    # positive = need to climb
    waypoint_reached: bool = False
    mission_complete: bool = False


# ------------------------------------------------------------------
# Geodetic helpers
# ------------------------------------------------------------------

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Great-circle distance between two points on Earth using the Haversine formula.

    Args:
        lat1, lon1: Origin in decimal degrees.
        lat2, lon2: Destination in decimal degrees.

    Returns:
        Distance in metres.
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def initial_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Initial bearing from point 1 to point 2 (degrees true, 0–360).
    """
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)

    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0
