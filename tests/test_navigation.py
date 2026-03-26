"""Tests for the Navigator."""

import math
import pytest
from src.navigation import (
    GuidanceCommand,
    Navigator,
    NavigationStatus,
    Waypoint,
    haversine_distance,
    initial_bearing,
)


class TestHaversineDistance:
    def test_same_point_is_zero(self):
        assert haversine_distance(0.0, 0.0, 0.0, 0.0) == pytest.approx(0.0)

    def test_known_distance(self):
        # London (51.5074° N, 0.1278° W) to Paris (48.8566° N, 2.3522° E)
        d = haversine_distance(51.5074, -0.1278, 48.8566, 2.3522)
        assert 340_000 < d < 345_000  # ~343 km

    def test_symmetry(self):
        d1 = haversine_distance(37.0, -122.0, 38.0, -121.0)
        d2 = haversine_distance(38.0, -121.0, 37.0, -122.0)
        assert d1 == pytest.approx(d2, rel=1e-9)


class TestInitialBearing:
    def test_due_north(self):
        bearing = initial_bearing(0.0, 0.0, 1.0, 0.0)
        assert bearing == pytest.approx(0.0, abs=0.01)

    def test_due_east(self):
        bearing = initial_bearing(0.0, 0.0, 0.0, 1.0)
        assert bearing == pytest.approx(90.0, abs=0.1)

    def test_due_south(self):
        bearing = initial_bearing(1.0, 0.0, 0.0, 0.0)
        assert bearing == pytest.approx(180.0, abs=0.01)

    def test_due_west(self):
        bearing = initial_bearing(0.0, 1.0, 0.0, 0.0)
        assert bearing == pytest.approx(270.0, abs=0.1)


class TestNavigator:
    def _simple_mission(self) -> list:
        return [
            Waypoint(latitude=0.0, longitude=0.0, altitude=50.0, name="WP1", radius=5.0),
            Waypoint(latitude=0.1, longitude=0.0, altitude=50.0, name="WP2", radius=5.0),
        ]

    def test_initial_status_is_idle(self):
        nav = Navigator()
        assert nav.status == NavigationStatus.IDLE

    def test_load_mission_sets_navigating(self):
        nav = Navigator()
        nav.load_mission(self._simple_mission())
        assert nav.status == NavigationStatus.NAVIGATING

    def test_empty_mission_raises(self):
        nav = Navigator()
        with pytest.raises(ValueError):
            nav.load_mission([])

    def test_current_waypoint_is_first(self):
        nav = Navigator()
        nav.load_mission(self._simple_mission())
        assert nav.current_waypoint.name == "WP1"

    def test_remaining_waypoints_count(self):
        nav = Navigator()
        nav.load_mission(self._simple_mission())
        assert nav.remaining_waypoints == 2

    def test_clear_mission_resets(self):
        nav = Navigator()
        nav.load_mission(self._simple_mission())
        nav.clear_mission()
        assert nav.status == NavigationStatus.IDLE
        assert nav.current_waypoint is None

    def test_update_returns_guidance_while_navigating(self):
        nav = Navigator()
        nav.load_mission(self._simple_mission())
        guidance = nav.update(latitude=0.05, longitude=0.05, altitude=50.0)
        assert isinstance(guidance, GuidanceCommand)
        assert guidance.distance_m > 0

    def test_reaching_waypoint_advances_index(self):
        nav = Navigator()
        wp = Waypoint(latitude=1.0, longitude=1.0, altitude=50.0, radius=100_000.0)  # huge radius
        nav.load_mission([wp, Waypoint(latitude=2.0, longitude=2.0, altitude=50.0)])
        guidance = nav.update(latitude=1.0, longitude=1.0, altitude=50.0)
        assert guidance.waypoint_reached
        assert nav.current_waypoint.latitude == pytest.approx(2.0)

    def test_completing_all_waypoints_sets_completed(self):
        nav = Navigator()
        wp = Waypoint(latitude=0.0, longitude=0.0, altitude=0.0, radius=1_000_000.0)
        nav.load_mission([wp])
        guidance = nav.update(latitude=0.0, longitude=0.0, altitude=0.0)
        assert guidance.mission_complete
        assert nav.status == NavigationStatus.COMPLETED

    def test_idle_navigator_returns_none(self):
        nav = Navigator()
        result = nav.update(latitude=0.0, longitude=0.0, altitude=0.0)
        assert result is None

    def test_add_waypoint(self):
        nav = Navigator()
        nav.add_waypoint(Waypoint(latitude=1.0, longitude=1.0))
        assert len(nav.waypoints) == 1
        assert nav.status == NavigationStatus.NAVIGATING
