"""Tests for the FlightController."""

import math
import pytest
from src.flight_controller import (
    AttitudeState,
    ControlOutput,
    FlightController,
    FlightMode,
    PIDState,
)


class TestPIDState:
    def test_proportional_term(self):
        pid = PIDState(kp=2.0, ki=0.0, kd=0.0)
        output = pid.compute(error=5.0, dt=0.1)
        assert output == pytest.approx(10.0)

    def test_integral_accumulates(self):
        pid = PIDState(kp=0.0, ki=1.0, kd=0.0, integral_limit=1000.0)
        pid.compute(error=1.0, dt=0.5)
        output = pid.compute(error=1.0, dt=0.5)
        assert output == pytest.approx(1.0)  # integral = 1.0, ki * integral = 1.0

    def test_integral_clamped(self):
        pid = PIDState(kp=0.0, ki=1.0, kd=0.0, integral_limit=5.0)
        for _ in range(100):
            pid.compute(error=1.0, dt=0.1)
        assert pid.integral == pytest.approx(5.0)

    def test_derivative_term(self):
        pid = PIDState(kp=0.0, ki=0.0, kd=1.0)
        pid.compute(error=0.0, dt=0.1)    # seed previous_error
        output = pid.compute(error=2.0, dt=0.1)
        assert output == pytest.approx(20.0)  # (2.0 - 0.0) / 0.1

    def test_zero_dt_returns_zero(self):
        pid = PIDState(kp=1.0, ki=1.0, kd=1.0)
        output = pid.compute(error=5.0, dt=0.0)
        assert output == 0.0

    def test_reset_clears_state(self):
        pid = PIDState(kp=0.0, ki=1.0, kd=1.0)
        pid.compute(error=3.0, dt=0.1)
        pid.reset()
        assert pid.integral == 0.0
        assert pid.previous_error == 0.0


class TestFlightController:
    def _make_controller(self) -> FlightController:
        fc = FlightController()
        fc.arm()
        return fc

    def test_initial_mode_is_ground(self):
        fc = FlightController()
        assert fc.mode == FlightMode.GROUND

    def test_arm_sets_takeoff_mode(self):
        fc = FlightController()
        fc.arm()
        assert fc.mode == FlightMode.TAKEOFF

    def test_arm_from_non_ground_raises(self):
        fc = FlightController()
        fc.arm()
        with pytest.raises(RuntimeError):
            fc.arm()

    def test_disarm_returns_to_ground(self):
        fc = self._make_controller()
        fc.disarm()
        assert fc.mode == FlightMode.GROUND

    def test_negative_altitude_target_raises(self):
        fc = self._make_controller()
        with pytest.raises(ValueError):
            fc.set_target_altitude(-10.0)

    def test_roll_target_clamped(self):
        fc = self._make_controller()
        fc.set_target_attitude(roll=200.0)
        assert fc.target_roll == FlightController.MAX_ROLL_DEG

    def test_pitch_target_clamped(self):
        fc = self._make_controller()
        fc.set_target_attitude(pitch=-200.0)
        assert fc.target_pitch == -FlightController.MAX_PITCH_DEG

    def test_yaw_target_normalised(self):
        fc = self._make_controller()
        fc.set_target_attitude(yaw=400.0)
        assert fc.target_yaw == pytest.approx(40.0)

    def test_update_returns_control_output(self):
        fc = self._make_controller()
        fc.set_target_altitude(50.0)
        output = fc.update(AttitudeState(), altitude=0.0, timestamp=0.1)
        assert isinstance(output, ControlOutput)

    def test_ground_mode_returns_zero_output(self):
        fc = FlightController()
        output = fc.update(AttitudeState(), altitude=0.0, timestamp=0.1)
        assert output.throttle == 0.0
        assert output.aileron == 0.0

    def test_emergency_stop_cuts_throttle(self):
        fc = self._make_controller()
        output = fc.emergency_stop()
        assert output.throttle == 0.0
        assert fc.mode == FlightMode.EMERGENCY

    def test_throttle_bounded(self):
        fc = self._make_controller()
        fc.set_target_altitude(1000.0)
        output = fc.update(AttitudeState(), altitude=0.0, timestamp=0.1)
        assert 0.0 <= output.throttle <= 1.0

    def test_aileron_bounded(self):
        fc = self._make_controller()
        fc.set_target_attitude(roll=45.0)
        output = fc.update(AttitudeState(roll=0.0), altitude=10.0, timestamp=0.1)
        assert -1.0 <= output.aileron <= 1.0

    def test_mode_advances_to_cruise_on_altitude(self):
        fc = self._make_controller()
        fc.set_target_altitude(50.0)
        # Simulate reaching 95% of target altitude
        fc.update(AttitudeState(), altitude=48.0, timestamp=0.1)
        assert fc.mode == FlightMode.CRUISE

    def test_yaw_error_wraps_correctly(self):
        assert FlightController._yaw_error(10.0, 350.0) == pytest.approx(20.0)
        assert FlightController._yaw_error(350.0, 10.0) == pytest.approx(-20.0)
