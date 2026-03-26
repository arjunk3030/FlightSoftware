"""
Flight Software — Entry Point

Demonstrates a minimal flight loop that ties together the flight controller,
navigation, telemetry logging, and safety monitoring subsystems.
"""

import time
from src.flight_controller import FlightController, AttitudeState, FlightMode
from src.navigation import Navigator, Waypoint
from src.safety import SafetyMonitor
from src.telemetry import TelemetryLogger, TelemetryRecord


# ---------------------------------------------------------------------------
# Simulated sensor helpers (replace with real hardware drivers)
# ---------------------------------------------------------------------------

def read_imu() -> AttitudeState:
    """Return attitude data from the IMU (stub)."""
    return AttitudeState(roll=0.0, pitch=2.0, yaw=45.0)


def read_altitude() -> float:
    """Return altitude in metres AGL from barometer / lidar (stub)."""
    return 0.0


def read_battery_voltage() -> float:
    """Return battery pack voltage (stub)."""
    return 12.4


def read_gps():
    """Return (latitude, longitude, groundspeed) from GPS (stub)."""
    return 37.7749, -122.4194, 0.0


def read_gps_satellites() -> int:
    """Return number of GPS satellites in view (stub)."""
    return 10


# ---------------------------------------------------------------------------
# Main flight loop
# ---------------------------------------------------------------------------

def run(loop_hz: float = 50.0, duration_s: float = 10.0) -> None:
    """
    Run the flight software main loop.

    Args:
        loop_hz:    Target control loop frequency in Hz.
        duration_s: How many seconds to run before landing.
    """
    dt = 1.0 / loop_hz

    controller = FlightController()
    navigator = Navigator()
    safety_monitor = SafetyMonitor()
    telemetry = TelemetryLogger()

    # --- Pre-flight checks ---
    report = safety_monitor.pre_flight(
        battery_voltage=read_battery_voltage(),
        gps_satellites=read_gps_satellites(),
        imu_ok=True,
        radio_ok=True,
    )
    print(report)

    if not report.is_safe_to_fly:
        print("Pre-flight checks FAILED — aborting.")
        return

    # --- Load a simple mission ---
    navigator.load_mission([
        Waypoint(latitude=37.7749, longitude=-122.4194, altitude=50.0, name="WP1"),
        Waypoint(latitude=37.7760, longitude=-122.4180, altitude=50.0, name="WP2"),
        Waypoint(latitude=37.7749, longitude=-122.4194, altitude=0.0,  name="HOME"),
    ])

    # --- Arm and take off ---
    controller.arm()
    controller.set_target_altitude(50.0)

    start = time.monotonic()
    print(f"Starting flight loop at {loop_hz} Hz for {duration_s}s …")

    while True:
        now = time.monotonic()
        elapsed = now - start

        # Read sensors
        state = read_imu()
        altitude = read_altitude()
        lat, lon, gspeed = read_gps()
        voltage = read_battery_voltage()

        # Safety monitoring
        safety_report = safety_monitor.monitor(altitude, state.roll, state.pitch, voltage)
        if not safety_report.is_safe_to_fly:
            print("EMERGENCY: safety violation detected.")
            controller.emergency_stop()
            break

        # Navigation guidance
        guidance = navigator.update(lat, lon, altitude)
        if guidance and guidance.mission_complete:
            print("Mission complete.")
            break

        # Flight control update
        output = controller.update(state, altitude, timestamp=now)

        # Telemetry
        telemetry.record(TelemetryRecord(
            timestamp=now,
            altitude=altitude,
            roll=state.roll,
            pitch=state.pitch,
            yaw=state.yaw,
            throttle=output.throttle,
            aileron=output.aileron,
            elevator=output.elevator,
            rudder=output.rudder,
            latitude=lat,
            longitude=lon,
            groundspeed=gspeed,
            battery_voltage=voltage,
            mode=controller.mode.value,
        ))

        # Transition to landing when time is up
        if elapsed >= duration_s and controller.mode != FlightMode.LAND:
            print("Time limit reached — initiating landing.")
            controller.mode = FlightMode.LAND
            controller.set_target_altitude(0.0)

        if controller.mode == FlightMode.GROUND:
            print("Aircraft on ground — shutting down.")
            break

        time.sleep(dt)

    print(telemetry.summary())
    telemetry.to_csv("/tmp/flight_telemetry.csv")
    print("Telemetry saved to /tmp/flight_telemetry.csv")


if __name__ == "__main__":
    run()
