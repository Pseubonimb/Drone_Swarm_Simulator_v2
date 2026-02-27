"""
Square formation: drones fly a square pattern with PID braking.
Imports PIDRegulator, VelocityMonitor, send_rc_override from core.
"""
import os
import sys

# Ensure project root is on path when run as script
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import threading
import time
from datetime import datetime

from pymavlink import mavutil

from core.control.pid import PIDRegulator
from core.mavlink.utils import RC_NEUTRAL, send_rc_override
from core.monitors.velocity_monitor import VelocityMonitor

# Square step commands (roll, pitch, throttle, yaw)
SQUARE_STEPS = [
    (RC_NEUTRAL, 1700, RC_NEUTRAL, RC_NEUTRAL),  # Back
    (1700, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL),  # Right
    (RC_NEUTRAL, 1300, RC_NEUTRAL, RC_NEUTRAL),  # Forward
    (1300, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL),  # Left
]
STEP_DURATION = 2  # seconds per segment


class DroneController:
    """Controller for one drone: connection, velocity monitor, PID braking."""

    def __init__(self, config: dict, logging_enabled: bool = False) -> None:
        self.config = config
        self.master = None
        self.velocity_monitor = None
        self.lock = threading.Lock()
        self.logging_enabled = logging_enabled
        if self.logging_enabled:
            self.logIter = 0
            os.makedirs("logs_SQUARE_two_drones", exist_ok=True)
            self.logfile = open(
                f"logs_SQUARE_two_drones/drone_{self.config['id']}_log.txt", "w"
            )
        self.roll_velocity_pid = PIDRegulator(
            kp=100.0,
            ki=100.0,
            kd=0.0,
            integral_limit=200.0,
            output_limit=500.0,
        )
        self.pitch_velocity_pid = PIDRegulator(
            kp=100.0,
            ki=0.0,
            kd=0.0,
            integral_limit=200.0,
            output_limit=500.0,
        )
        self.last_movement_mode = None
        self.last_rc_channels = {
            "roll": RC_NEUTRAL,
            "pitch": RC_NEUTRAL,
            "throttle": RC_NEUTRAL,
            "yaw": RC_NEUTRAL,
        }
        self.rc_channels_lock = threading.Lock()

    def connect(self) -> None:
        self.master = mavutil.mavlink_connection(
            f'udp:127.0.0.1:{self.config["udp_port"]}'
        )
        self.master.wait_heartbeat()
        self.velocity_monitor = VelocityMonitor(self.master)

    def initialize(self) -> None:
        self.master.set_mode(4)  # GUIDED
        time.sleep(0.5)
        self.master.arducopter_arm()
        time.sleep(2)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, 1,
        )
        time.sleep(5)
        self.master.set_mode(2)  # ALT_HOLD
        send_rc_override(
            self.master,
            RC_NEUTRAL,
            RC_NEUTRAL,
            RC_NEUTRAL,
            RC_NEUTRAL,
            controller=self,
        )
        time.sleep(0.5)
        self.velocity_monitor.start()

    def start_rc_keepalive(self) -> None:
        def keepalive_loop() -> None:
            while True:
                try:
                    with self.rc_channels_lock:
                        roll = self.last_rc_channels["roll"]
                        pitch = self.last_rc_channels["pitch"]
                        throttle = self.last_rc_channels["throttle"]
                        yaw = self.last_rc_channels["yaw"]
                    send_rc_override(
                        self.master, roll, pitch, throttle, yaw, controller=self
                    )
                    time.sleep(0.2)
                except Exception:
                    break

        threading.Thread(target=keepalive_loop, daemon=True).start()

    def move_with_pid_braking(
        self,
        direction: tuple,
        kpx: float = 2000,
        kpy: float = 2000,
        move_duration: float = 5,
        target_velocity: float = 0.0,
        max_brake_time: float = 10.0,
        velocity_threshold: float = 0.8,
        use_pid: bool = True,
    ) -> None:
        roll, pitch, throttle, yaw = direction
        if self.last_movement_mode != "moving":
            self.roll_velocity_pid.reset()
            self.pitch_velocity_pid.reset()
        self.last_movement_mode = "moving"
        start = time.time()
        while time.time() - start < move_duration:
            send_rc_override(
                self.master, roll, pitch, throttle, yaw, controller=self
            )
            time.sleep(0.1)
        self.last_movement_mode = "braking"
        start_time = time.time()
        last_update_time = time.time()
        while time.time() - start_time < max_brake_time:
            current_time = time.time()
            dt = current_time - last_update_time
            if dt <= 0:
                dt = 0.05
            last_update_time = current_time
            current_velocity = self.velocity_monitor.get_velocity()
            current_x_velocity = current_velocity["vx"]
            current_y_velocity = current_velocity["vy"]
            if (
                abs(current_x_velocity) < velocity_threshold
                and abs(current_y_velocity) < velocity_threshold
            ):
                self.roll_velocity_pid.reset()
                self.pitch_velocity_pid.reset()
                break
            error_vx = target_velocity - current_x_velocity
            error_vy = target_velocity - current_y_velocity
            if use_pid:
                brake_output_pitch = self.pitch_velocity_pid.update(
                    error_vx, dt
                )
                brake_intensity_x = int(abs(brake_output_pitch))
                brake_intensity_x = min(500, max(50, brake_intensity_x))
                brake_direction_x = -1 if error_vx > 0 else 1
                brake_pitch = RC_NEUTRAL + (
                    brake_direction_x * brake_intensity_x
                )
                brake_output_roll = self.roll_velocity_pid.update(error_vy, dt)
                brake_intensity_y = int(abs(brake_output_roll))
                brake_intensity_y = min(500, max(50, brake_intensity_y))
                brake_direction_y = 1 if error_vy > 0 else -1
                brake_roll = RC_NEUTRAL + (
                    brake_direction_y * brake_intensity_y
                )
            else:
                if kpx > 0:
                    brake_intensity_x = int(abs(error_vx) * kpx)
                    brake_intensity_x = min(500, max(50, brake_intensity_x))
                    brake_direction_x = 1 if error_vx > 0 else -1
                    brake_pitch = RC_NEUTRAL + (
                        brake_direction_x * brake_intensity_x
                    )
                else:
                    brake_pitch = RC_NEUTRAL
                if kpy > 0:
                    brake_intensity_y = int(abs(error_vy) * kpy)
                    brake_intensity_y = min(500, max(50, brake_intensity_y))
                    brake_direction_y = -1 if error_vy > 0 else 1
                    brake_roll = RC_NEUTRAL + (
                        brake_direction_y * brake_intensity_y
                    )
                else:
                    brake_roll = RC_NEUTRAL
            send_rc_override(
                self.master, brake_roll, brake_pitch, RC_NEUTRAL, RC_NEUTRAL,
                controller=self,
            )
            time.sleep(0.05)

    def stop(self) -> None:
        send_rc_override(
            self.master,
            RC_NEUTRAL,
            RC_NEUTRAL,
            RC_NEUTRAL,
            RC_NEUTRAL,
            controller=self,
        )
        self.velocity_monitor.stop()
        if self.logging_enabled:
            self.logfile.close()
        self.master.set_mode(9)  # LAND


def square_pattern(
    controller: DroneController,
    step_duration: float = 2,
    kpx: float = 100,
    kpy: float = 100,
    use_pid: bool = True,
    velocity_threshold: float = 0.2,
) -> None:
    """Run square pattern with PID braking."""
    while True:
        for step in SQUARE_STEPS:
            controller.move_with_pid_braking(
                step,
                kpx=kpx,
                kpy=kpy,
                move_duration=step_duration,
                target_velocity=0.0,
                max_brake_time=5.0,
                velocity_threshold=velocity_threshold,
                use_pid=use_pid,
            )
            send_rc_override(
                controller.master,
                RC_NEUTRAL,
                RC_NEUTRAL,
                RC_NEUTRAL,
                RC_NEUTRAL,
                controller=controller,
            )
            time.sleep(1)


def initialize_drone_parallel(
    controller: DroneController, init_barrier: threading.Barrier
) -> None:
    try:
        controller.connect()
        controller.initialize()
        controller.start_rc_keepalive()
        init_barrier.wait()
    except Exception:
        import traceback
        traceback.print_exc()
        raise


def main() -> None:
    DRONES_CONFIG = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "square"}
        for i in range(5)
    ]
    controllers = []
    for config in DRONES_CONFIG:
        controllers.append(DroneController(config, logging_enabled=True))
    init_barrier = threading.Barrier(len(controllers) + 1)
    for controller in controllers:
        threading.Thread(
            target=initialize_drone_parallel,
            args=(controller, init_barrier),
            daemon=False,
        ).start()
    try:
        init_barrier.wait(timeout=30)
    except threading.BrokenBarrierError:
        return
    time.sleep(2)
    try:
        square_threads = []
        for controller in controllers:
            t = threading.Thread(
                target=square_pattern,
                args=(controller, STEP_DURATION, 100, 100, True),
                kwargs={"velocity_threshold": 0.2},
                daemon=True,
            )
            t.start()
            square_threads.append(t)
        while True:
            for i, c in enumerate(controllers):
                if c.logging_enabled and hasattr(c, "logfile"):
                    vel = c.velocity_monitor.get_velocity()
                    now = datetime.now()
                    ts = f"{now.second:02d}.{now.microsecond // 1000:03d}"
                    c.logfile.write(f"'t': {ts}, {vel}\n")
            time.sleep(0.0025)
    except KeyboardInterrupt:
        for controller in controllers:
            controller.stop()
            controller.master.set_mode(9)


if __name__ == "__main__":
    main()
