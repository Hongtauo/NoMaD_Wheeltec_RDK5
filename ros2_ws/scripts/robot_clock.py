"""Session-scoped, fail-closed clock preparation for the WheelTec robot.

This module does nothing on import.  A remote operator owns the session, keeps it
alive while its ROS children run, and calls ``restore`` only after they stop.
The operator measures the PC/robot offset and supplies its conservative error
bound to ``verify``; the robot cannot independently know the PC's clock.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import platform
import re
import stat
import subprocess
import time
import uuid

try:
    import fcntl
except ImportError:  # Importable for mocked tests on Windows; never operational.
    fcntl = None


ROBOT_IP = "192.168.0.100"
LOCK_PATH = Path("/home/wheeltec/nomad_live_control/.operator.lock")
CLOCK_UNITS = (
    "ntp.service",
    "ntpsec.service",
    "chrony.service",
    "systemd-timesyncd.service",
)
MIN_EPOCH_NS = 1704067200 * 1_000_000_000  # 2024-01-01 UTC
MAX_EPOCH_NS = 4133980800 * 1_000_000_000  # 2101-01-01 UTC, exclusive


class RobotClockError(RuntimeError):
    """An operation was refused or could not be completed safely."""


def _current_uid() -> int:
    return os.getuid()


def _read_text(path: str | Path) -> str:
    return Path(path).read_text(encoding="utf-8").strip()


def _is_clock_process(comm: str, args: str) -> bool:
    return comm in {
        "ntpd", "ntpsec", "ntpdate", "sntp", "chronyd", "openntpd", "systemd-timesyn",
        "systemd-timesyncd",
    } or bool(re.search(r"(?:^|[ /])(?:ntpd|chronyd)(?:\s|$)", args))


def _is_ros_process(comm: str, args: str) -> bool:
    # Only the ROS discovery daemon is exempt.  A CLI that launches hardware,
    # generic component containers, and native/Python nodes are not exempt.
    if (
        "ros2cli.daemon.daemonize" in args
        or re.search(r"(?:^|[/\s])_ros2_daemon(?:\s|$)", args)
    ):
        return False
    terms = (
        "--ros-args", "/opt/ros/", "/opt/tros/", "wheeltec_ros2/",
        "nomad_navigation", "robot_control_node", "robot_gateway",
        "component_container", "robot_state_publisher",
        "static_transform_publisher", "controller_server", "planner_server",
        "behavior_server", "bt_navigator", "velocity_smoother",
        "lifecycle_manager", "map_server", "amcl", "slam_toolbox", "rviz2",
        "wheeltec_robot", "wheeltec_camera", "astra_camera", "orbbec_camera",
        "realsense2_camera", "v4l2_camera", "usb_cam", "ydlidar", "rplidar",
        "sllidar", "lslidar", "ydlaser", "depthimage_to_laserscan",
    )
    return bool(re.search(r"(?:^|[/\s])ros2(?:\s|$)", args)) or any(
        term in args or term in comm for term in terms
    )


class RobotClockSession:
    """Hold a robot-exclusive lock and suspend only active clock services.

    Passwords stay in memory and travel to sudo only over its standard input.
    Returned dictionaries and exception messages never contain command output,
    arbitrary process arguments, or the password.
    """

    def __init__(self, sudo_password: str, *, transport: str = 'dds'):
        if not isinstance(sudo_password, str) or not sudo_password or any(
            char in sudo_password for char in "\r\n\x00"
        ):
            raise RobotClockError("A nonempty, single-line sudo password is required")
        self._sudo_password = sudo_password
        if transport not in ('dds', 'api'):
            raise RobotClockError('Unknown robot transport')
        self.transport = transport
        self.robot_identity = None
        self.session_id = str(uuid.uuid4())
        self.phase = "new"
        self._lock_fd: int | None = None
        self._original_units: dict[str, dict] = {}
        self._stopped_units: list[str] = []
        self._timezone: str | None = None
        self._last_restore_result: dict | None = None

    @staticmethod
    def _run(command: list[str], *, input_text: str | None = None):
        try:
            result = subprocess.run(
                command, input=input_text, capture_output=True, text=True,
                timeout=20, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            raise RobotClockError("Required robot command failed or timed out") from None
        if result.returncode:
            raise RobotClockError("Required robot command returned an error")
        return result

    def _privileged(self, command: list[str]):
        if not self._sudo_password:
            raise RobotClockError("This clock session has already been closed")
        return self._run(
            ["sudo", "-S", "-p", "", "--", *command],
            input_text=self._sudo_password + "\n",
        )

    def _check_identity(self) -> None:
        if self.transport == 'api':
            from nomad_navigation.robot_identity import (require_robot_identity,
                                                        validate_api_environment)
            try:
                validate_api_environment(os.environ, platform.machine())
                self.robot_identity = require_robot_identity()
            except (OSError, ValueError, subprocess.SubprocessError):
                raise RobotClockError('Enrolled campus robot identity/environment validation failed') from None
            return
        if platform.machine().lower() not in {"aarch64", "arm64"}:
            raise RobotClockError("Clock preparation is restricted to the ARM64 robot")
        if _current_uid() == 0:
            raise RobotClockError("Run the operator as wheeltec, not as root")
        try:
            interfaces = json.loads(self._run(["ip", "-j", "-4", "address", "show"]).stdout)
            addresses = {
                address.get("local")
                for interface in interfaces
                for address in interface.get("addr_info", [])
            }
        except (ValueError, TypeError, AttributeError):
            raise RobotClockError("Cannot validate the robot's local IPv4 addresses") from None
        if ROBOT_IP not in addresses:
            raise RobotClockError("Required robot IPv4 address is not locally assigned")

    def _acquire_lock(self) -> None:
        if fcntl is None:
            raise RobotClockError("POSIX advisory locks are required on the robot")
        try:
            flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0)
            lock_fd = os.open(LOCK_PATH, flags, 0o600)
        except OSError:
            raise RobotClockError("Cannot open the robot operator lock") from None
        try:
            metadata = os.fstat(lock_fd)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != _current_uid():
                raise RobotClockError("The operator lock must be a regular file owned by the operator")
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.fchmod(lock_fd, 0o600)
        except (OSError, RobotClockError):
            os.close(lock_fd)
            raise RobotClockError("Another operator is running, or the lock is unsafe") from None
        self._lock_fd = lock_fd

    def _release_lock(self) -> None:
        if self._lock_fd is not None:
            descriptor, self._lock_fd = self._lock_fd, None
            try:
                fcntl.flock(descriptor, fcntl.LOCK_UN)
            finally:
                os.close(descriptor)

    def _processes(self) -> list[dict]:
        rows = self._run(["ps", "-eo", "pid=,comm=,args="]).stdout.splitlines()
        processes = []
        for row in rows:
            fields = row.strip().split(None, 2)
            if not fields:
                continue
            if len(fields) != 3 or not fields[0].isdigit():
                raise RobotClockError("Cannot parse the robot's process inventory")
            pid = int(fields[0])
            if pid != os.getpid():
                processes.append({"pid": pid, "comm": fields[1], "args": fields[2]})
        return processes

    def _check_no_ros_processes(self, processes: list[dict]) -> None:
        blocked = [p["pid"] for p in processes if _is_ros_process(p["comm"], p["args"])]
        if blocked:
            raise RobotClockError(
                "Stop existing ROS/hardware/control processes before clock changes; PIDs: "
                + ",".join(str(pid) for pid in blocked)
            )

    def _unit_state(self, unit: str) -> dict:
        output = self._run([
            "systemctl", "show", unit, "--property=Id", "--property=LoadState",
            "--property=ActiveState", "--property=MainPID", "--property=ControlGroup",
        ]).stdout
        values = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
        active = values.get("ActiveState")
        if active not in {"active", "inactive", "failed", "activating", "deactivating", "reloading"}:
            raise RobotClockError("Cannot determine clock service state: " + unit)
        if active == "deactivating":
            raise RobotClockError("A clock service is transitioning; retry after it settles: " + unit)
        try:
            main_pid = int(values.get("MainPID", "0"))
        except ValueError:
            raise RobotClockError("Cannot determine clock service PID: " + unit) from None
        return {
            "id": values.get("Id", unit), "state": active,
            "active": active in {"active", "activating", "reloading"},
            "main_pid": main_pid, "cgroup": values.get("ControlGroup", ""),
        }

    def _check_clock_ownership(self, processes: list[dict]) -> None:
        states = [state for state in self._original_units.values() if state["active"]]
        for process in processes:
            if not _is_clock_process(process["comm"], process["args"]):
                continue
            if any(process["pid"] == state["main_pid"] for state in states):
                continue
            try:
                cgroups = [
                    line.split(":", 2)[2]
                    for line in _read_text(f"/proc/{process['pid']}/cgroup").splitlines()
                    if len(line.split(":", 2)) == 3
                ]
            except OSError:
                raise RobotClockError("Cannot establish clock daemon ownership") from None
            if not any(
                state["cgroup"] and any(
                    path == state["cgroup"] or path.startswith(state["cgroup"] + "/")
                    for path in cgroups
                ) for state in states
            ):
                raise RobotClockError(
                    "An unmanaged clock daemon is running; PID: " + str(process["pid"])
                )

    def _check_clock_sources_stopped(self) -> None:
        if any(self._unit_state(unit)["active"] for unit in CLOCK_UNITS):
            raise RobotClockError("A clock service is still active or was restarted")
        if any(_is_clock_process(p["comm"], p["args"]) for p in self._processes()):
            raise RobotClockError("A clock daemon remains active outside the stopped services")

    def _require_locked(self, allowed: set[str]) -> None:
        if self._lock_fd is None or self.phase not in allowed:
            raise RobotClockError("Clock operation is not valid in the current session phase")

    def prepare(self) -> dict:
        if self.phase != "new":
            raise RobotClockError("Clock preparation can run only once per session")
        try:
            self._check_identity()
            self._acquire_lock()
            processes = self._processes()
            self._check_no_ros_processes(processes)
            self._timezone = _read_text("/etc/timezone")
            self._original_units = {unit: self._unit_state(unit) for unit in CLOCK_UNITS}
            self._check_clock_ownership(processes)
            # Alias units can refer to the same daemon. Record each service once.
            seen_ids = set()
            for unit, state in self._original_units.items():
                if state["active"] and state["id"] not in seen_ids:
                    seen_ids.add(state["id"])
                    # Record before executing: a command can time out after stopping.
                    self._stopped_units.append(unit)
                    self._privileged(["systemctl", "stop", unit])
            self._check_clock_sources_stopped()
            self._check_no_ros_processes(self._processes())
            self.phase = "prepared"
            return {
                "success": True, "phase": self.phase, "session_id": self.session_id,
                "timezone": self._timezone, "original_clock_services": self._original_units,
                "temporarily_stopped": list(self._stopped_units),
            }
        except (RobotClockError, OSError) as error:
            # Preserve the original sanitized error while attempting full rollback.
            recovery = self.restore()
            suffix = "" if recovery["success"] else "; clock service restoration needs attention"
            if isinstance(error, RobotClockError):
                raise RobotClockError(str(error) + suffix) from None
            raise RobotClockError("Could not prepare robot clock session" + suffix) from None

    def sample(self) -> dict:
        self._require_locked({"prepared", "set", "verified"})
        return {"time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns()}

    def set_time(self, epoch_ns: int) -> dict:
        received_monotonic = time.monotonic_ns()
        self._require_locked({"prepared", "set"})
        if isinstance(epoch_ns, bool) or not isinstance(epoch_ns, int) or not (
            MIN_EPOCH_NS <= epoch_ns < MAX_EPOCH_NS
        ):
            raise RobotClockError("Clock target must be integer Unix nanoseconds in years 2024..2100")
        self._check_no_ros_processes(self._processes())
        self._check_clock_sources_stopped()
        # Advance source UTC by time spent checking safety on this robot. The
        # PC still measures the actual offset afterwards; never relax its bound.
        processing_ns = time.monotonic_ns() - received_monotonic
        if not 0 <= processing_ns <= 20_000_000_000:
            raise RobotClockError("Clock request processing stalled or monotonic clock regressed")
        target_ns = epoch_ns + processing_ns
        if target_ns >= MAX_EPOCH_NS:
            raise RobotClockError("Adjusted clock target is outside the allowed date range")
        seconds, nanos = divmod(target_ns, 1_000_000_000)
        self._privileged(["/usr/bin/date", "--utc", "--set", f"@{seconds}.{nanos:09d}"])
        self.phase = "set"
        return {"success": True, "requested_epoch_ns": epoch_ns,
                "processing_compensation_ns": processing_ns,
                "target_epoch_ns": target_ns, **self.sample()}

    def verify(self, bound_sec: float) -> dict:
        self._require_locked({"set"})
        if isinstance(bound_sec, bool) or not isinstance(bound_sec, (int, float)) or not (
            math.isfinite(bound_sec) and 0 <= bound_sec <= 0.1
        ):
            raise RobotClockError("PC/robot clock error bound must be finite and at most 0.1 seconds")
        self._check_identity()
        self._check_no_ros_processes(self._processes())
        self._check_clock_sources_stopped()
        if _read_text("/etc/timezone") != self._timezone:
            raise RobotClockError("Robot timezone changed during clock preparation")
        try:
            boot_id = str(uuid.UUID(_read_text("/proc/sys/kernel/random/boot_id")))
        except (OSError, ValueError):
            raise RobotClockError("Cannot validate the robot boot ID") from None
        token = {
            "schema": 1, "verified": True, "boot_id": boot_id,
            "monotonic_sec": time.monotonic(), "pc_clock_bound_sec": float(bound_sec),
            "uid": _current_uid(), "session_id": self.session_id,
            "robot_ip": ROBOT_IP, "timezone": self._timezone,
        }
        if self.transport == 'api':
            token.pop('robot_ip')  # DHCP address is routing, never enrollment identity.
            token.update(transport='api', robot_identity=self.robot_identity)
        self.phase = "verified"
        return token

    def restoration_result(self) -> dict | None:
        """Return existing rollback evidence without running any OS command."""
        if self._last_restore_result is None:
            return None
        return {
            key: list(value) if isinstance(value, list) else value
            for key, value in self._last_restore_result.items()
        }

    def restore(self) -> dict:
        """Restore only services originally active, without enabling any unit.

        On a failed restart the lock and password remain available for an explicit
        retry. A dead operator's OS releases its lock; it does not silently claim
        that NTP has been restored.
        """
        if self.phase == "restored" and self._last_restore_result is not None:
            return self.restoration_result()
        errors = []
        restored = []
        for unit in list(reversed(self._stopped_units)):
            try:
                self._privileged(["systemctl", "start", unit])
                if not self._unit_state(unit)["active"]:
                    raise RobotClockError("Clock service did not become active")
                self._stopped_units.remove(unit)
                restored.append(unit)
            except RobotClockError:
                errors.append("Could not restore clock service: " + unit)
        if not errors:
            try:
                self._release_lock()
            except OSError:
                errors.append("Could not release robot operator lock")
            self._sudo_password = ""
            self.phase = "restored"
        else:
            self.phase = "restore_failed"
        self._last_restore_result = {
            "success": not errors, "phase": self.phase, "session_id": self.session_id,
            "restored_services": restored, "pending_services": list(self._stopped_units),
            "errors": errors,
        }
        return self.restoration_result()
