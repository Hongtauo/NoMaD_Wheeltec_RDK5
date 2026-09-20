"""Host-independent tests: every OS, sudo and subprocess effect is mocked."""

import json
import stat
import subprocess
import types
import unittest
from unittest.mock import Mock, patch

import robot_clock as clock


class FakeRobot:
    def __init__(self):
        self.calls = []
        self.address = clock.ROBOT_IP
        self.processes = []
        self.timezone = "Asia/Shanghai"
        self.boot_id = "ae3af47f-9040-42cd-b164-f5d6e4e8ef96"
        self.active = {unit: unit == "ntp.service" for unit in clock.CLOCK_UNITS}
        self.ids = {unit: unit for unit in clock.CLOCK_UNITS}
        self.fail_stop = set()
        self.fail_start = set()
        self.extra_clock = False
        self.cgroup = "0::/unknown.service"

    def read(self, path):
        path = str(path)
        if path == "/etc/timezone":
            return self.timezone
        if path == "/proc/sys/kernel/random/boot_id":
            return self.boot_id
        if path.endswith("/cgroup"):
            return self.cgroup
        raise AssertionError("Unexpected read: " + path)

    def run(self, command, **kwargs):
        self.calls.append((command, kwargs))
        if command[0] == "sudo":
            assert command[:5] == ["sudo", "-S", "-p", "", "--"]
            assert kwargs["input"] == "private-test-secret\n"
            command = command[5:]
        else:
            assert kwargs.get("input") is None
        if command[:2] == ["ip", "-j"]:
            output = json.dumps([{"addr_info": [{"local": self.address}]}])
        elif command[0] == "ps":
            lines = [
                "42 python3 python3 robot_operator.py",
                "43 python3 python3 -c from ros2cli.daemon.daemonize import main",
                *self.processes,
            ]
            if self.active["ntp.service"]:
                lines.append("100 ntpd /usr/sbin/ntpd -g -u ntp:ntp")
            if self.extra_clock:
                lines.append("900 chronyd /usr/sbin/chronyd")
            output = "\n".join(lines)
        elif command[:2] == ["systemctl", "show"]:
            unit = command[2]
            state = "active" if self.active[unit] else "inactive"
            output = "\n".join([
                "Id=" + self.ids[unit], "LoadState=loaded", "ActiveState=" + state,
                "MainPID=" + ("100" if self.active[unit] else "0"),
                "ControlGroup=/system.slice/" + unit,
            ])
        elif command[:2] == ["systemctl", "stop"]:
            unit = command[2]
            if unit in self.fail_stop:
                return subprocess.CompletedProcess(command, 1, "", "private-test-secret")
            self.active[unit] = False
            for alias in self.active:
                if self.ids[alias] == self.ids[unit]:
                    self.active[alias] = False
            output = ""
        elif command[:2] == ["systemctl", "start"]:
            unit = command[2]
            if unit in self.fail_start:
                return subprocess.CompletedProcess(command, 1, "", "private-test-secret")
            self.active[unit] = True
            output = ""
        elif command[:3] == ["/usr/bin/date", "--utc", "--set"]:
            output = "ignored display value"
        else:
            raise AssertionError("Unexpected command: " + repr(command))
        return subprocess.CompletedProcess(command, 0, output, "")


class RobotClockTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.flock = Mock()
        self.close = Mock()
        mocks = {
            "_current_uid": Mock(return_value=1000),
            "_read_text": Mock(side_effect=self.robot.read),
            "fcntl": types.SimpleNamespace(LOCK_EX=2, LOCK_NB=4, LOCK_UN=8, flock=self.flock),
        }
        self.patches = [patch.object(clock, name, value) for name, value in mocks.items()]
        self.patches += [
            patch.object(clock.platform, "machine", return_value="aarch64"),
            patch.object(clock.os, "getpid", return_value=42),
            patch.object(clock.os, "open", return_value=55),
            patch.object(clock.os, "fstat", return_value=types.SimpleNamespace(
                st_mode=stat.S_IFREG | 0o600, st_uid=1000,
            )),
            patch.object(clock.os, "fchmod", create=True),
            patch.object(clock.os, "close", self.close),
            patch.object(clock.subprocess, "run", side_effect=self.robot.run),
            patch.object(clock.time, "time_ns", return_value=1_788_700_000_123_456_789),
            patch.object(clock.time, "monotonic_ns", return_value=123_000_000_000),
            patch.object(clock.time, "monotonic", return_value=123.0),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.session = clock.RobotClockSession("private-test-secret")

    def sudo_commands(self):
        return [command[5:] for command, _ in self.robot.calls if command[0] == "sudo"]

    def ready(self):
        self.session.prepare()
        self.session.set_time(1_788_700_000_123_456_789)

    def test_constructor_has_no_effects_and_secret_is_not_in_repr(self):
        self.assertEqual(self.robot.calls, [])
        self.assertNotIn("private-test-secret", repr(self.session))

    def test_prepare_suspends_only_active_unit_and_keeps_lock(self):
        result = self.session.prepare()
        self.assertEqual(result["temporarily_stopped"], ["ntp.service"])
        self.assertEqual(self.sudo_commands(), [["systemctl", "stop", "ntp.service"]])
        self.flock.assert_called_once_with(55, 6)
        self.close.assert_not_called()
        self.assertNotIn("private-test-secret", json.dumps(result))

    def test_prepare_handles_active_alias_only_once(self):
        self.robot.active["ntpsec.service"] = True
        self.robot.ids["ntpsec.service"] = "ntp.service"
        self.session.prepare()
        self.assertEqual(self.sudo_commands(), [["systemctl", "stop", "ntp.service"]])

    def test_restore_restarts_only_originally_active_unit(self):
        self.ready()
        self.session.verify(0.01)
        result = self.session.restore()
        self.assertTrue(result["success"])
        self.assertEqual(result["restored_services"], ["ntp.service"])
        self.assertEqual(self.sudo_commands()[-1], ["systemctl", "start", "ntp.service"])
        self.flock.assert_called_with(55, 8)
        self.close.assert_called_once_with(55)
        self.assertEqual(self.session._sudo_password, "")
        self.assertFalse(any("enable" in command or "disable" in command for command in self.sudo_commands()))

    def test_restore_can_be_retried_after_start_failure(self):
        self.session.prepare()
        self.robot.fail_start.add("ntp.service")
        result = self.session.restore()
        self.assertFalse(result["success"])
        self.assertEqual(result["pending_services"], ["ntp.service"])
        self.close.assert_not_called()
        self.robot.fail_start.clear()
        self.assertTrue(self.session.restore()["success"])

    def test_restore_is_idempotent(self):
        self.session.prepare()
        self.session.restore()
        call_count = len(self.robot.calls)
        self.assertTrue(self.session.restore()["success"])
        self.assertEqual(len(self.robot.calls), call_count)

    def test_restore_with_no_prepare_does_not_touch_clock(self):
        self.assertTrue(self.session.restore()["success"])
        self.assertEqual(self.robot.calls, [])

    def test_restoration_evidence_is_effect_free_and_copied(self):
        self.assertIsNone(self.session.restoration_result())
        self.session.prepare()
        result = self.session.restore()
        calls = list(self.robot.calls)
        result['restored_services'].clear()
        evidence = self.session.restoration_result()
        self.assertEqual(evidence['restored_services'], ['ntp.service'])
        evidence['pending_services'].append('not-real.service')
        self.assertEqual(self.session.restore()['pending_services'], [])
        self.assertEqual(self.robot.calls, calls)

    def test_prepare_refusal_retains_successful_noop_rollback(self):
        self.robot.processes = ['77 test wheeltec_robot_node']
        with self.assertRaisesRegex(clock.RobotClockError, 'PIDs: 77'):
            self.session.prepare()
        self.assertEqual(self.session.phase, 'restored')
        result = self.session.restoration_result()
        self.assertTrue(result['success'])
        self.assertEqual(result['pending_services'], [])
        self.assertEqual(result['restored_services'], [])
        self.assertEqual(self.sudo_commands(), [])

    def test_retry_replaces_failed_restoration_evidence(self):
        self.session.prepare()
        self.robot.fail_start.add('ntp.service')
        self.session.restore()
        self.assertFalse(self.session.restoration_result()['success'])
        self.robot.fail_start.clear()
        result = self.session.restore()
        self.assertTrue(result['success'])
        self.assertEqual(self.session.restoration_result(), result)

    def test_prepare_refuses_non_arm64(self):
        with patch.object(clock.platform, "machine", return_value="x86_64"):
            with self.assertRaisesRegex(clock.RobotClockError, "ARM64"):
                self.session.prepare()
        self.assertEqual(self.sudo_commands(), [])

    def test_prepare_refuses_root(self):
        with patch.object(clock, "_current_uid", return_value=0):
            with self.assertRaisesRegex(clock.RobotClockError, "not as root"):
                self.session.prepare()
        self.assertEqual(self.sudo_commands(), [])

    def test_prepare_requires_actual_local_robot_ip(self):
        self.robot.address = "192.168.0.180"
        with self.assertRaisesRegex(clock.RobotClockError, "not locally assigned"):
            self.session.prepare()
        self.assertEqual(self.sudo_commands(), [])

    def test_existing_operator_lock_refuses_without_privilege(self):
        self.flock.side_effect = BlockingIOError()
        with self.assertRaisesRegex(clock.RobotClockError, "Another operator"):
            self.session.prepare()
        self.assertEqual(self.sudo_commands(), [])
        self.close.assert_called_once_with(55)

    def test_prepare_refuses_hardware_or_control_and_never_kills(self):
        for command in [
            "ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py",
            "python3 -m nomad_navigation.robot_gateway",
            "/opt/ros/humble/lib/rclcpp_components/component_container",
            "wheeltec_robot_node --ros-args -p car_mode:=V550_akm",
            "controller_server", "ros2 topic echo /odom",
        ]:
            with self.subTest(command=command):
                self.robot.processes = ["77 test " + command]
                session = clock.RobotClockSession("private-test-secret")
                with self.assertRaisesRegex(clock.RobotClockError, "PIDs: 77"):
                    session.prepare()
        self.assertEqual(self.sudo_commands(), [])

    def test_unmanaged_clock_daemon_refused_before_any_stop(self):
        self.robot.extra_clock = True
        with self.assertRaisesRegex(clock.RobotClockError, "unmanaged"):
            self.session.prepare()
        self.assertEqual(self.sudo_commands(), [])

    def test_clock_daemon_can_be_associated_using_cgroup(self):
        self.robot.extra_clock = True
        self.robot.cgroup = "0::/system.slice/ntp.service/worker"
        states = {"ntp.service": {"active": True, "main_pid": 100, "cgroup": "/system.slice/ntp.service"}}
        self.session._original_units = states
        self.session._check_clock_ownership([{"pid": 900, "comm": "chronyd", "args": "chronyd"}])

    def test_partial_stop_failure_restores_already_stopped_services(self):
        self.robot.active["chrony.service"] = True
        self.robot.fail_stop.add("chrony.service")
        with self.assertRaises(clock.RobotClockError):
            self.session.prepare()
        self.assertTrue(self.robot.active["ntp.service"])
        self.assertIn(["systemctl", "start", "ntp.service"], self.sudo_commands())
        self.close.assert_called_once_with(55)

    def test_stop_error_does_not_expose_sudo_output_or_password(self):
        self.robot.fail_stop.add("ntp.service")
        with self.assertRaises(clock.RobotClockError) as captured:
            self.session.prepare()
        self.assertNotIn("private-test-secret", str(captured.exception))

    def test_set_time_uses_exact_unix_nanoseconds_and_never_writes_rtc(self):
        self.ready()
        self.assertIn(
            ["/usr/bin/date", "--utc", "--set", "@1788700000.123456789"],
            self.sudo_commands(),
        )
        self.assertFalse(any("hwclock" in " ".join(command) for command in self.sudo_commands()))
        self.assertEqual(self.robot.timezone, "Asia/Shanghai")

    def test_set_time_compensates_local_safety_check_latency(self):
        self.session.prepare()
        target = 1_788_700_000_123_456_789
        with patch.object(clock.time, "monotonic_ns", side_effect=[100, 250_000_100, 250_000_200]):
            result = self.session.set_time(target)
        self.assertEqual(result['processing_compensation_ns'], 250_000_000)
        self.assertEqual(result['target_epoch_ns'], target + 250_000_000)
        self.assertIn(["/usr/bin/date", "--utc", "--set", "@1788700000.373456789"], self.sudo_commands())

    def test_set_time_range_and_types_rejected_without_date(self):
        self.session.prepare()
        for value in [True, 1.2, "1788700000", 0, clock.MIN_EPOCH_NS - 1, clock.MAX_EPOCH_NS]:
            with self.subTest(value=value), self.assertRaises(clock.RobotClockError):
                self.session.set_time(value)
        self.assertFalse(any(command[0] == "/usr/bin/date" for command in self.sudo_commands()))

    def test_sample_contains_both_clocks(self):
        self.session.prepare()
        self.assertEqual(self.session.sample(), {
            "time_ns": 1_788_700_000_123_456_789, "monotonic_ns": 123_000_000_000,
        })

    def test_verify_returns_json_token_and_locks_out_further_clock_changes(self):
        self.ready()
        token = self.session.verify(0.025)
        self.assertEqual(token["schema"], 1)
        self.assertTrue(token["verified"])
        self.assertEqual(token["boot_id"], self.robot.boot_id)
        self.assertEqual(token["monotonic_sec"], 123.0)
        self.assertEqual(token["pc_clock_bound_sec"], 0.025)
        self.assertEqual(token["uid"], 1000)
        self.assertEqual(token["session_id"], self.session.session_id)
        self.assertNotIn("private-test-secret", json.dumps(token))
        with self.assertRaises(clock.RobotClockError):
            self.session.set_time(1_788_700_000_123_456_789)
        with self.assertRaises(clock.RobotClockError):
            self.session.verify(0.01)

    def test_verify_rejects_bad_error_bounds(self):
        self.ready()
        for bound in [float("nan"), float("inf"), -0.01, 0.100001, True, "0.01"]:
            with self.subTest(bound=bound), self.assertRaises(clock.RobotClockError):
                self.session.verify(bound)
        self.assertEqual(self.session.phase, "set")

    def test_verify_rejects_no_set_even_after_prepare(self):
        self.session.prepare()
        with self.assertRaises(clock.RobotClockError):
            self.session.verify(0.01)

    def test_timezone_change_rejects_token(self):
        self.ready()
        self.robot.timezone = "UTC"
        with self.assertRaisesRegex(clock.RobotClockError, "timezone changed"):
            self.session.verify(0.01)

    def test_restarted_clock_service_prevents_date_and_token(self):
        self.ready()
        self.robot.active["chrony.service"] = True
        with self.assertRaisesRegex(clock.RobotClockError, "still active"):
            self.session.verify(0.01)
        with self.assertRaisesRegex(clock.RobotClockError, "still active"):
            self.session.set_time(1_788_700_000_123_456_789)

    def test_new_hardware_process_prevents_date(self):
        self.session.prepare()
        self.robot.processes = ["90 wheeltec wheeltec_robot_node"]
        with self.assertRaisesRegex(clock.RobotClockError, "PIDs: 90"):
            self.session.set_time(1_788_700_000_123_456_789)
        self.assertFalse(any(command[0] == "/usr/bin/date" for command in self.sudo_commands()))

    def test_invalid_boot_id_refuses_verification(self):
        self.ready()
        self.robot.boot_id = "invalid"
        with self.assertRaisesRegex(clock.RobotClockError, "boot ID"):
            self.session.verify(0.01)

    def test_initial_and_closed_phase_reject_clock_access(self):
        for action in [self.session.sample, lambda: self.session.set_time(clock.MIN_EPOCH_NS), lambda: self.session.verify(0.01)]:
            with self.assertRaises(clock.RobotClockError):
                action()
        self.session.prepare()
        with self.assertRaises(clock.RobotClockError):
            self.session.prepare()
        self.session.restore()
        with self.assertRaises(clock.RobotClockError):
            self.session.sample()

    def test_password_must_be_single_line(self):
        for password in ["", "a\nb", "a\rb", "a\0b", None]:
            with self.subTest(password=password), self.assertRaises(clock.RobotClockError):
                clock.RobotClockSession(password)


if __name__ == "__main__":
    unittest.main()
