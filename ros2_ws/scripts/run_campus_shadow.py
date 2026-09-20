#!/usr/bin/env python3
"""Bounded camera + shadow API validation on the enrolled robot. No base driver."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

from robot_operator import OperatorSession


class OwnedProcess:
    """Use the reviewed identity-pinned cleanup, without creating a ROS operator.

    A ros2 launch parent's exit does not prove its native descendants exited.
    The random inherited marker and regularly recorded PID/birth identities
    retain ownership across reparenting; pidfds avoid signalling reused PIDs.
    """

    def __init__(self):
        self.owner = OperatorSession()
        self.owner.token = {'session_id': 'campus-shadow-' + str(uuid.uuid4())}

    @property
    def process(self):
        return self.owner.launcher

    def start(self, command, log):
        self.owner.require_pidfd()
        if self.process is not None:
            raise RuntimeError('Owned shadow process can start only once')
        environment = dict(os.environ, NOMAD_OPERATOR_SESSION_ID=self.owner.token['session_id'])
        self.owner.launcher = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=environment)
        # Do not poll/reap before this read: even a very short-lived child keeps
        # its PID and /proc identity while it is our unreaped child.
        row = self.owner.process_metadata(Path(f'/proc/{self.process.pid}'))
        if row['uid'] != os.getuid():
            raise RuntimeError('Shadow process has unexpected ownership')
        self.owner.launch_birth = int(row['birth'])
        self.owner.owned_identities[self.process.pid] = row['birth']
        self.scan()

    def scan(self):
        if self.process is None:
            return
        if self.owner.launch_birth is None:
            raise RuntimeError('Cannot establish shadow launcher identity; inspect remaining processes')
        self.owner.owned_processes()
        if self.owner.ownership_errors:
            raise RuntimeError('; '.join(self.owner.ownership_errors))

    def close(self):
        if self.process is None:
            return
        if self.owner.launch_birth is None:
            raise RuntimeError('Cannot verify owned shadow processes for cleanup; inspect the robot')
        # This scans even when launcher.poll() reports exit. It signals only
        # exact owned identities, never all PIDs in a numeric process group.
        if not self.owner.stop_owned_processes():
            details = '; '.join(self.owner.process_cleanup_errors)
            raise RuntimeError('Owned camera/client cleanup is unconfirmed; inspect the robot'
                               + (': ' + details if details else ''))


class IdleLock:
    """Share the existing operator lock without taking any clock action."""

    def __init__(self):
        self.descriptor = None

    def acquire(self):
        import fcntl
        import stat
        from robot_clock import LOCK_PATH
        descriptor = os.open(LOCK_PATH, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
        try:
            metadata = os.fstat(descriptor)
            if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.getuid()
                    or stat.S_IMODE(metadata.st_mode) != 0o600):
                raise RuntimeError('Operator lock has unsafe owner, type or permissions')
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(descriptor)
            raise
        self.descriptor = descriptor

    def close(self):
        if self.descriptor is not None:
            import fcntl
            descriptor, self.descriptor = self.descriptor, None
            try:
                fcntl.flock(descriptor, fcntl.LOCK_UN)
            finally:
                os.close(descriptor)


def observe_children(camera, client, *, scan_due):
    """Poll direct children before cross-session /proc scans; never skip cleanup.

    Camera and API client have separate ownership trackers. A child can exit
    between another tracker's stat snapshot and its environ read. Reaping our
    direct children first avoids most stale snapshot observations. A race that
    occurs during a scan is retried only when every error names an exact direct
    child which was live just before that scan and is now demonstrably exited.
    The replacement scan remains strict, and close() still verifies descendants.
    """
    groups = (camera, client)

    def exited():
        codes = [group.process.poll() for group in groups]
        if codes[0] is not None:
            raise RuntimeError('Camera process exited unexpectedly')
        if codes[1] is not None:
            if codes[1] != 0:
                raise RuntimeError('Shadow client failed; inspect its report')
            return True
        return False

    if exited():
        return True  # main's finally still invokes both full descendant cleanups.
    if not scan_due:
        return False
    # Both Popen children are still unreaped here. Their numeric PIDs cannot be
    # reused before this process explicitly polls/waits on these same objects.
    live_children = {group.process.pid: group for group in groups}
    for group in groups:
        try:
            group.scan()
        except RuntimeError:
            errors = group.owner.ownership_errors
            affected = set()
            for error in errors:
                match = re.fullmatch(r'Cannot verify potential operator process PID ([0-9]+)', error)
                if match is None or int(match.group(1)) not in live_children:
                    raise
                affected.add(int(match.group(1)))
            if not affected:
                raise
            for pid in affected:
                child = live_children[pid]
                if (child.owner.launch_birth is None
                        or child.owner.owned_identities.get(pid) != str(child.owner.launch_birth)):
                    raise
                try:
                    # exit_mm/environ can become unavailable just before the
                    # final waitpid notification. This bounded wait never
                    # signals a process or treats a running process as exited.
                    child.process.wait(timeout=.05)
                except subprocess.TimeoutExpired:
                    raise RuntimeError('; '.join(errors)) from None
            # Do not clear/filter/ignore an ownership error. Re-enumerate both
            # complete process graphs after reaping; any remaining uncertainty
            # is still fatal, including an unknown/reused PID or live orphan.
            for strict_group in groups:
                strict_group.scan()
            return exited()
    return exited()


def require_idle():
    needles = ('wheeltec_robot_node', 'nomad_navigation.robot_control_node',
               'nomad_navigation.robot_gateway', 'robot_operator.py',
               'nomad_navigation.campus_api_client', 'component_container', 'camera_node',
               'robot_session_server.py')
    matches = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            arguments = (process / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Root-owned unrelated OS processes are not potential wheeltec ROS nodes.
            if process.stat().st_uid == os.getuid():
                raise RuntimeError('Cannot inspect same-user process ' + process.name)
            continue
        if any(any(needle.encode() in arg for needle in needles) for arg in arguments):
            matches.append(int(process.name))
    if matches:
        raise RuntimeError('Existing hardware/control/camera/client processes; stop them first: ' + str(matches))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=45, choices=range(10, 601), metavar='10..600')
    args = parser.parse_args()
    from nomad_navigation.robot_identity import (
        require_robot_identity, read_campus_client_config, validate_api_environment)
    import platform
    require_robot_identity()
    validate_api_environment(os.environ, platform.machine())
    OperatorSession.require_pidfd()  # Refuse before starting even camera-only hardware.
    config = read_campus_client_config()
    from campus_health import health
    health(config['server_url'], config['ca_file'])
    output = Path('/home/wheeltec/nomad_live_control/runs/campus_shadow') / (
        'shadow__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {'mode': 'camera_and_shadow_only', 'base_driver_started': False,
              'control_output': False, 'automatic_arm': False, 'seconds': args.seconds,
              'cleanup_errors': [], 'status': 'starting'}
    camera, client = OwnedProcess(), OwnedProcess()
    lock = IdleLock()
    stopping = False
    def stop(_sig, _frame):
        nonlocal stopping
        stopping = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        lock.acquire()
        require_idle()  # Under the same lock as the clock/operator startup path.
        with (output / 'camera.log').open('x') as camera_log, (output / 'client.log').open('x') as client_log:
            camera.start(['ros2', 'launch', 'turn_on_wheeltec_robot',
                          'wheeltec_camera.launch.py'], camera_log)
            command = ['python3', '-m', 'nomad_navigation.campus_api_client',
                       '--server-url', config['server_url'], '--ca-file', config['ca_file'],
                       '--token-file', config['token_file'], '--seconds', str(args.seconds),
                       '--output-root', str(output)]
            client.start(command, client_log)
            deadline = time.monotonic() + args.seconds + 15
            report.update(status='running', camera_pid=camera.process.pid, client_pid=client.process.pid)
            print(json.dumps({'output': str(output), 'mode': report['mode']}), flush=True)
            next_scan = 0.
            while not stopping and time.monotonic() < deadline:
                scan_due = time.monotonic() >= next_scan
                if observe_children(camera, client, scan_due=scan_due):
                    break
                if scan_due:
                    next_scan = time.monotonic() + .25
                time.sleep(.1)
            else:
                if not stopping:
                    raise RuntimeError('Shadow session exceeded its time bound')
            report['status'] = 'stopped' if stopping else 'completed'
    except Exception as error:
        report.update(status='failed', error=str(error))
    finally:
        for process in (client, camera):
            try:
                process.close()
            except Exception as error:
                report['cleanup_errors'].append(str(error))
        try:
            lock.close()
        except Exception as error:
            report['cleanup_errors'].append('Operator lock release: ' + str(error))
        report['cleanup_verified'] = not report['cleanup_errors']
        if report['cleanup_errors']:
            report['status'] = 'cleanup_failed'
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'report': str(output / 'report.json'), 'status': report['status'],
                          'cleanup_errors': report['cleanup_errors']}), flush=True)
    return 0 if report['status'] == 'completed' and not report['cleanup_errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
