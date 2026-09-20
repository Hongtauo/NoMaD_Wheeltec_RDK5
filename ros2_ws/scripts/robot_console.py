#!/usr/bin/env python3
"""Windows operator console using system OpenSSH and Python standard library.

Use this user's saved robot credential when configured, otherwise prompt.
Passwords never enter argv, project files or reports. Clock sampling begins
after SSH authentication; password autofill never confirms motion arming.
"""
import argparse
from datetime import datetime, timezone
import getpass
import ipaddress
import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import uuid

from robot_credentials import read_password
from robot_ssh_autofill import auto_ssh_options
from campus_session_rpc import ReconnectingRpc, ConnectionLost, ArmCancelled


def validate_cleanup_result(value):
    """Accept only a structured terminal result from the authenticated peer."""
    if (not isinstance(value, dict) or type(value.get('stopped')) is not bool
            or not isinstance(value.get('errors'), list)
            or any(not isinstance(error, str) for error in value['errors'])
            or value.get('hardware_stop_guaranteed') is not False
            or 'clock_restore' not in value):
        raise ValueError('Invalid remote cleanup result')
    clock = value['clock_restore']
    if clock is not None:
        if not isinstance(clock, dict):
            raise ValueError('Invalid remote clock cleanup result')
        if 'error' in clock:
            if not isinstance(clock['error'], str) or not value['errors']:
                raise ValueError('Invalid remote clock cleanup failure')
        else:
            if (type(clock.get('success')) is not bool
                    or not isinstance(clock.get('errors'), list)
                    or any(not isinstance(error, str) for error in clock['errors'])
                    or not isinstance(clock.get('pending_services'), list)
                    or any(not isinstance(unit, str) for unit in clock['pending_services'])):
                raise ValueError('Invalid remote clock restoration result')
            if ((not clock['success'] or clock['errors'] or clock['pending_services'])
                    and not value['errors']):
                raise ValueError('Remote cleanup omitted its clock restoration failure')
    return value


def ssh_command(transport='dds', robot_host='192.168.0.100'):
    if transport not in ('dds', 'api'):
        raise ValueError('Unknown robot transport')
    address = ipaddress.IPv4Address(robot_host)
    if (address.is_unspecified or address.is_loopback or address.is_multicast
            or address.is_link_local or int(address) == 0xffffffff):
        raise ValueError('Robot host must be a unicast IPv4 address')
    if transport == 'dds' and str(address) != '192.168.0.100':
        raise ValueError('Use --transport api for campus DHCP; legacy DDS uses its reviewed robot IP')
    command = [
            'ssh', '-T', '-o', 'StrictHostKeyChecking=yes', '-o', 'UpdateHostKeys=no',
            '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=2',
            '-o', 'ServerAliveCountMax=3']
    if transport == 'api':
        # Reuse the previously enrolled robot key, never trust a DHCP address's
        # new key automatically or weaken StrictHostKeyChecking.
        command += ['-o', 'HostKeyAlias=192.168.0.100']
    setup = 'campus/setup.bash' if transport == 'api' else 'setup.bash'
    remote = ('source /home/wheeltec/nomad_live_control/' + setup + ' >/dev/null && '
              'exec python3 -u /home/wheeltec/nomad_live_control/robot_operator.py')
    if transport == 'api':
        remote += ' --transport api'
    return command + ['wheeltec@' + str(address), remote]


class Rpc:
    def __init__(self, transport='dds', robot_host='192.168.0.100', *, autofill=False):
        command = ssh_command(transport, robot_host)
        options = {}
        if autofill:
            # Only the already-enrolled password prompt may use the credential.
            # Keep host-key verification; never answer trust/passphrase prompts.
            command = [command[0], '-o', 'PreferredAuthentications=password',
                       '-o', 'PasswordAuthentication=yes', '-o', 'PubkeyAuthentication=no',
                       '-o', 'KbdInteractiveAuthentication=no', '-o', 'BatchMode=no',
                       '-o', 'NumberOfPasswordPrompts=1', *command[1:]]
            options = auto_ssh_options()
        self.process = subprocess.Popen(command,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
            text=True, encoding='utf-8', bufsize=1, **options)
        self.inbox = queue.Queue()
        self.lock = threading.Lock()
        self.sequence = 0
        self.fault = None
        self.heartbeats = threading.Event()
        self.closing = threading.Event()
        self.reader_done = threading.Event()
        self.closed_received = threading.Event()
        self.closed_result = None
        self.protocol_error = None
        self.close_lock = threading.Lock()
        self.stop_attempted = False
        threading.Thread(target=self.read, daemon=True).start()
        try:
            reply = self.inbox.get(timeout=90.)
            if not isinstance(reply, dict) or reply.get('event') != 'ready' or reply.get('protocol') != 1:
                raise RuntimeError('Robot operator did not complete its SSH protocol handshake')
        except BaseException:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5.)
            raise

    def read(self):
        try:
            for line in self.process.stdout:
                message = json.loads(line)
                if not isinstance(message, dict):
                    raise ValueError('Operator response must be an object')
                if message.get('event') == 'closed':
                    if self.closed_received.is_set():
                        raise ValueError('Duplicate terminal operator response')
                    # Save independently of the request queue: a rejected prepare
                    # is followed by this event and EOF, not a reply to a new stop.
                    self.closed_result = validate_cleanup_result(message.get('result'))
                    self.closed_received.set()
                self.inbox.put(message)
        except Exception:
            self.protocol_error = 'Invalid or lost SSH operator protocol'
            self.fault = self.protocol_error
        finally:
            self.reader_done.set()
            self.inbox.put(None)

    def call(self, op, timeout=30., **fields):
        deadline = time.monotonic() + timeout
        if not self.lock.acquire(timeout=max(0., timeout)):
            raise RuntimeError('Operator request lock timed out; no request sent')
        try:
            if self.closing.is_set() and op != 'stop':
                raise RuntimeError('Operator session is closing; no request sent')
            if self.closed_received.is_set():
                raise RuntimeError('Remote session already closed; no request sent')
            if self.process.poll() is not None or self.fault:
                raise RuntimeError(self.fault or 'SSH process ended')
            if time.monotonic() >= deadline:
                raise RuntimeError('Operator request timed out before sending; no request sent')
            self.sequence += 1
            identifier = self.sequence
            self.process.stdin.write(json.dumps({'id': identifier, 'op': op, **fields}, allow_nan=False) + '\n')
            self.process.stdin.flush()
            while time.monotonic() < deadline:
                try:
                    response = self.inbox.get(timeout=max(.001, deadline - time.monotonic()))
                except queue.Empty:
                    break
                if response is None:
                    raise RuntimeError('SSH connection closed; no automatic retry or rearm')
                if response.get('event') == 'closed':
                    # Terminal cleanup is never evidence that an arm/request
                    # succeeded, even when it races that request's response.
                    raise RuntimeError('Remote session closed before request confirmation; not retried')
                if response.get('event') == 'fault':
                    raise RuntimeError(response.get('error', 'Robot session fault'))
                if type(response.get('id')) is int and response['id'] == identifier:
                    if type(response.get('ok')) is not bool:
                        raise RuntimeError('Invalid operator response; result uncertain')
                    if not response['ok']:
                        raise RuntimeError(response.get('error', 'Robot rejected request'))
                    if 'result' not in response:
                        raise RuntimeError('Missing operator response result; result uncertain')
                    return response['result']
            raise RuntimeError('Operator request timed out; result uncertain, not retried')
        finally:
            self.lock.release()

    def start_heartbeat(self):
        def heartbeat():
            while not self.heartbeats.wait(1.):
                try:
                    self.call('ping', timeout=2.)
                except Exception:
                    if self.heartbeats.is_set():
                        return  # A deliberate close is not a heartbeat fault.
                    self.fault = 'Operator heartbeat lost: remote session will disarm; use physical estop if needed'
                    print('\n' + self.fault, file=sys.stderr, flush=True)
                    try:
                        self.process.stdin.close()
                    except Exception:
                        pass
                    return
        threading.Thread(target=heartbeat, daemon=True).start()

    def close(self, timeout=40.):
        self.heartbeats.set()
        self.closing.set()
        deadline = time.monotonic() + timeout
        if not self.close_lock.acquire(timeout=max(0., timeout)):
            raise RuntimeError('Remote cleanup uncertain: another close did not finish')
        try:
            if (not self.stop_attempted and not self.closed_received.is_set()
                    and self.process.poll() is None and self.fault is None):
                self.stop_attempted = True
                try:
                    self.call('stop', timeout=max(.001, deadline - time.monotonic()))
                except Exception:
                    # The remote can already be closing after prepare/start
                    # failed. Do not retry stop; retain its final event instead.
                    pass
            try:
                self.process.stdin.close()
            except Exception:
                pass
            try:
                self.process.wait(timeout=max(.001, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                # This terminates only the local SSH transport, never a guessed robot PID.
                self.process.terminate()
                self.process.wait(timeout=5.)
            # process.poll()/wait() can observe exit before the reader has
            # consumed buffered stdout. EOF without a valid event is uncertain.
            self.reader_done.wait(timeout=max(0., deadline - time.monotonic()))
            if self.protocol_error or not self.closed_received.is_set():
                raise RuntimeError('Remote cleanup uncertain: no valid closed result received over SSH')
            return self.closed_result
        finally:
            self.close_lock.release()


def timed_sample(query):
    before = time.time_ns()
    mono = time.monotonic_ns()
    remote = query()
    elapsed = (time.monotonic_ns() - mono) / 1e9
    after = time.time_ns()
    if abs((after - before) / 1e9 - elapsed) > .02:
        raise RuntimeError('Windows clock stepped during measurement; stop and check host time')
    stamp = remote['time_ns'] if isinstance(remote, dict) else remote
    if type(stamp) is not int or stamp <= 0:
        raise ValueError('Invalid remote clock sample')
    offset = (stamp - (before + after) / 2) / 1e9
    uncertainty = elapsed / 2 + .001
    return {'offset_sec': offset, 'rtt_sec': elapsed, 'uncertainty_sec': uncertainty,
            'bound_sec': abs(offset) + uncertainty, 'remote_time_ns': stamp}


def runtime_clock_samples():
    # Measure over one established channel. Starting docker.exe for each
    # sample would mostly measure CLI startup, not the kernel clock offset.
    code = "import sys,time\nprint('clock-ready',flush=True)\nfor line in sys.stdin:\n print(time.time_ns(),flush=True)\n"
    process = subprocess.Popen(['docker', 'exec', '-i', 'nomad-robot-runtime', 'python3', '-u', '-c', code],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
                               text=True, bufsize=1)
    inbox = queue.Queue()
    def read():
        for line in process.stdout:
            inbox.put(line.strip())
        inbox.put(None)
    threading.Thread(target=read, daemon=True).start()
    try:
        if inbox.get(timeout=15.) != 'clock-ready':
            raise RuntimeError('Runtime clock probe failed to start')
        def query():
            process.stdin.write('sample\n')
            process.stdin.flush()
            return int(inbox.get(timeout=5.))
        query()  # Establish the interactive path before measuring.
        values = [timed_sample(query) for _ in range(5)]
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5.)
    best = min(values, key=lambda row: row['rtt_sec'])
    if best['bound_sec'] > .1:
        raise RuntimeError(f'Windows/Docker clock bound {best["bound_sec"]:.3f} s exceeds 0.1 s')
    return {'samples': values, 'best': best}


def synchronize(rpc, evidence=None):
    # Offset and uncertainty use a persistent authenticated SSH channel, so
    # password entry and SSH startup cannot make the supplied UTC timestamp stale.
    initial = [timed_sample(lambda: rpc.call('sample')) for _ in range(3)]
    evidence = {} if evidence is None else evidence
    evidence['initial'] = initial
    best = min(initial, key=lambda row: row['rtt_sec'])
    if best['uncertainty_sec'] > .1:
        raise RuntimeError('SSH latency too uncertain for safe clock synchronization')
    attempts = []
    evidence['attempts'] = attempts
    for _ in range(3):
        target = time.time_ns() + int(best['rtt_sec'] / 2 * 1e9)
        applied = rpc.call('set_time', epoch_ns=target)
        samples = [timed_sample(lambda: rpc.call('sample')) for _ in range(5)]
        best = min(samples, key=lambda row: row['rtt_sec'])
        attempts.append({'applied': applied, 'samples': samples, 'best': best})
        print(f'校时复测：偏差 {best["offset_sec"] * 1000:.1f} ms，误差上界 {best["bound_sec"] * 1000:.1f} ms。')
        if best['bound_sec'] <= .1:
            token = rpc.call('verify', bound_sec=best['bound_sec'])
            return {'initial': initial, 'attempts': attempts, 'verified': token}
    raise RuntimeError('Clock verification did not reach a 0.1 s error bound; robot was not started')


def show_status(value):
    print(json.dumps(value, ensure_ascii=False, indent=2), flush=True)


def verify_disarmed(rpc, seconds, evidence, allow_network_wait=False):
    """Bounded read-only status observation; never calls arm or a reset service."""
    startup_deadline = time.monotonic() + 30.
    ready_since = None
    samples = evidence.setdefault('samples', [])
    evidence.update(requested_seconds=seconds, status='waiting_ready', arm_requested=False,
                    scope='reported control/gateway status; not a raw cmd_vel/odom audit')
    next_log = 0.
    while True:
        snapshot = rpc.call('status', timeout=3.)
        now = time.monotonic()
        samples.append({'monotonic_sec': now, 'snapshot': snapshot})
        if len(samples) > 4096:
            raise RuntimeError('Disarmed verification sample bound exceeded')
        ros = snapshot.get('ros', {})
        control, gateway = ros.get('control', {}), ros.get('gateway', {})
        cv, gv = control.get('value', {}), gateway.get('value', {})
        def numeric_zero(value):
            return type(value) in (int, float) and math.isfinite(value) and value == 0.
        control_zero = (isinstance(cv, dict)
                        and {'armed', 'active_mode', 'linear_x', 'angular_z', 'model_state'} <= cv.keys()
                        and cv['armed'] is False and cv['active_mode'] is None
                        and numeric_zero(cv['linear_x']) and numeric_zero(cv['angular_z']))
        gateway_zero = (isinstance(gv, dict) and {'v', 'w', 'fault'} <= gv.keys()
                        and numeric_zero(gv['v']) and numeric_zero(gv['w']) and gv['fault'] is None)
        if (cv or control.get('fresh') is True) and not control_zero:
            raise RuntimeError('Disarmed verification saw invalid/nonzero/armed controller status')
        if (gv or gateway.get('fresh') is True) and not gateway_zero:
            raise RuntimeError('Disarmed verification saw invalid/nonzero/faulted gateway status')
        fresh = control.get('fresh') is True and gateway.get('fresh') is True
        model_ok = cv.get('model_state') == 'running' if isinstance(cv, dict) else False
        if allow_network_wait and ready_since is not None and isinstance(cv, dict):
            model_ok = model_ok or (cv.get('network_pause_resume') is True
                and cv.get('model_state') in ('network_wait', 'waiting_context'))
        ready = (snapshot.get('running') is True and fresh and control_zero and gateway_zero
                 and model_ok)
        if ready:
            ready_since = now if ready_since is None else ready_since
            evidence.update(status='observing', elapsed_ready_seconds=now-ready_since)
            if now >= next_log:
                print(f'未解锁复验：模型{cv["model_state"]}，状态新鲜，报告速度为零，已连续 {now-ready_since:.1f}s。', flush=True)
                next_log = now + 5.
            if now-ready_since >= seconds:
                evidence['status'] = 'passed'
                return
        elif ready_since is not None:
            raise RuntimeError('Disarmed verification lost fresh running model/control/gateway status')
        elif now > startup_deadline:
            raise RuntimeError('Disarmed verification did not become ready within 30 seconds')
        time.sleep(.25)


def operator_menu(rpc, cap, network_pause_resume=False):
    print(f'\n速度指令上限 {cap} m/s；启动未解锁。s=停止，t=定时，c=持续，i=状态，q=结束整组。')
    print('每次解锁前须在现场、区域清空且物理急停可用；手停、急停或操作台断联后不会自行行驶。')
    if network_pause_resume:
        print('连续挡已启用网络暂停/恢复：超时零速度等待，3次新鲜预测且停稳后恢复；定时挡仍锁停。')
    if getattr(rpc, 'reconnectable', False) is True:
        return reconnecting_operator_menu(rpc)
    while rpc.fault is None:
        command = input('\n[t/c/s/i/q] > ').strip().lower()
        if command == 'q':
            return
        if command == 's':
            show_status(rpc.call('disarm'))
        elif command == 'i':
            show_status(rpc.call('status'))
        elif command in ('t', 'c'):
            duration = 0.
            if command == 't':
                try:
                    duration = float(input('本次秒数 (0,10] > '))
                except ValueError:
                    print('秒数无效。')
                    continue
                if not math.isfinite(duration) or not 0 < duration <= 10:
                    print('必须是大于0、至多10的有限秒数。')
                    continue
            confirmed = input('已在现场、急停有效、车辆静止且画面/轨迹正常？输入 RUN 解锁 > ')
            if confirmed != 'RUN':
                print('已取消，未解锁。')
                continue
            try:
                show_status(rpc.call('arm', mode=0 if command == 't' else 1,
                                     duration_sec=duration, confirmed=True))
            except Exception as error:
                print(str(error))
                # Uncertain reply is not permission to retry an arm.
                try:
                    show_status(rpc.call('disarm'))
                except Exception:
                    raise RuntimeError('Cannot confirm disarm; immediately check robot / physical estop')
        else:
            print('请选择 t/c/s/i/q。')
    raise RuntimeError(rpc.fault)


def reconnecting_operator_menu(rpc):
    """Never replay RUN; an opted-in existing continuous task may resume safely."""
    auto_continue = getattr(rpc, 'operator_pause_resume', False) is True
    if auto_continue:
        print('已启用操作台暂停/恢复：连续 RUN 后短暂失联先停车，30 秒内重连且安全检查通过可能自动继续。'
              '请持续现场看护；s/q、急停、硬故障或过期取消续行。定时挡不自动恢复。')
    else:
        print('操作台短暂失联会撤销行驶，不销毁相机/推理；连接恢复后必须重新选择 t/c 并确认 RUN。')
    while rpc.fault is None:
        command = input('\n[t/c/s/i/q] > ').strip().lower()
        if command == 'q':
            return
        if command not in ('t', 'c', 's', 'i'):
            print('请选择 t/c/s/i/q。')
            continue
        if not rpc.connected:
            if command == 's':
                try:
                    rpc.call('disarm')
                except ConnectionLost:
                    pass  # The reconnect path must keep the pending revocation.
            show_status(rpc.connection_status())
            print('连接尚未恢复，未发送解锁。请保持现场看护；'
                  + ('原连续任务可能在恢复后自行继续，s/q 可取消。' if auto_continue
                     else '恢复后重新选择 t/c。'))
            continue
        try:
            if command in ('s', 'i'):
                show_status(rpc.call('disarm' if command == 's' else 'status', timeout=2.))
                continue
            duration = 0.
            if command == 't':
                try:
                    duration = float(input('本次秒数 (0,10] > '))
                except ValueError:
                    print('秒数无效。')
                    continue
                if not math.isfinite(duration) or not 0 < duration <= 10:
                    print('必须是大于0、至多10的有限秒数。')
                    continue
            # Request before the human prompt. A disconnect while the prompt is
            # open invalidates its local epoch even if connectivity later returns.
            ticket = rpc.call('arm_prepare', timeout=2.)
            if auto_continue and command == 'c':
                print('本次 RUN 将允许短暂操作台掉线后自动续行；离开现场前必须 s 或 q。')
            confirmed = input('已在现场、急停有效、车辆静止且画面/轨迹正常？输入 RUN 解锁 > ')
            if confirmed != 'RUN':
                print('已取消，未解锁。')
                continue
            show_status(rpc.call('arm', mode=0 if command == 't' else 1,
                duration_sec=duration, confirmed=True, ticket=ticket['ticket'],
                generation=ticket['generation'], _connection_epoch=ticket['_connection_epoch']))
        except (ConnectionLost, ArmCancelled) as error:
            print(str(error))
            if (auto_continue and isinstance(error, ConnectionLost)
                    and not rpc.connection_status().get('pending_disarm', False)):
                print('本次请求不会重试；原连续任务可能在重连和安全检查通过后自行继续。'
                      '保持现场看护，s/q 可取消。')
            else:
                print('本次操作不会自动重试；确认车辆停止后，等待连接恢复并重新选择 t/c。')
        except Exception as error:
            print(str(error))
            # An explicit arm refusal never grants permission to retry it.
            # Prefer an acknowledged revoke; if unavailable, detachment and the
            # independent robot lease revoke motion and the menu waits safely.
            if command in ('t', 'c'):
                try:
                    show_status(rpc.call('disarm', timeout=2.))
                except Exception:
                    print('未收到解除使能确认，请现场确认车辆已停／使用物理急停。')
    raise RuntimeError(rpc.fault)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--sync-only', action='store_true', help='Synchronize/check clocks, restore NTP, exit; no robot nodes')
    parser.add_argument('--verify-disarmed-seconds', type=int, default=0, choices=[0, *range(10, 601)],
                        metavar='10..600', help='Observe without an arm/menu, then stop all owned nodes')
    parser.add_argument('--max-linear-mps', type=float, choices=(.05, .1, .2), default=None)
    parser.add_argument('--transport', choices=('dds', 'api'), default='dds')
    parser.add_argument('--network-pause-resume', action='store_true')
    parser.add_argument('--operator-pause-resume', action='store_true',
                        help='Explicitly retain a continuous task for up to 30 s of operator-link loss; never drives while disconnected')
    parser.add_argument('--manual-password', action='store_true',
                        help='Ignore the saved robot credential; prompt for SSH and sudo')
    parser.add_argument('--robot-host', default='192.168.0.100',
                        help='Current robot IPv4; API mode verifies the enrolled legacy SSH host key')
    args = parser.parse_args(argv)
    if args.sync_only and args.verify_disarmed_seconds:
        parser.error('--sync-only and --verify-disarmed-seconds are mutually exclusive')
    if args.max_linear_mps is None:
        args.max_linear_mps = .05 if args.transport == 'api' else .2
    if args.network_pause_resume and args.transport != 'api':
        parser.error('--network-pause-resume requires --transport api')
    if args.operator_pause_resume and (args.transport != 'api' or not args.network_pause_resume or args.sync_only):
        parser.error('--operator-pause-resume requires --transport api --network-pause-resume and cannot use --sync-only')
    try:
        ssh_command(args.transport, args.robot_host)  # Validate before prompts, output or network.
    except ValueError as error:
        parser.error(str(error))
    output = Path(__file__).resolve().parents[2] / 'outputs/ros2_robot_operator' / (
        'console__' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        + '__' + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'starting', 'sync_only': args.sync_only,
              'transport': args.transport, 'robot_host': args.robot_host,
              'max_linear_mps': args.max_linear_mps, 'automatic_arm': False,
              'network_pause_resume': args.network_pause_resume,
              'source': 'Windows system UTC; no Windows NTP settings changed'}
    rpc, result, password = None, 1, None
    print('本入口先校时，再启动未解锁的小车节点；不会自动运动。')
    try:
        if not args.manual_password:
            password = read_password()
        autofill = password is not None
        report['password_autofill'] = autofill
        if autofill:
            print('SSH 和 sudo 将使用 Windows 当前用户保存的小车凭据；RUN 行驶确认仍需手动输入。')
        else:
            print('SSH 密码由系统 OpenSSH 提示输入；sudo 密码稍后单独输入，不保存。')
        if args.transport == 'dds':
            report['docker_clock'] = runtime_clock_samples()
        rpc_type = ReconnectingRpc if args.transport == 'api' and not args.sync_only else Rpc
        rpc_options = dict(transport=args.transport, robot_host=args.robot_host, autofill=autofill)
        if args.operator_pause_resume:
            rpc_options['operator_pause_resume'] = True
        report['operator_pause_resume'] = args.operator_pause_resume
        rpc = rpc_type(**rpc_options)
        if not autofill:
            password = getpass.getpass('小车 sudo 密码（不保存）: ')
        try:
            report['prepare'] = rpc.call('prepare', sudo_password=password)
        finally:
            password = None
        report['clock'] = {}
        report['clock'] = synchronize(rpc, report['clock'])
        bound = report['clock']['verified']['pc_clock_bound_sec']
        print(f'机器人校时通过，PC/机器人误差上界 {bound * 1000:.1f} ms。')
        if args.sync_only:
            print('仅校时模式：没有启动相机、底盘、控制器或网关。')
        else:
            start_options = {'max_linear_mps': args.max_linear_mps}
            if args.network_pause_resume:
                start_options['network_pause_resume'] = True
            if args.operator_pause_resume:
                start_options['operator_pause_resume'] = True
            report['start'] = rpc.call('start', **start_options)
            rpc.start_heartbeat()
            show_status(report['start'])
            if args.transport == 'api':
                print('校园 API 客户端随本组启动；先确认服务器和实时预览正常，再单独明确解锁。')
            else:
                print('请在另一个 PowerShell 启动 run_robot_live.ps1 -Seconds 0，然后检查图像与状态。')
            if args.verify_disarmed_seconds:
                verify_disarmed(rpc, args.verify_disarmed_seconds,
                                report.setdefault('disarmed_verification', {}),
                                allow_network_wait=args.network_pause_resume)
            else:
                operator_menu(rpc, args.max_linear_mps, args.network_pause_resume)
        report['status'], result = 'finished', 0
    except KeyboardInterrupt:
        report['status'], result = 'operator_interrupted', 0
        print('\n正在解除使能、关闭本会话节点并恢复原校时服务……')
    except Exception as error:
        report.update(status='failed', error=str(error))
        print('停止：' + str(error), file=sys.stderr)
    finally:
        password = None
        if rpc:
            try:
                report['cleanup'] = rpc.close()
                if report['cleanup'] and (report['cleanup'].get('errors') or not report['cleanup'].get('stopped')):
                    report['status'], result = 'cleanup_failed', 1
                    show_status(report['cleanup'])
            except Exception as error:
                report.update(status='cleanup_uncertain', cleanup_error=str(error))
                print('无法确认远端清理，请现场检查车辆和校时服务。', file=sys.stderr)
                result = 1
            if getattr(rpc, 'reconnectable', False) is True:
                report['operator_connection'] = rpc.connection_status()
                report['operator_connection_events'] = list(rpc.diagnostics)
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
                                           + '\n', encoding='utf-8')
        print('记录：' + str(output / 'report.json'))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
