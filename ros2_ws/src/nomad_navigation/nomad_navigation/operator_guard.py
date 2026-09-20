"""Robot-steady-clock motion gate for a reconnectable, supervised operator.

This is an authorization gate, not a replacement for camera/odometry/gateway
watchdogs. Link recovery never creates an arm intent. Default mode cancels the
old run; opt-in mode delegates bounded continuous-intent recovery to the core.
An expired or invalid generation must be replaced by a heartbeat before a new,
nonce-bound arm can be considered.
"""

from dataclasses import dataclass
import json
import math
import uuid


GUARD_ENV = 'NOMAD_OPERATOR_GUARD_SESSION'
OPERATOR_PAUSE_RESUME_ENV = 'NOMAD_OPERATOR_PAUSE_RESUME'
OPERATOR_LINK_TOPIC = '/nomad/robot_control/operator_link'
MAX_LEASE_NS = 2_000_000_000
LEASE_TRANSPORT_TOLERANCE_NS = 50_000_000
MAX_MESSAGE_BYTES = 2048
MAX_ARM_NONCES = 4096
_COMMON = frozenset(('schema', 'session_id', 'generation', 'sequence',
                     'connected', 'deadline_steady_ns', 'action'))
_ARM = frozenset(('arm_nonce', 'mode', 'duration_sec'))


def operator_pause_resume_from_env(environ):
    if OPERATOR_PAUSE_RESUME_ENV not in environ:
        return False
    if environ[OPERATOR_PAUSE_RESUME_ENV] != '1' or GUARD_ENV not in environ:
        raise ValueError('Operator pause/resume requires value 1 and a configured operator guard')
    canonical_uuid(environ[GUARD_ENV])
    return True


def canonical_uuid(value):
    if not isinstance(value, str):
        raise ValueError('Expected a canonical UUID string')
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError) as error:
        raise ValueError('Expected a canonical UUID string') from error
    if str(parsed) != value:
        raise ValueError('Expected a canonical UUID string')
    return value


def _unique_object(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError('Duplicate operator-link field')
        result[key] = value
    return result


@dataclass(frozen=True)
class GuardDecision:
    accepted: bool
    reason: str
    disarm: bool = False
    arm_request: object = None
    pause: bool = False
    generation_changed: bool = False


class OperatorGuard:
    def __init__(self, session_id, pause_resume=False):
        self.session_id = canonical_uuid(session_id)
        if type(pause_resume) is not bool:
            raise ValueError('Operator pause/resume must be an explicit boolean')
        self.pause_resume = pause_resume
        self._transient_pause = False
        self._last_accepted_lease = None
        self.generation = 0
        self.sequence = 0
        self.connected = False
        self.deadline_steady_ns = 0
        self.reason = 'operator_link_waiting'
        self.requires_new_generation = True
        self.last_arm_result = None
        # Never evict a nonce and thereby make an old confirmation reusable.
        # Exhaustion rejects further arms and requires a new operator session.
        self._arm_nonces = set()

    def revoke(self, reason='operator_link_lost'):
        self._transient_pause = False
        self._last_accepted_lease = None
        self.connected = False
        self.deadline_steady_ns = 0
        self.requires_new_generation = True
        self.reason = reason
        return GuardDecision(False, reason, disarm=True)

    def pause(self, reason='operator_link_paused'):
        if not self.pause_resume:
            return self.revoke(reason)
        self.connected = False
        self.deadline_steady_ns = 0
        self.requires_new_generation = True
        self._transient_pause = True
        self.reason = reason
        return GuardDecision(False, reason, pause=True)

    def check(self, now_ns):
        if type(now_ns) is not int or now_ns < 0:
            return self.revoke('operator_link_invalid_clock')
        if self.connected and now_ns >= self.deadline_steady_ns:
            return self.pause('operator_link_expired')
        if not self.connected or self.requires_new_generation:
            return GuardDecision(False, self.reason, disarm=not self._transient_pause,
                                 pause=self._transient_pause)
        return GuardDecision(True, 'operator_link_connected')

    def accept_json(self, text, now_ns):
        try:
            if (not isinstance(text, str) or len(text) > MAX_MESSAGE_BYTES
                    or len(text.encode('utf-8')) > MAX_MESSAGE_BYTES):
                raise ValueError('Operator-link message too large')
            payload = json.loads(text, object_pairs_hook=_unique_object,
                                 parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        except (ValueError, TypeError, UnicodeError, RecursionError):
            return self.revoke('operator_link_invalid_message')
        return self.accept(payload, now_ns)

    def accept(self, payload, now_ns):
        # Expiry is checked before processing a queued message. A same-generation
        # heartbeat/arm cannot revive a lease whose deadline has already passed.
        self.check(now_ns)
        try:
            if type(now_ns) is not int or now_ns < 0:
                raise ValueError('Invalid monotonic clock')
            if type(payload) is not dict:
                raise ValueError('Expected object')
            action = payload.get('action')
            if action not in ('heartbeat', 'disarm', 'arm', 'pause'):
                raise ValueError('Invalid action')
            if set(payload) != (_COMMON | _ARM if action == 'arm' else _COMMON):
                raise ValueError('Unexpected fields')
            if type(payload['schema']) is not int or payload['schema'] != 1:
                raise ValueError('Invalid schema')
            if canonical_uuid(payload['session_id']) != self.session_id:
                raise ValueError('Wrong session')
            generation, sequence = payload['generation'], payload['sequence']
            if (type(generation) is not int or not 0 < generation < 2**63
                    or type(sequence) is not int or not 0 < sequence < 2**63):
                raise ValueError('Invalid generation or sequence')
            if type(payload['connected']) is not bool:
                raise ValueError('Invalid connected flag')
            deadline = payload['deadline_steady_ns']
            if type(deadline) is not int or not 0 <= deadline < 2**63:
                raise ValueError('Invalid deadline')
            if action == 'pause' and (payload['connected']
                    or deadline > now_ns + MAX_LEASE_NS + LEASE_TRANSPORT_TOLERANCE_NS):
                raise ValueError('Invalid pause message')
            if action == 'arm':
                canonical_uuid(payload['arm_nonce'])
                mode, duration = payload['mode'], payload['duration_sec']
                if type(mode) is not int or mode not in (0, 1):
                    raise ValueError('Invalid arm mode')
                if (type(duration) not in (int, float) or not math.isfinite(duration)
                        or (mode == 0 and not 0 < duration <= 10)
                        or (mode == 1 and duration != 0)):
                    raise ValueError('Invalid arm duration')
        except (ValueError, TypeError, KeyError, OverflowError):
            return self.revoke('operator_link_invalid_message')

        if generation < self.generation:
            return self.revoke('operator_link_old_generation')
        if sequence <= self.sequence:
            return self.revoke('operator_link_replayed_sequence')
        self.sequence = sequence

        if action == 'disarm':
            # Stop is unconditional. Consuming its generation also prevents a
            # previously queued arm from undoing a manual stop.
            self.generation = generation
            return self.revoke('operator_disarmed')

        if action == 'pause':
            self.generation = generation
            return self.pause()

        if not payload['connected']:
            self.generation = generation
            return self.revoke('operator_link_lost')
        if (self.pause_resume and self._transient_pause and action == 'heartbeat'
                and generation == self.generation and deadline <= now_ns
                and self._last_accepted_lease == (generation, deadline)):
            # A valid heartbeat already queued on the local DDS channel may
            # arrive after its original lease expires. Consume its new sequence
            # but grant absolutely no authority/time: remain paused, require a
            # new generation, and leave the core's first-loss deadline intact.
            # Different deadlines, replay, old generations and arm requests do
            # not enter this narrow queue-drain path and still hard-revoke.
            return GuardDecision(False, self.reason, pause=True)
        if not now_ns < deadline <= now_ns + MAX_LEASE_NS + LEASE_TRANSPORT_TOLERANCE_NS:
            self.generation = generation
            return self.revoke('operator_link_invalid_deadline')

        changed = generation > self.generation
        if changed:
            self.generation = generation
            self.last_arm_result = None
            if action != 'heartbeat':
                return self.revoke('operator_link_generation_requires_heartbeat')
        elif self.requires_new_generation:
            return self.revoke('operator_link_requires_new_generation')

        if action == 'arm':
            nonce = payload['arm_nonce']
            if nonce in self._arm_nonces:
                return self.revoke('operator_link_replayed_arm')
            if len(self._arm_nonces) >= MAX_ARM_NONCES:
                return self.revoke('operator_link_arm_nonce_capacity')
            self._arm_nonces.add(nonce)

        self.connected = True
        self._transient_pause = False
        self._last_accepted_lease = (generation, deadline)
        self.deadline_steady_ns = deadline
        self.requires_new_generation = False
        self.reason = 'operator_link_connected'
        if action == 'arm':
            return GuardDecision(True, self.reason, arm_request={
                'arm_nonce': payload['arm_nonce'], 'generation': generation,
                'mode': payload['mode'], 'duration_sec': payload['duration_sec']})
        return GuardDecision(True, 'operator_link_generation_changed' if changed else self.reason,
                             disarm=changed and not self.pause_resume,
                             pause=changed and self.pause_resume,
                             generation_changed=changed)

    def record_arm_result(self, request, success, message):
        self.last_arm_result = dict(arm_nonce=request['arm_nonce'],
                                   generation=request['generation'],
                                   success=bool(success), message=str(message))

    def snapshot(self, now_ns):
        self.check(now_ns)
        return dict(enabled=True, pause_resume=self.pause_resume,
                    session_id=self.session_id, connected=self.connected,
                    generation=self.generation, sequence=self.sequence,
                    deadline_steady_ns=self.deadline_steady_ns,
                    remaining_sec=max(0, self.deadline_steady_ns - now_ns) / 1e9,
                    requires_new_generation=self.requires_new_generation,
                    reason=self.reason, last_arm_result=self.last_arm_result)
