"""Pure, causal acceptance gates for AprilTag observations and delayed TF display.

No ROS, camera, controller, file I/O, pose smoothing, or replacement poses.
Continuity is not geometric truth: a consistently wrong PnP solution can pass.
"""
from dataclasses import dataclass
import math


def cadence_gap_s(expected_period_s, minimum_gap_s=0.60):
    """Use the configured producer cadence, never learn stalls as a new rate.

    One missing 1 Hz input exceeds 1.25 s and restarts confirmation. A detector
    frame without the Tag still invalidates its track immediately.
    """
    period, minimum = float(expected_period_s), float(minimum_gap_s)
    if not math.isfinite(period) or not 0 <= period <= 1.0:
        raise ValueError("expected Tag input period must be in [0,1] seconds")
    if not math.isfinite(minimum) or not 0 < minimum <= 1.25:
        raise ValueError("minimum Tag gap must be in (0,1.25] seconds")
    return max(minimum, 1.25 * period)


@dataclass(frozen=True)
class GuardConfig:
    confirm_frames: int = 3
    confirm_duration_s: float = 0.15
    max_gap_s: float = 0.60
    max_angular_rate_deg_s: float = 60.0
    angular_slack_deg: float = 6.0
    max_linear_rate_m_s: float = 0.50
    linear_slack_m: float = 0.015

    def __post_init__(self):
        if (isinstance(self.confirm_frames, bool) or
                not isinstance(self.confirm_frames, int) or
                not 2 <= self.confirm_frames <= 100):
            raise ValueError("confirm_frames must be an integer in [2,100]")
        for name in ("confirm_duration_s", "max_gap_s", "max_angular_rate_deg_s",
                     "angular_slack_deg", "max_linear_rate_m_s", "linear_slack_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + " must be finite and nonnegative")
        if not 0 < self.confirm_duration_s <= self.max_gap_s:
            raise ValueError("require 0 < confirm_duration_s <= max_gap_s")
        if self.max_angular_rate_deg_s <= 0 or self.max_linear_rate_m_s <= 0:
            raise ValueError("motion rates must be positive")
        if self.angular_slack_deg >= 180:
            raise ValueError("angular_slack_deg must be below 180")


@dataclass(frozen=True)
class _Observation:
    stamp: float
    translation: tuple
    quaternion: tuple


@dataclass
class _TagState:
    accepted: object = None
    pending_first: object = None
    pending_last: object = None
    pending_count: int = 0
    last_stamp: object = None

    def clear_pending(self):
        self.pending_first = self.pending_last = None
        self.pending_count = 0


def _observation(stamp, translation, quaternion):
    stamp = float(stamp)
    t, q = tuple(float(x) for x in translation), tuple(float(x) for x in quaternion)
    if (len(t) != 3 or len(q) != 4 or not math.isfinite(stamp) or stamp < 0 or
            not all(math.isfinite(x) for x in t + q)):
        raise ValueError("invalid numeric observation")
    norm = math.hypot(*q)
    if norm < 1e-12 or abs(norm-1.0) > 0.05:
        raise ValueError("invalid quaternion norm")
    return _Observation(stamp, t, tuple(x/norm for x in q))


def _difference(a, b):
    distance = math.hypot(*(x-y for x, y in zip(a.translation, b.translation)))
    dot = min(1.0, abs(sum(x*y for x, y in zip(a.quaternion, b.quaternion))))
    angle = math.degrees(2*math.acos(dot))
    return distance, angle


class TemporalPoseGuard:
    """Confirm new tracks; reject spikes without publishing or mutating old poses.

    After every detector frame, call finish_frame(stamp, valid_geometry_ids).
    A missing/invalid observation breaks confirmation. A rejected pose never
    advances the accepted reference. The motion allowance uses the *adjacent*
    observation interval, not an ever-growing interval since last acceptance.
    A coherent new track can be reacquired after the normal confirmation period.
    """
    def __init__(self, config=None):
        self.config = config or GuardConfig()
        self._states = {}
        self._context = None
        self._context_set = False

    def reset(self):
        self._states.clear()
        self._context = None
        self._context_set = False

    def invalidate(self, tag_id):
        self._states.pop(int(tag_id), None)

    def finish_frame(self, stamp_s, observed_ids):
        # The caller passes geometry-valid IDs, including temporally rejected IDs.
        observed = {int(x) for x in observed_ids}
        for tag_id in list(self._states):
            if tag_id not in observed:
                self.invalidate(tag_id)

    def _compatible(self, a, b, interval):
        distance, angle = _difference(a, b)
        interval = max(0.0, min(float(interval), self.config.max_gap_s))
        allowed_distance = self.config.linear_slack_m + self.config.max_linear_rate_m_s*interval
        allowed_angle = self.config.angular_slack_deg + self.config.max_angular_rate_deg_s*interval
        return (distance <= allowed_distance + 1e-10 and angle <= allowed_angle + 1e-8,
                {"translation_delta_m": distance, "angular_delta_deg": angle,
                 "translation_limit_m": allowed_distance, "angular_limit_deg": allowed_angle})

    def update(self, tag_id, stamp_s, translation_xyz, quaternion_xyzw, context=None):
        if not self._context_set or context != self._context:
            self._states.clear()
            self._context, self._context_set = context, True
        try:
            valid_id = not isinstance(tag_id, bool) and int(tag_id) == tag_id and int(tag_id) >= 0
        except (ValueError, TypeError, OverflowError):
            valid_id = False
        if not valid_id:
            return {"accepted": False, "reason": "invalid_tag_id"}
        tag_id = int(tag_id)
        try:
            observation = _observation(stamp_s, translation_xyz, quaternion_xyzw)
        except (ValueError, TypeError, OverflowError):
            self.invalidate(tag_id)
            return {"accepted": False, "reason": "invalid_pose"}
        state = self._states.setdefault(tag_id, _TagState())
        if state.last_stamp is not None:
            dt = observation.stamp - state.last_stamp
            if dt == 0:
                return {"accepted": False, "reason": "duplicate_timestamp"}
            if dt < 0:
                self._states[tag_id] = _TagState(last_stamp=observation.stamp)
                return {"accepted": False, "reason": "time_regression"}
            if dt > self.config.max_gap_s:
                state = self._states[tag_id] = _TagState()
        else:
            dt = 0.0
        state.last_stamp = observation.stamp
        details = {}
        if state.accepted is not None:
            accepted_age = observation.stamp - state.accepted.stamp
            if accepted_age <= self.config.max_gap_s:
                compatible, details = self._compatible(
                    state.accepted, observation, min(dt, accepted_age))
                if compatible:
                    state.accepted = observation
                    state.clear_pending()
                    return {"accepted": True, "reason": "tracking", **details}
            else:
                # Never silently relax a jump after a long rejection sequence.
                state.accepted = None
                state.clear_pending()
        was_tracking = state.accepted is not None
        if (state.pending_last is None or
                not self._compatible(state.pending_last, observation,
                                     observation.stamp-state.pending_last.stamp)[0]):
            state.pending_first = state.pending_last = observation
            state.pending_count = 1
        else:
            state.pending_last = observation
            state.pending_count += 1
        span = observation.stamp - state.pending_first.stamp
        if (state.pending_count >= self.config.confirm_frames and
                span + 1e-9 >= self.config.confirm_duration_s):
            count = state.pending_count
            state.accepted = observation
            state.clear_pending()
            return {"accepted": True, "reason": "reacquired" if was_tracking else "confirmed",
                    "confirmation_samples": count, "confirmation_span_s": span, **details}
        return {"accepted": False, "reason": "jump_pending" if was_tracking else "confirming",
                "confirmation_samples": state.pending_count, "confirmation_span_s": span, **details}


class PendingTransformQueue:
    """Bounded, monotonic-age queue. resolver(payload) is a nonblocking lookup.

    A resolver returns None while the exact-time transform is unavailable.
    Expired/older results are discarded, never retried with a latest transform.
    """
    def __init__(self, capacity=32, max_age_s=0.5):
        if (isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1 or
                not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError("invalid transform queue bounds")
        self.capacity, self.max_age_s = capacity, float(max_age_s)
        self.items = []
        self.last_delivered_stamp = {}
        self.expired = self.overflow = self.superseded = 0

    def clear(self):
        self.items.clear()
        self.last_delivered_stamp.clear()

    def discard_tags(self, tag_ids):
        removed = {int(x) for x in tag_ids}
        self.items = [item for item in self.items if item["tag_id"] not in removed]
        for tag_id in removed:
            self.last_delivered_stamp.pop(tag_id, None)

    def enqueue(self, tag_id, stamp_ns, payload, now_monotonic):
        tag_id, stamp_ns = int(tag_id), int(stamp_ns)
        now_monotonic = float(now_monotonic)
        if stamp_ns <= 0 or not math.isfinite(now_monotonic):
            raise ValueError("invalid transform queue timestamp")
        if stamp_ns <= self.last_delivered_stamp.get(tag_id, -1):
            self.superseded += 1
            return
        if any(i["tag_id"] == tag_id and i["stamp_ns"] == stamp_ns for i in self.items):
            return
        if len(self.items) >= self.capacity:
            self.items.pop(0)
            self.overflow += 1
        self.items.append({"tag_id": tag_id, "stamp_ns": stamp_ns,
                           "queued_monotonic": now_monotonic, "payload": payload})

    def resolve_ready(self, now_monotonic, resolver):
        now_monotonic = float(now_monotonic)
        if not math.isfinite(now_monotonic):
            raise ValueError("invalid monotonic clock")
        pending, resolved = [], []
        for item in self.items:
            age = now_monotonic-item["queued_monotonic"]
            if age < 0 or age >= self.max_age_s:
                self.expired += 1
                continue
            if item["stamp_ns"] <= self.last_delivered_stamp.get(item["tag_id"], -1):
                self.superseded += 1
                continue
            value = resolver(item["payload"])
            if value is None:
                pending.append(item)
            else:
                self.last_delivered_stamp[item["tag_id"]] = item["stamp_ns"]
                resolved.append({**item, "resolved": value, "age_s": age})
        self.items = pending
        # If multiple are ready in one tick, only expose the newest per tag.
        newest = {}
        for item in resolved:
            newest[item["tag_id"]] = item
        return list(newest.values())
