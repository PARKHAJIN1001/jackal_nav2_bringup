"""Shared ROS-independent freshness and continuous readiness state."""

import math


def fresh(stamp, receipt, now, monotonic, timeout=0.3, future=0.05):
    """Measurement time and receipt time must both be valid."""
    return (math.isfinite(stamp) and stamp > 0 and
            -future <= now - stamp <= timeout and
            0 <= monotonic - receipt <= timeout)


def valid_transform(transform):
    """Reject nonfinite and non-unit transforms even when timestamps are fresh."""
    t, q = transform.translation, transform.rotation
    return (all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)) and
            abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) <= 0.01)


class StabilityWindow:
    """Continuous full-stack evidence, including bounded recovery after a fault."""

    def __init__(self, timeout=600.0, settle=180.0):
        if not all(math.isfinite(v) and v > 0 for v in (timeout, settle)):
            raise ValueError('stability times must be finite and positive')
        if settle >= timeout:
            raise ValueError('stability settle must be less than timeout')
        self.timeout, self.settle = timeout, settle
        self.started = self.since = self.last = None
        self.settled = self.pose_seen = False
        self.phase = 'INPUT_WAITING'
        self.resets = 0

    def update(self, healthy, pose_ready, now):
        if self.phase == 'FAILED':
            return self.phase
        if not math.isfinite(now):
            raise ValueError('monotonic time must be finite')
        if self.started is None:
            self.started = now
        # A stalled monitor cannot credit time it did not observe.
        if self.last is not None and not 0 <= now - self.last <= 0.3:
            healthy = False
        self.last = now
        if self.pose_seen and not pose_ready:
            healthy = False
        if not healthy:
            if self.settled:
                self.started = now  # new bounded recovery period
            if self.since is not None:
                self.resets += 1
            self.since = None
            self.settled = self.pose_seen = False
            self.phase = 'INPUT_WAITING'
        else:
            if self.since is None:
                self.since = now
            self.settled = self.settled or now >= self.since + self.settle
            self.pose_seen = self.settled and pose_ready
            self.phase = ('READY' if self.pose_seen else 'INITIAL_POSE_REQUIRED'
                          if self.settled else 'STABILIZING')
        if not self.settled and now >= self.started + self.timeout:
            self.phase = 'FAILED'
        return self.phase


class FreshWindow:
    """A bounded state machine; publisher discovery is never readiness evidence."""

    def __init__(self, settle, max_age, max_gap, future_tolerance, min_messages):
        for name, value in [('settle', settle), ('max_age', max_age), ('max_gap', max_gap)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('future_tolerance must be finite and nonnegative')
        if min_messages < 2:
            raise ValueError('min_messages must be at least 2')
        self.settle, self.max_age, self.max_gap = settle, max_age, max_gap
        self.future_tolerance, self.min_messages = future_tolerance, min_messages
        self.reset('no messages received')

    def reset(self, reason):
        self.resets = getattr(self, 'resets', 0) + 1
        self.first = self.last = self.stamp = None
        self.count = 0
        self.reason = reason

    def observe(self, stamp, ros_now, steady_now, error=''):
        if error:
            self.reset(error)
            return
        if not math.isfinite(stamp) or stamp <= 0:
            self.reset('invalid measurement time')
            return
        age = ros_now - stamp
        if not -self.future_tolerance <= age <= self.max_age:
            self.reset('stale or future measurement')
            return
        if self.last is not None and (
                steady_now - self.last > self.max_gap or
                not 0 < stamp - self.stamp <= self.max_gap):
            self.reset('measurement gap or timestamp regression')
        if self.first is None:
            self.first = steady_now
        self.last, self.stamp = steady_now, stamp
        self.count += 1
        if self.count > 1:
            self.reason = 'collecting continuous measurements'

    def ready(self, ros_now, steady_now):
        if self.last is None:
            return False
        if (steady_now - self.last > self.max_gap or
                not -self.future_tolerance <= ros_now - self.stamp <= self.max_age):
            self.reset('input stopped or became stale')
            return False
        return self.count >= self.min_messages and self.last - self.first >= self.settle
