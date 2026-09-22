"""ROS-independent, latched operator stop; a reset never resumes an old goal."""

import math


class OperatorState:
    def __init__(
        self,
        stop_button=-1,
        reset_button=-1,
        deadman_buttons=(),
        neutral_axes=(),
        mapping_verified=False,
    ):
        self.stop_button, self.reset_button = stop_button, reset_button
        self.deadman_buttons = tuple(deadman_buttons)
        self.neutral_axes = tuple(neutral_axes)
        self.verified = bool(
            mapping_verified
            and stop_button >= 0
            and reset_button >= 0
            and len({stop_button, reset_button, *deadman_buttons}) == 2 + len(deadman_buttons)
            and len(deadman_buttons) == 2
            and len(neutral_axes) > 0
            and all(math.isfinite(a) for a in neutral_axes)
        )
        self.phase, self.reason = 'STOPPED', 'startup'
        self.joy_at = self.link_at = self.odom_at = -math.inf
        self.link = False
        self.rest_since = None
        self.reset_since = None
        self.reset_released = False
        self.reset_pressed = self.stop_pressed = self.deadman = False
        self.neutral = False
        self.active = {}
        self.banned = set()
        self.armed_stamp = None
        self.last_ros = None

    def stop(self, reason):
        self.banned.update(self.active)
        self.phase, self.reason = 'STOPPED', reason
        self.armed_stamp = None
        if reason in ('circle', 'manual_takeover'):
            self.reset_since = None
            self.reset_released = False

    def joy(self, buttons, axes, now):
        needed = (self.stop_button, self.reset_button, *self.deadman_buttons)
        valid = (
            self.verified
            and (len(self.neutral_axes) == 0 or len(axes) >= len(self.neutral_axes))
            and all(0 <= i < len(buttons) for i in needed)
            and all(b in (0, 1) for b in buttons)
            and all(math.isfinite(a) for a in axes)
        )
        if not valid:
            self.stop('mapping_unverified_or_invalid_joy')
            return
        self.joy_at = now
        self.stop_pressed = bool(buttons[self.stop_button])
        self.reset_pressed = bool(buttons[self.reset_button])
        self.deadman = any(buttons[i] for i in self.deadman_buttons)
        if len(self.neutral_axes) > 0 and len(axes) >= len(self.neutral_axes):
            self.neutral = all(abs(a - b) <= 0.2 for a, b in zip(axes, self.neutral_axes))
        else:
            self.neutral = True
        if self.stop_pressed or self.deadman:
            self.stop('circle' if self.stop_pressed else 'manual_takeover')
        elif not self.reset_pressed:
            self.reset_released = True
            self.reset_since = None

    def connection(self, connected, now):
        self.link, self.link_at = connected, now
        if not connected:
            self.stop('bluetooth_disconnected')

    def odometry(self, linear, angular, now):
        if not 0 <= now - self.odom_at <= 0.3:
            self.rest_since = None
        self.odom_at = now
        if not (
            math.isfinite(linear)
            and math.isfinite(angular)
            and abs(linear) <= 0.02
            and abs(angular) <= 0.03
        ):
            self.rest_since = None
        elif self.rest_since is None:
            self.rest_since = now

    def goals(self, active):
        self.active = dict(active)  # UUID -> ROS acceptance nanoseconds
        if self.phase == 'STOPPED':
            self.banned.update(active)

    def tick(self, now, ros_stamp, infrastructure_ready):
        if self.last_ros is not None and ros_stamp < self.last_ros:
            self.stop('clock_regression')
        self.last_ros = ros_stamp

        if self.stop_pressed or self.deadman:
            self.stop('circle' if self.stop_pressed else 'manual_takeover')
            return True

        if self.phase == 'STOPPED':
            can_reset = self.reset_pressed and self.neutral
            if can_reset:
                if self.reset_since is None:
                    self.reset_since = now
                if now - self.reset_since >= 2.0:
                    if self.verified and 0 <= now - self.joy_at <= 1.0 and self.link:
                        self.phase, self.reason = 'WAITING FOR NEW GOAL', 'armed_waiting_for_goal'
                        self.armed_stamp = ros_stamp
                        self.reset_since = None
            else:
                self.reset_since = None
            return True

        healthy = (
            self.verified
            and 0 <= now - self.joy_at <= 1.0
            and self.link
            and 0 <= now - self.link_at <= 3.0
            and 0 <= now - self.odom_at <= 1.0
            and infrastructure_ready
        )
        if not healthy:
            if not self.verified:
                reason = 'mapping_unverified'
            elif not 0 <= now - self.joy_at <= 1.0:
                reason = 'joy_stale'
            elif not self.link:
                reason = 'bluetooth_disconnected'
            elif not 0 <= now - self.link_at <= 3.0:
                reason = 'bluetooth_status_stale'
            elif not 0 <= now - self.odom_at <= 1.0:
                reason = 'platform_odom_stale'
            else:
                reason = 'bridge_or_rearm_checks_pending'
            self.stop(reason)
            return True

        if self.active and all(
            k not in self.banned and stamp > self.armed_stamp for k, stamp in self.active.items()
        ):
            self.phase, self.reason = 'RUNNING', 'new_goal'
            return False
        if self.active:
            self.stop('old_goal_after_reset')
        else:
            self.phase, self.reason = 'WAITING FOR NEW GOAL', 'armed_waiting_for_goal'
        return True


class StopHeartbeat:

    def __init__(self):
        self.received = None
        self.blocked = True

    def receive(self, blocked, now):
        self.blocked, self.received = bool(blocked), now

    def required(self, now):
        return self.blocked or self.received is None or not 0 <= now - self.received <= 0.25
