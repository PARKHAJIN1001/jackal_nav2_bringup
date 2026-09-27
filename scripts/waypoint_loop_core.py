"""ROS-independent state machine for two-point waypoint loop navigation."""

import math


def calculate_heading(from_pt, to_pt):
    """Calculate 2D yaw heading angle in radians from from_pt to to_pt."""
    return math.atan2(to_pt[1] - from_pt[1], to_pt[0] - from_pt[0])


class LoopState:
    """Pure state machine: collect two points, then shuttle between them."""

    VALID_POLICIES = ('nav2', 'other')

    def __init__(self, loop_count=5, driving_policy='nav2'):
        if not isinstance(loop_count, int) or loop_count < 1:
            raise ValueError(f'loop_count must be a positive integer, got {loop_count}')
        if driving_policy not in self.VALID_POLICIES:
            raise ValueError(f'driving_policy must be one of {self.VALID_POLICIES}')
        self.loop_count = loop_count
        self.driving_policy = driving_policy
        self.waypoints = []          # list of (x, y) tuples; max 2
        self.phase = 'COLLECTING'
        self.reason = 'waiting_for_waypoints'
        self.current_target = 0      # 0 = start, 1 = end
        self.completed_laps = 0
        self.operator_armed = False  # True when operator_stop phase == WAITING FOR NEW GOAL
        self.total_goals_sent = 0

    # ------------------------------------------------------------------
    # Waypoint collection
    # ------------------------------------------------------------------

    def add_waypoint(self, x, y):
        """Accept a new waypoint; return a human-readable status string."""
        if len(self.waypoints) >= 2 or self.phase != 'COLLECTING':
            # Reset waypoints: make this new point the new Start point (1/2)
            self.waypoints = [(float(x), float(y))]
            self.phase = 'COLLECTING'
            self.reason = 'waiting_for_second_waypoint'
            self.current_target = 0
            self.completed_laps = 0
            self.operator_armed = False
            return f'Reset! Waypoint 1/2 collected: ({x:.2f}, {y:.2f})'

        self.waypoints.append((float(x), float(y)))
        if len(self.waypoints) == 1:
            self.reason = 'waiting_for_second_waypoint'
            return f'Waypoint 1/2 collected: ({x:.2f}, {y:.2f})'

        self.phase = 'READY_FOR_START'
        self.reason = 'hold_triangle_to_move_to_start'
        return (
            f'Waypoint 2/2 collected: ({x:.2f}, {y:.2f}); '
            'hold triangle for 2s to move to Start point'
        )

    # ------------------------------------------------------------------
    # Operator arm detection
    # ------------------------------------------------------------------

    def get_target_goal(self):
        """Return (x, y, yaw) for current navigation target, or None."""
        if not self.waypoints or len(self.waypoints) < 2:
            return None
        start_pt = self.waypoints[0]
        end_pt = self.waypoints[1]
        if self.current_target == 0:
            target_pt = start_pt
            if self.phase == 'NAVIGATING_TO_START':
                yaw = calculate_heading(start_pt, end_pt)
            else:
                yaw = calculate_heading(end_pt, start_pt)
        else:
            target_pt = end_pt
            yaw = calculate_heading(start_pt, end_pt)
        return (target_pt[0], target_pt[1], yaw)

    def operator_arm(self):
        """Process operator activation via button hold; returns goal or None."""
        if self.phase == 'READY_FOR_START':
            self.phase = 'NAVIGATING_TO_START'
            self.reason = 'moving_to_start_point'
            self.current_target = 0
            self.total_goals_sent += 1
            return self.get_target_goal()
        if self.phase == 'AT_START_WAITING_FOR_LOOP':
            self.phase = 'NAVIGATING'
            self.reason = 'lap_in_progress'
            self.current_target = 1  # head to end point
            return self._next_goal()
        return None

    def operator_phase(self, phase_str):
        """Update from operator_stop diagnostics; returns next goal or None."""
        was_armed = self.operator_armed
        self.operator_armed = (phase_str == 'WAITING FOR NEW GOAL')
        # Only trigger on the rising edge of WAITING FOR NEW GOAL
        if self.operator_armed and not was_armed:
            return self.operator_arm()
        return None

    # ------------------------------------------------------------------
    # Goal completion
    # ------------------------------------------------------------------

    def goal_succeeded(self):
        """Process a successful goal and return the next goal or None."""
        if self.phase == 'NAVIGATING_TO_START':
            # Arrived at start point; pause and wait for the second button press!
            self.phase = 'AT_START_WAITING_FOR_LOOP'
            self.reason = 'hold_triangle_to_start_loop'
            self.current_target = 0
            return None

        if self.phase in ('NAVIGATING', 'WAITING_EXTERNAL'):
            reached = self.current_target
            if reached == 0:
                self.completed_laps += 1
                self.reason = f'lap_{self.completed_laps}/{self.loop_count}'
                if self.completed_laps >= self.loop_count:
                    self.phase = 'COMPLETED'
                    self.reason = f'all_{self.loop_count}_laps_completed'
                    return None
            self.current_target = 1 - reached
            return self._next_goal()

        return None

    def goal_failed(self, detail=''):
        """Record a navigation goal rejection or abort."""
        self.phase = 'ERROR'
        self.reason = f'goal_failed:{detail}' if detail else 'goal_failed'

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _next_goal(self):
        """Determine and return the next goal based on driving policy."""
        goal = self.get_target_goal()
        if self.driving_policy == 'nav2':
            self.total_goals_sent += 1
            return goal
        # 'other' policy: wait for external cmd_vel
        self.phase = 'WAITING_EXTERNAL'
        self.reason = 'waiting_for_external_cmd_vel'
        return None

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def summary(self):
        """Return a dict suitable for diagnostics or logging."""
        return {
            'phase': self.phase,
            'reason': self.reason,
            'waypoints': self.waypoints,
            'current_target': self.current_target,
            'completed_laps': self.completed_laps,
            'loop_count': self.loop_count,
            'driving_policy': self.driving_policy,
            'total_goals_sent': self.total_goals_sent,
        }
