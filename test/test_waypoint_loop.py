"""Unit tests for waypoint_loop_core.LoopState."""

import math
import os
import sys

import pytest

# The core module lives next to the test, installed via FILES into lib/.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from waypoint_loop_core import calculate_heading, LoopState  # noqa: E402


class TestLoopStateInit:

    def test_defaults(self):
        s = LoopState()
        assert s.loop_count == 5
        assert s.driving_policy == 'nav2'
        assert s.phase == 'COLLECTING'

    def test_invalid_count(self):
        with pytest.raises(ValueError):
            LoopState(loop_count=0)
        with pytest.raises(ValueError):
            LoopState(loop_count=-1)

    def test_invalid_policy(self):
        with pytest.raises(ValueError):
            LoopState(driving_policy='unknown')


class TestWaypointCollection:

    def test_collect_two(self):
        s = LoopState()
        result = s.add_waypoint(1.0, 2.0)
        assert '1/2' in result
        assert s.phase == 'COLLECTING'
        result = s.add_waypoint(3.0, 4.0)
        assert '2/2' in result
        assert s.phase == 'READY_FOR_START'
        assert s.waypoints == [(1.0, 2.0), (3.0, 4.0)]

    def test_reset_waypoints_in_ready_state(self):
        s = LoopState()
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(1.0, 1.0)
        assert s.phase == 'READY_FOR_START'
        # Clicking again resets and sets as new first waypoint
        result = s.add_waypoint(2.0, 2.0)
        assert 'Reset' in result
        assert s.phase == 'COLLECTING'
        assert s.waypoints == [(2.0, 2.0)]
        # Adding second waypoint after reset
        result2 = s.add_waypoint(3.0, 3.0)
        assert '2/2' in result2
        assert s.phase == 'READY_FOR_START'
        assert s.waypoints == [(2.0, 2.0), (3.0, 3.0)]

    def test_reset_waypoints_in_navigating_state(self):
        s = LoopState()
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(1.0, 1.0)
        s.operator_arm()
        assert s.phase == 'NAVIGATING_TO_START'
        result = s.add_waypoint(5.0, 5.0)
        assert 'Reset' in result
        assert s.phase == 'COLLECTING'
        assert s.waypoints == [(5.0, 5.0)]

    def test_reset_waypoints_in_at_start_state(self):
        s = LoopState()
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(1.0, 1.0)
        s.operator_arm()
        s.goal_succeeded()
        assert s.phase == 'AT_START_WAITING_FOR_LOOP'
        result = s.add_waypoint(7.0, 7.0)
        assert 'Reset' in result
        assert s.phase == 'COLLECTING'
        assert s.waypoints == [(7.0, 7.0)]

    def test_reset_waypoints_in_completed_state(self):
        s = LoopState(loop_count=1)
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(1.0, 1.0)
        s.operator_arm()  # move to start
        s.goal_succeeded()  # at start
        s.operator_arm()  # to end
        s.goal_succeeded()  # to start
        s.goal_succeeded()  # complete
        assert s.phase == 'COMPLETED'
        result = s.add_waypoint(8.0, 8.0)
        assert 'Reset' in result
        assert s.phase == 'COLLECTING'
        assert s.waypoints == [(8.0, 8.0)]
        assert s.completed_laps == 0


class TestOperatorPhase:

    def _ready_state(self):
        s = LoopState(loop_count=2)
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(10.0, 10.0)
        return s

    def test_arm_sends_first_goal(self):
        s = self._ready_state()
        goal = s.operator_phase('WAITING FOR NEW GOAL')
        assert goal[:2] == (0.0, 0.0)
        assert goal[2] == pytest.approx(math.atan2(10.0, 10.0))
        assert s.phase == 'NAVIGATING_TO_START'

    def test_non_arm_phase_ignored(self):
        s = self._ready_state()
        assert s.operator_phase('STOPPED') is None
        assert s.phase == 'READY_FOR_START'

    def test_arm_before_ready_ignored(self):
        s = LoopState()
        s.add_waypoint(0.0, 0.0)
        assert s.operator_phase('WAITING FOR NEW GOAL') is None
        assert s.phase == 'COLLECTING'


class TestHeadingCalculation:
    """Test heading calculation helper."""

    def test_straight_east(self):
        yaw = calculate_heading((0.0, 0.0), (5.0, 0.0))
        assert yaw == pytest.approx(0.0)

    def test_straight_north(self):
        yaw = calculate_heading((0.0, 0.0), (0.0, 5.0))
        assert yaw == pytest.approx(math.pi / 2.0)

    def test_straight_west(self):
        yaw = calculate_heading((5.0, 0.0), (0.0, 0.0))
        assert abs(yaw) == pytest.approx(math.pi)

    def test_diagonal(self):
        yaw = calculate_heading((1.0, 1.0), (4.0, 4.0))
        assert yaw == pytest.approx(math.pi / 4.0)


class TestNav2FullLoop:
    """Test a complete nav2-policy loop with loop_count=2."""

    def _armed_state(self):
        s = LoopState(loop_count=2, driving_policy='nav2')
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(5.0, 5.0)
        s.operator_phase('WAITING FOR NEW GOAL')
        return s

    def test_full_cycle(self):
        s = self._armed_state()
        assert s.phase == 'NAVIGATING_TO_START'

        # Arrive at start → pauses and waits for second button press!
        goal = s.goal_succeeded()
        assert goal is None
        assert s.phase == 'AT_START_WAITING_FOR_LOOP'
        assert s.completed_laps == 0

        # Operator presses button second time → starts loop towards end (5, 5)
        goal = s.operator_arm()
        assert goal[:2] == (5.0, 5.0)
        assert goal[2] == pytest.approx(math.atan2(5.0, 5.0))
        assert s.phase == 'NAVIGATING'

        # Arrive at end → should head back to start (0, 0)
        goal = s.goal_succeeded()
        assert goal[:2] == (0.0, 0.0)
        assert goal[2] == pytest.approx(math.atan2(-5.0, -5.0))
        assert s.completed_laps == 0  # not a full lap yet

        # Arrive at start → lap 1 complete, head to end
        goal = s.goal_succeeded()
        assert goal[:2] == (5.0, 5.0)
        assert goal[2] == pytest.approx(math.atan2(5.0, 5.0))
        assert s.completed_laps == 1

        # Arrive at end → head back to start
        goal = s.goal_succeeded()
        assert goal[:2] == (0.0, 0.0)
        assert goal[2] == pytest.approx(math.atan2(-5.0, -5.0))

        # Arrive at start → lap 2 complete = COMPLETED
        goal = s.goal_succeeded()
        assert goal is None
        assert s.phase == 'COMPLETED'
        assert s.completed_laps == 2

    def test_total_goals(self):
        s = self._armed_state()
        count = 1  # initial goal to start
        goal = s.goal_succeeded()
        assert s.phase == 'AT_START_WAITING_FOR_LOOP'
        loop_goal = s.operator_arm()
        assert loop_goal is not None
        count += 1
        while s.phase not in ('COMPLETED', 'ERROR'):
            goal = s.goal_succeeded()
            if goal is not None:
                count += 1
        # loop_count=2: start(1) + end(2) + start(3) + end(4) + start(5) = 5
        assert count == 5
        assert s.total_goals_sent == 5


class TestOtherPolicy:
    """Test the 'other' driving policy."""

    def test_waits_for_external(self):
        s = LoopState(loop_count=1, driving_policy='other')
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(1.0, 1.0)
        s.operator_phase('WAITING FOR NEW GOAL')
        assert s.phase == 'NAVIGATING_TO_START'

        # Arrive at start → pauses and waits for second button press!
        goal = s.goal_succeeded()
        assert goal is None
        assert s.phase == 'AT_START_WAITING_FOR_LOOP'

        # Operator presses button second time → switches to WAITING_EXTERNAL
        goal = s.operator_arm()
        assert goal is None
        assert s.phase == 'WAITING_EXTERNAL'

        # External controller completes the trip to end point
        goal = s.goal_succeeded()
        assert goal is None
        assert s.phase == 'WAITING_EXTERNAL'  # waiting for return trip

        # External controller returns to start → lap 1 complete
        goal = s.goal_succeeded()
        assert goal is None
        assert s.phase == 'COMPLETED'
        assert s.completed_laps == 1


class TestGoalFailure:

    def test_failure_sets_error(self):
        s = LoopState(loop_count=1)
        s.add_waypoint(0.0, 0.0)
        s.add_waypoint(1.0, 1.0)
        s.operator_phase('WAITING FOR NEW GOAL')
        s.goal_failed('rejected')
        assert s.phase == 'ERROR'
        assert 'rejected' in s.reason


class TestSummary:

    def test_summary_keys(self):
        s = LoopState()
        summary = s.summary()
        expected_keys = {
            'phase', 'reason', 'waypoints', 'current_target',
            'completed_laps', 'loop_count', 'driving_policy', 'total_goals_sent',
        }
        assert set(summary.keys()) == expected_keys
