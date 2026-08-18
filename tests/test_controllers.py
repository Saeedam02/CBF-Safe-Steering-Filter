import math

import pytest

from cbf_safe_steering.controllers import GoalSeekingController
from cbf_safe_steering.models import VehicleState
from cbf_safe_steering.utils import wrap_angle


def test_goal_seeking_controller_points_toward_goal():
    controller = GoalSeekingController(heading_gain=1.0)
    command = controller.command(VehicleState(0.0, 0.0, 0.0), (1.0, 1.0), 2.0)
    assert command == pytest.approx(math.pi / 4.0)


def test_goal_seeking_controller_respects_yaw_rate_limit():
    controller = GoalSeekingController(heading_gain=10.0)
    command = controller.command(VehicleState(0.0, 0.0, 0.0), (-1.0, 0.0), 0.3)
    assert abs(command) == pytest.approx(0.3)


def test_wrap_angle_range_and_periodicity():
    values = [-9.0, -math.pi, -0.1, 0.0, math.pi, 9.0]
    for angle in values:
        wrapped = wrap_angle(angle)
        assert -math.pi <= wrapped < math.pi
        assert wrap_angle(angle + 2.0 * math.pi) == pytest.approx(wrapped)
