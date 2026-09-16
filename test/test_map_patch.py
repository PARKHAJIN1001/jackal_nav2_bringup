"""Unit tests for robot-centric static costmap extraction."""

import importlib.util
import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / 'scripts' / 'map_patch_node.py'
SPEC = importlib.util.spec_from_file_location('map_patch_node', MODULE_PATH)
MAP_PATCH = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MAP_PATCH
SPEC.loader.exec_module(MAP_PATCH)


def _source_geometry(origin_yaw=0.0):
    half_extent = 2.5
    origin_x = (
        math.cos(origin_yaw) * -half_extent
        - math.sin(origin_yaw) * -half_extent
    )
    origin_y = (
        math.sin(origin_yaw) * -half_extent
        + math.cos(origin_yaw) * -half_extent
    )
    return MAP_PATCH.GridGeometry(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
    )


def _output_geometry():
    return MAP_PATCH.GridGeometry(
        width=3,
        height=3,
        resolution=1.0,
        origin_x=-1.5,
        origin_y=-1.5,
    )


def _numbered_grid():
    return [row * 10 + col for row in range(5) for col in range(5)]


def test_identity_pose_extracts_centered_patch():
    patch = MAP_PATCH.extract_robot_centric_patch(
        _numbered_grid(), _source_geometry(), 0.0, 0.0, 0.0,
        _output_geometry())

    assert patch == [11, 12, 13, 21, 22, 23, 31, 32, 33]


def test_robot_yaw_rotates_map_into_x_forward_frame():
    patch = MAP_PATCH.extract_robot_centric_patch(
        _numbered_grid(), _source_geometry(), 0.0, 0.0, math.pi / 2.0,
        _output_geometry())

    assert patch == [13, 23, 33, 12, 22, 32, 11, 21, 31]


def test_source_origin_yaw_is_respected():
    patch = MAP_PATCH.extract_robot_centric_patch(
        _numbered_grid(), _source_geometry(math.pi / 2.0),
        0.0, 0.0, math.pi / 2.0, _output_geometry())

    assert patch == [11, 12, 13, 21, 22, 23, 31, 32, 33]


def test_out_of_bounds_cells_are_unknown():
    patch = MAP_PATCH.extract_robot_centric_patch(
        _numbered_grid(), _source_geometry(), 2.0, 0.0, 0.0,
        _output_geometry())

    assert patch == [13, 14, -1, 23, 24, -1, 33, 34, -1]


def test_invalid_data_size_is_rejected():
    with pytest.raises(ValueError, match='data length'):
        MAP_PATCH.extract_robot_centric_patch(
            [0], _source_geometry(), 0.0, 0.0, 0.0,
            _output_geometry())


def test_quaternion_is_normalized_and_validated():
    assert MAP_PATCH.quaternion_to_yaw(0.0, 0.0, 2.0, 2.0) == pytest.approx(
        math.pi / 2.0)
    with pytest.raises(ValueError, match='non-zero'):
        MAP_PATCH.quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)
