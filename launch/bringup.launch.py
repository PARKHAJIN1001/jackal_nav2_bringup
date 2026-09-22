#!/usr/bin/env python3
"""Deprecated name for the single staged launch; use nav_session.py nav."""

import importlib.util
from pathlib import Path

from launch.actions import OpaqueFunction


def _reject_legacy(context):
    removed = {'use_lidar_relay', 'use_scan_projection', 'use_composition',
               'container_name', 'use_respawn'} & context.launch_configurations.keys()
    if removed:
        raise RuntimeError('Legacy external-FAST/composition options are unsupported: '
                           + ', '.join(sorted(removed)) +
                           '; use localization/navigation sublaunches for custom composition')
    return []


def generate_launch_description():
    path = Path(__file__).with_name('nav_bringup.launch.py')
    spec = importlib.util.spec_from_file_location('jackal_staged_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    # The legacy guard runs before any declarations or process actions.
    from launch import LaunchDescription
    return LaunchDescription([OpaqueFunction(function=_reject_legacy), *description.entities])
