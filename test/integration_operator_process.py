"""Check the installed stop process heartbeat and clean restart in domain 191."""

import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def test_installed_operator_starts_stopped_and_heartbeats(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '191'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init(args=[])
    observer = Node('operator_process_fixture')
    samples = []
    executable = (Path(get_package_prefix('jackal_nav2_bringup')) /
                  'lib/jackal_nav2_bringup/operator_stop.py')
    observer.create_subscription(Bool, '/nav2/operator_stop',
                                 lambda msg: samples.append((time.monotonic(), msg.data)), 1)
    try:
        for run in range(2):
            with (tmp_path / f'operator_{run}.log').open('w') as stream:
                child = subprocess.Popen([
                    str(executable),
                    '--ros-args', '-p', 'nuc_host:=127.0.0.1',
                ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    samples.clear()
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline and len(samples) < 30:
                        rclpy.spin_once(observer, timeout_sec=.05)
                        assert child.poll() is None
                    assert len(samples) >= 30
                    assert all(blocked for _, blocked in samples)
                    # Ignore discovery/transient startup; steady heartbeat is 20Hz.
                    span = samples[-1][0]-samples[-21][0]
                    assert .8 <= span <= 1.2
                finally:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=5)
                        raise
                assert child.returncode == 0, Path(stream.name).read_text()
    finally:
        observer.destroy_node()
        rclpy.try_shutdown()
