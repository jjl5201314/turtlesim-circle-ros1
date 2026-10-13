#!/usr/bin/env python3
"""Check actual ROS integration outputs; called only by the ROS1 workflow."""
import csv
import math
from pathlib import Path
import sys


def verify(directory, mode):
    def rows(name):
        with (directory / name).open(newline='', encoding='utf-8') as source:
            return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(source)]

    commands = rows('cmd_vel.csv')
    poses = rows('pose.csv')
    expected_v = 1.0 if mode == 'speed' else 2.0 * math.pi / 8.0
    expected_w = 2.0 * math.pi / 8.0
    moving = [row for row in commands if row['linear_x'] > 0.01]
    assert len(moving) > 100, 'Insufficient moving command samples'
    assert len(poses) > 100, 'Insufficient turtle pose samples'
    assert all(abs(row['linear_x'] - expected_v) < 1e-6 for row in moving)
    assert all(abs(row['angular_z'] - expected_w) < 1e-6 for row in moving)
    assert all(abs(commands[-1][field]) < 1e-9 for field in ['linear_x', 'angular_z']), 'Missing final stop'
    assert 7.5 < moving[-1]['time_s'] - moving[0]['time_s'] < 8.3, 'Unexpected motion duration'
    moving_poses = [row for row in poses if row['linear_velocity'] > 0.01]
    assert len(moving_poses) > 100, 'Turtlesim did not move'
    assert max(abs(row['linear_velocity'] - expected_v) for row in moving_poses) < 0.02
    assert max(abs(row['angular_velocity'] - expected_w) for row in moving_poses) < 0.02
    displacement = math.hypot(poses[-1]['x'] - poses[0]['x'], poses[-1]['y'] - poses[0]['y'])
    assert displacement < 0.4, 'One-circle path did not approximately close'
    assert max(row['x'] for row in poses) - min(row['x'] for row in poses) > 1.8
    print('PASS: real ROS recording, mode={}, {} command samples, {} pose samples, closure={:.4f}'.format(mode, len(commands), len(poses), displacement))


if __name__ == '__main__':
    verify(Path(sys.argv[1]), sys.argv[2])
