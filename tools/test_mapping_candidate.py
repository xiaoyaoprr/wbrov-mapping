#!/usr/bin/env python3
"""Run the candidate's Python behavior tests without ROS nodes or hardware."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-opencv', action='store_true',
                        help='Fail if the three numerical projection tests would be skipped.')
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if options.require_opencv:
        subprocess.run([sys.executable, '-B', '-c', 'import numpy, cv2'], check=True)
    suites = (
        ('src/visualization', 'test_submap_display_geometry.py'),
        ('runtime/libexec/wbrov_corner_recovery', 'test_observing_ack.py'),
        ('runtime/libexec/wbrov_corner_recovery', 'test_tag_visualization.py'),
        ('tests/recorded_replay', 'test_*audit.py'),
    )
    for folder, pattern in suites:
        result = subprocess.run([sys.executable, '-B', '-m', 'unittest', 'discover',
            '-s', str(root / folder), '-p', pattern, '-v'], cwd=root)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == '__main__':
    sys.exit(main())
