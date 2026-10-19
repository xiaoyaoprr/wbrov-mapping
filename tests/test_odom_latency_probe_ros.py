"""Real DDS probe integration on an isolated, hardware-free ROS domain."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--message-info-directory', type=Path, required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '207'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import OdomInfo
    rclpy.init()
    node = rclpy.create_node('wbrov_latency_probe_test_publisher')
    publishers = [(node.create_publisher(kind, topic, 10), kind, topic)
                  for topic, kind in [('/odom', Odometry), ('/odom_info', OdomInfo)]]
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='wbrov-probe-test-') as folder:
            output = Path(folder)/'events.jsonl'
            process = subprocess.Popen([sys.executable, '-B', str(args.probe), str(output),
                                        '--seconds', '6', '--message-info-directory',
                                        str(args.message_info_directory)],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                for publisher, kind, _ in publishers:
                    message = kind()
                    message.header.stamp = node.get_clock().now().to_msg()
                    if kind is OdomInfo:
                        message.lost = True
                        message.inliers = 17
                        message.matches = 23
                    publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=.05)
            stdout, stderr = process.communicate(timeout=10)
            assert process.returncode == 0, stderr
            rows = [json.loads(line) for line in output.read_text().splitlines()]
            for _, _, topic in publishers:
                observed = [row for row in rows if row.get('topic') == topic and 'source_age_ns' in row]
                assert len(observed) > 10, (topic, len(observed))
                gids = {bytes(endpoint.endpoint_gid).hex()
                        for endpoint in node.get_publishers_info_by_topic(topic)
                        if endpoint.node_name == node.get_name()}
                assert gids and {row['publisher_gid'] for row in observed} == gids
                assert all(0 <= row['source_age_ns'] < 2_000_000_000 for row in observed)
                if topic == '/odom_info':
                    assert all(row['lost'] and row['inliers'] == 17 and row['matches'] == 23
                               for row in observed)
            print(json.dumps(dict(passed=True, actual_dds_messages=True, hardware_started=False,
                                  probe_summary=json.loads(stdout), records=len(rows))))
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
