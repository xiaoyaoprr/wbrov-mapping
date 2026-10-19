#!/usr/bin/env python3
"""Passive, separate-process odometry timing. No images/clouds/control publishers."""
import argparse
import collections
import json
import sys
import time
from pathlib import Path
from diagnostic_writer import DiagnosticWriter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--message-info-directory', type=Path, required=True,
                        help='verified private runtime libexec/wbrov_corner_recovery directory')
    args = parser.parse_args()
    if not 0 < args.seconds <= 1800:
        parser.error('seconds must be in (0, 1800]')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import OdomInfo
    # This robot's Humble executor discards MessageInfo before calling Python.
    # Reuse the reviewed adapter and its adjacent native library, rather than
    # guessing publisher identity or changing the installed ROS implementation.
    metadata_dir = args.message_info_directory.resolve(strict=True)
    sys.path.insert(0, str(metadata_dir))
    from message_info_executor import MessageInfoExecutor
    rclpy.init()
    node = Node('wbrov_isolated_odom_latency_probe', enable_rosout=False, start_parameter_services=False)
    executor = MessageInfoExecutor()
    executor.add_node(node)
    writer = None
    counts = collections.Counter()
    subscriptions = []
    begin = time.monotonic()

    def callback(topic, msg, info):
        entry = time.monotonic_ns()
        arrival = node.get_clock().now().nanoseconds
        source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        record = dict(topic=topic, source_ns=source, arrival_ros_ns=arrival,
                      callback_entry_steady_ns=entry, source_age_ns=arrival-source,
                      publisher_gid=bytes(info.publisher_gid).hex())
        if isinstance(msg, OdomInfo):
            record.update(lost=msg.lost, inliers=msg.inliers, matches=msg.matches,
                          icp_correspondences=msg.icp_correspondences, interval_s=msg.interval)
        record['callback_build_ns'] = time.monotonic_ns()-entry
        writer.append(record)
        writer.append(dict(kind='callback_timing',topic=topic,source_ns=source,
                           entry_steady_ns=entry,after_enqueue_steady_ns=time.monotonic_ns()))
        counts[topic] += 1

    try:
        writer = DiagnosticWriter(args.output)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        def receiver(topic):
            def receive(msg, info):
                callback(topic, msg, info)
            return receive
        for topic, kind in [('/odom', Odometry), ('/odom_info', OdomInfo)]:
            subscription = node.create_subscription(kind, topic, receiver(topic), qos)
            subscription._wbrov_message_info_required = True
            subscriptions.append(subscription)
        while rclpy.ok() and time.monotonic()-begin < args.seconds:
            if args.output.with_suffix('.stop').exists():
                break
            executor.spin_once(timeout_sec=.05)
        writer.close()
        print(json.dumps(dict(complete=True, seconds=time.monotonic()-begin, counts=dict(counts),
                              passive_only=True, scope='subscriber timing; GIDs recorded, not independently authenticated')))
    finally:
        try:
            if writer is not None:
                writer.close()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
