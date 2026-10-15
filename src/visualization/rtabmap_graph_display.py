#!/usr/bin/env python3
"""Read /mapGraph and publish a stateless, unambiguous visualization only.

No camera/SDK/control imports, TF publication, SLAM services, or parameter
changes. --check-only imports dependencies and checks syntax/messages without
initializing rclpy or constructing a Node. Source the normal ROS environment.
"""

import sys
if __name__ == "__main__":
    # Neither preflight nor the visualization process needs bytecode files.
    sys.dont_write_bytecode = True

import json
import math
import os
import stat
import time
from pathlib import Path

import graph_display_geometry as geometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from rtabmap_msgs.msg import MapGraph, OdomInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


INPUT_TOPIC = "/mapGraph"
OUTPUT_TOPIC = "/mapping/graph_display"


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def vector(value, keys=("x", "y", "z")):
    return [finite(getattr(value, key)) for key in keys]


def transform(value):
    return {"translation": vector(value.translation),
            "quaternion_xyzw": vector(value.rotation, ("x", "y", "z", "w"))}


class PoseDiagnostics:
    """Bounded passive telemetry; never feeds a pose back into ROS or control."""
    MAX_BYTES = 128 * 1024 * 1024
    DURATION_NS = 1800 * 1_000_000_000

    def __init__(self, path):
        path = Path(path)
        parent = path.parent
        info = parent.stat()
        if (not path.is_absolute() or parent.resolve() != parent or
                not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or
                stat.S_IMODE(info.st_mode) & 0o077):
            raise ValueError("pose diagnostics requires an owned private directory")
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        self.stream = os.fdopen(fd, "w", encoding="utf-8", buffering=65536)
        self.start_ns = time.monotonic_ns()
        self.sequence = 0
        self.bytes = 0
        self.counts = {}
        self.write({"kind": "start", "schema_version": 1,
                    "duration_limit_s": self.DURATION_NS // 1_000_000_000, "bytes_limit": self.MAX_BYTES,
                    "pose_feedback": False, "timestamps_modified": False,
                    "publisher_identity_authenticated": False,
                    "purpose": "match exact header stamps against authenticated observer logs; endpoint snapshots do not authenticate individual messages"})

    def write(self, row):
        if self.stream is None:
            return
        row.update(sequence=self.sequence, arrival_monotonic_ns=time.monotonic_ns(),
                   arrival_wall_ns=time.time_ns())
        data = json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n"
        self.stream.write(data)
        self.bytes += len(data.encode("utf-8"))
        self.sequence += 1

    def tick(self):
        if self.stream is None:
            return False
        if time.monotonic_ns() - self.start_ns >= self.DURATION_NS:
            self.close("duration_limit")
        elif self.bytes >= self.MAX_BYTES - 65536:
            self.close("size_limit")
        else:
            self.stream.flush()
        return self.stream is not None

    def record(self, topic, message):
        if self.stream is None:
            return
        if time.monotonic_ns() - self.start_ns >= self.DURATION_NS or self.bytes >= self.MAX_BYTES - 65536:
            self.tick()
            return
        if topic == "/tf":
            selected = [t for t in message.transforms if (t.header.frame_id, t.child_frame_id)
                        in (("map", "odom"), ("odom", "base_link"))]
        else:
            selected = [message]
        for item in selected:
            row = {"kind": "sample", "topic": topic,
                   "stamp_ns": item.header.stamp.sec * 1_000_000_000 + item.header.stamp.nanosec,
                   "frame_id": item.header.frame_id,
                   "publisher_gid": None,
                   "publisher_identity_authenticated": False}
            if topic == "/odom":
                row.update(child_frame_id=item.child_frame_id, position=vector(item.pose.pose.position),
                           quaternion_xyzw=vector(item.pose.pose.orientation, ("x", "y", "z", "w")),
                           covariance_diagonal=[finite(item.pose.covariance[i]) for i in (0, 7, 14, 21, 28, 35)],
                           linear_velocity=vector(item.twist.twist.linear), angular_velocity=vector(item.twist.twist.angular))
            elif topic == "/zed/imu/data":
                row.update(quaternion_xyzw=vector(item.orientation, ("x", "y", "z", "w")),
                           angular_velocity=vector(item.angular_velocity), linear_acceleration=vector(item.linear_acceleration),
                           orientation_covariance=[finite(v) for v in item.orientation_covariance])
            elif topic == "/odom_info_lite":
                row.update(lost=item.lost, inliers=item.inliers, matches=item.matches, features=item.features,
                           estimation_s=finite(item.time_estimation), interval_s=finite(item.interval),
                           gravity_roll_error=finite(item.gravity_roll_error), gravity_pitch_error=finite(item.gravity_pitch_error),
                           transform=transform(item.transform), guess=transform(item.guess))
            else:
                row.update(child_frame_id=item.child_frame_id, transform=transform(item.transform))
            self.counts[topic] = self.counts.get(topic, 0) + 1
            self.write(row)

    def close(self, reason="process_exit"):
        if self.stream is not None:
            self.write({"kind": "stop", "reason": reason, "counts": self.counts,
                        "elapsed_s": (time.monotonic_ns() - self.start_ns) / 1e9})
            self.stream.close()
            self.stream = None


def graph_snapshot(message):
    """Copy only fields used by the pure geometry builder; no frame conversion."""
    return {
        "header": {"frame_id": message.header.frame_id,
                   "stamp": {"sec": message.header.stamp.sec,
                             "nanosec": message.header.stamp.nanosec}},
        "poses_id": list(message.poses_id),
        "poses": [{"position": {k: getattr(p.position, k) for k in ("x", "y", "z")},
                   "orientation": {k: getattr(p.orientation, k) for k in ("x", "y", "z", "w")}}
                  for p in message.poses],
        "links": [{"from_id": e.from_id, "to_id": e.to_id, "type": e.type}
                  for e in message.links],
    }


def marker_array(specs):
    message = MarkerArray()
    for spec in specs:
        marker = Marker()
        marker.header.frame_id = spec.frame_id
        marker.header.stamp.sec = spec.stamp_sec
        marker.header.stamp.nanosec = spec.stamp_nanosec
        marker.ns = spec.namespace
        marker.id = spec.marker_id
        marker.action = Marker.DELETEALL if spec.kind == "clear" else Marker.ADD
        marker.type = {"clear": Marker.CUBE, "line_list": Marker.LINE_LIST,
                       "text": Marker.TEXT_VIEW_FACING}[spec.kind]
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec.position
        marker.scale.x, marker.scale.y, marker.scale.z = spec.scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec.color
        marker.frame_locked = False
        marker.text = spec.text
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in spec.points]
        # Default lifetime=0: persist as map history until the next DELETEALL.
        message.markers.append(marker)
    return message


class GraphDisplayNode(Node):
    def __init__(self):
        super().__init__("rtabmap_graph_display", start_parameter_services=False)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, OUTPUT_TOPIC, qos)
        self.graph_sub = self.create_subscription(MapGraph, INPUT_TOPIC, self.on_graph, qos)
        self.rejected_snapshots = 0
        self.pose_diagnostics = None
        path = os.environ.get("WBROV_POSE_DIAGNOSTICS_PATH")
        if path:
            self.pose_diagnostics = PoseDiagnostics(path)
            passive_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            self.diagnostic_subs = []
            # This installed Humble rclpy supplies only the message, no MessageInfo.
            # Never claim that a graph endpoint snapshot authenticates its sender.
            def on_odom(message):
                self.pose_diagnostics.record("/odom", message)
            def on_imu(message):
                self.pose_diagnostics.record("/zed/imu/data", message)
            def on_info(message):
                self.pose_diagnostics.record("/odom_info_lite", message)
            def on_tf(message):
                self.pose_diagnostics.record("/tf", message)
            for msg_type, topic, callback in [(Odometry, "/odom", on_odom), (Imu, "/zed/imu/data", on_imu),
                                               (OdomInfo, "/odom_info_lite", on_info), (TFMessage, "/tf", on_tf)]:
                self.diagnostic_subs.append(self.create_subscription(msg_type, topic, callback, passive_qos))
            def diagnostic_tick():
                if self.pose_diagnostics.tick():
                    endpoints = {}
                    for topic in ("/odom", "/odom_info_lite", "/zed/imu/data", "/tf"):
                        endpoints[topic] = [{"node": p.node_name, "namespace": p.node_namespace,
                                             "gid": bytes(p.endpoint_gid).hex()}
                                            for p in self.get_publishers_info_by_topic(topic)]
                    self.pose_diagnostics.write({"kind": "publisher_endpoints", "endpoints": endpoints,
                                                 "individual_message_identity_authenticated": False})
            self.diagnostic_timer = self.create_timer(1.0, diagnostic_tick)
            self.get_logger().info("Passive pose diagnostics: first 1800 seconds, maximum 128 MiB: " + path)
        self.get_logger().info(
            "Visualization only: /mapGraph -> /mapping/graph_display; "
            "positive-node neighbor edges 0/6; Tag N labels are map history, not live detections.")

    def on_graph(self, message):
        try:
            snapshot = geometry.build_graph_geometry(graph_snapshot(message))
        except (geometry.GraphValidationError, AttributeError, TypeError, OverflowError) as error:
            self.marker_pub.publish(marker_array(geometry.clear_marker_specs()))
            self.rejected_snapshots += 1
            # Bound log volume without keeping any old graph geometry.
            if self.rejected_snapshots == 1 or self.rejected_snapshots % 20 == 0:
                self.get_logger().warning("Graph display cleared: " + str(error))
            return
        self.marker_pub.publish(marker_array(geometry.marker_specs(snapshot)))


def check_only():
    for source in (Path(__file__), Path(geometry.__file__)):
        compile(source.read_bytes(), str(source), "exec", dont_inherit=True)
    empty = MapGraph()
    empty.header.frame_id = "map"
    result = marker_array(geometry.marker_specs(geometry.build_graph_geometry(graph_snapshot(empty))))
    assert len(result.markers) == 1 and result.markers[0].action == Marker.DELETEALL
    print(json.dumps({"status": "PASS", "mode": "check-only",
        "input": INPUT_TOPIC, "output": OUTPUT_TOPIC,
        "qos": "reliable,transient_local,keep_last:1",
        "ros_dependencies_imported": True, "rclpy_initialized": False,
        "node_created": False, "hardware_started": False}))


def main(args=None):
    effective = sys.argv[1:] if args is None else list(args)
    if "--check-only" in effective:
        if effective != ["--check-only"]:
            raise SystemExit("Use --check-only without other arguments")
        check_only()
        return
    rclpy.init(args=args)
    node = None
    try:
        node = GraphDisplayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.pose_diagnostics is not None:
                node.pose_diagnostics.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
