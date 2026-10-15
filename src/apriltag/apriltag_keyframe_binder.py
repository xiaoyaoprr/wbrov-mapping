#!/usr/bin/env python3
"""Bind geometry-valid AprilTag observations to exact RTAB-Map keyframes.

This node is observer-only. It has subscriptions and local 0600 outputs, but no
publisher, service client, parameter client, database access, or graph mutation.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
from pathlib import Path
import stat
import sys
import time


SCHEMA_VERSION = 1
O_CLOEXEC = getattr(os, "O_CLOEXEC", 0)
O_NOFOLLOW = getattr(os, "O_NOFOLLOW", 0)


class BinderError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise BinderError(message)


def valid_finite_sequence(values, count):
    return (
        isinstance(values, (list, tuple))
        and len(values) == count
        and all(isinstance(value, (int, float)) and math.isfinite(value)
                for value in values)
    )


class ExactKeyframeBinder:
    """Order-independent, exact-stamp join with explicit reset boundaries."""

    def __init__(self, max_pending_age_s=2.5):
        require(math.isfinite(max_pending_age_s) and 0.1 <= max_pending_age_s <= 10.0,
                "max pending age must be in [0.1, 10.0] seconds")
        self.max_pending_age_s = float(max_pending_age_s)
        self.pending_tags = {}
        self.pending_infos = {}
        self.reset_epoch = 0
        self.odom_lost = False
        self.accepted = 0
        self.rejected = 0
        self.duplicate = 0

    @staticmethod
    def _validate_stamp(stamp_ns):
        require(isinstance(stamp_ns, int) and stamp_ns > 0,
                "stamp must be a positive integer nanosecond value")

    def add_tags(self, stamp_ns, observations, now):
        self._validate_stamp(stamp_ns)
        require(math.isfinite(now), "receipt time must be finite")
        require(isinstance(observations, list) and observations,
                "tag message must contain at least one observation")
        normalized = []
        seen = set()
        for observation in observations:
            require(isinstance(observation, dict), "observation must be an object")
            tag_id = observation.get("tag_id")
            require(isinstance(tag_id, int) and tag_id >= 0, "invalid tag ID")
            require(tag_id not in seen, "duplicate tag ID in one image")
            seen.add(tag_id)
            require(valid_finite_sequence(observation.get("camera_translation_m"), 3),
                    "invalid camera translation")
            quaternion = observation.get("camera_quaternion_xyzw")
            require(valid_finite_sequence(quaternion, 4), "invalid camera quaternion")
            norm = math.sqrt(sum(value * value for value in quaternion))
            require(abs(norm - 1.0) <= 0.05, "camera quaternion is not near unit")
            require(isinstance(observation.get("camera_frame"), str) and
                    observation["camera_frame"], "missing camera frame")
            for name in ("range_m", "reprojection_px", "image_u_norm",
                         "image_v_norm", "mean_side_px"):
                require(isinstance(observation.get(name), (int, float)) and
                        math.isfinite(observation[name]), f"invalid {name}")
            require(observation["range_m"] > 0.0 and
                    observation["reprojection_px"] >= 0.0 and
                    observation["mean_side_px"] > 0.0,
                    "invalid positive view geometry")
            require(0.0 <= observation["image_u_norm"] <= 1.0 and
                    0.0 <= observation["image_v_norm"] <= 1.0,
                    "image center is outside normalized image bounds")
            normalized.append(dict(observation))
        if stamp_ns in self.pending_tags:
            if self.pending_tags[stamp_ns]["observations"] == normalized:
                self.duplicate += 1
                return []
            raise BinderError("conflicting tag payload for one image stamp")
        self.pending_tags[stamp_ns] = {
            "receipt_monotonic_s": float(now),
            "observations": normalized,
            "reset_epoch": self.reset_epoch,
        }
        return self._match(stamp_ns, now)

    def add_info(self, stamp_ns, ref_id, map_id, now):
        self._validate_stamp(stamp_ns)
        require(isinstance(ref_id, int), "Info.ref_id must be an integer")
        require(isinstance(map_id, int) and map_id >= 0,
                "map ID must be a nonnegative integer")
        require(math.isfinite(now), "receipt time must be finite")
        if ref_id <= 0:
            self.rejected += 1
            return [{
                "accepted": False,
                "reason": "nonpositive_info_ref_id",
                "image_stamp_ns": stamp_ns,
                "keyframe_id": ref_id,
                "map_id": map_id,
                "reset_epoch": self.reset_epoch,
            }]
        payload = {
            "receipt_monotonic_s": float(now),
            "ref_id": ref_id,
            "map_id": map_id,
            "reset_epoch": self.reset_epoch,
        }
        if stamp_ns in self.pending_infos:
            old = self.pending_infos[stamp_ns]
            if old["ref_id"] == ref_id and old["map_id"] == map_id:
                self.duplicate += 1
                return []
            raise BinderError("conflicting RTAB-Map Info for one image stamp")
        self.pending_infos[stamp_ns] = payload
        return self._match(stamp_ns, now)

    def _match(self, stamp_ns, now):
        if stamp_ns not in self.pending_tags or stamp_ns not in self.pending_infos:
            return []
        tags = self.pending_tags.pop(stamp_ns)
        info = self.pending_infos.pop(stamp_ns)
        if tags["reset_epoch"] != info["reset_epoch"]:
            self.rejected += len(tags["observations"])
            return [{
                "accepted": False,
                "reason": "reset_epoch_mismatch",
                "image_stamp_ns": stamp_ns,
                "tag_id": observation["tag_id"],
                "tag_reset_epoch": tags["reset_epoch"],
                "info_reset_epoch": info["reset_epoch"],
            } for observation in tags["observations"]]
        output = []
        for observation in tags["observations"]:
            output.append({
                "schema_version": SCHEMA_VERSION,
                "accepted": True,
                "reason": "exact_stamp_keyframe_binding",
                "image_stamp_ns": stamp_ns,
                "tag_id": observation["tag_id"],
                "rtabmap_landmark_id": observation.get(
                    "rtabmap_landmark_id", observation["tag_id"] + 1),
                "keyframe_id": info["ref_id"],
                "map_id": info["map_id"],
                "reset_epoch": info["reset_epoch"],
                "camera_translation_m": observation["camera_translation_m"],
                "camera_quaternion_xyzw": observation["camera_quaternion_xyzw"],
                "range_m": observation.get("range_m"),
                "reprojection_px": observation.get("reprojection_px"),
                "image_u_norm": observation.get("image_u_norm"),
                "image_v_norm": observation.get("image_v_norm"),
                "mean_side_px": observation.get("mean_side_px"),
                "camera_frame": observation.get("camera_frame"),
                "binding_monotonic_s": float(now),
                "graph_edge_added": False,
                "usable_as_absolute_anchor": False,
            })
        self.accepted += len(output)
        return output

    def mark_odom(self, lost, now):
        require(isinstance(lost, bool) and math.isfinite(now), "invalid odometry state")
        events = []
        if lost and not self.odom_lost:
            self.reset_epoch += 1
            events.extend(self._reject_all("odometry_loss_boundary"))
        self.odom_lost = lost
        return events

    def expire(self, now):
        require(math.isfinite(now), "expiry time must be finite")
        events = []
        for stamp_ns, payload in list(self.pending_tags.items()):
            if now - payload["receipt_monotonic_s"] > self.max_pending_age_s:
                self.pending_tags.pop(stamp_ns)
                for observation in payload["observations"]:
                    events.append({
                        "accepted": False,
                        "reason": "tag_without_exact_info",
                        "image_stamp_ns": stamp_ns,
                        "tag_id": observation["tag_id"],
                        "reset_epoch": payload["reset_epoch"],
                    })
        for stamp_ns, payload in list(self.pending_infos.items()):
            if now - payload["receipt_monotonic_s"] > self.max_pending_age_s:
                self.pending_infos.pop(stamp_ns)
                events.append({
                    "accepted": False,
                    "reason": "info_without_exact_tag",
                    "image_stamp_ns": stamp_ns,
                    "keyframe_id": payload["ref_id"],
                    "reset_epoch": payload["reset_epoch"],
                })
        self.rejected += len(events)
        return events

    def _reject_all(self, reason):
        events = []
        for stamp_ns, payload in self.pending_tags.items():
            for observation in payload["observations"]:
                events.append({
                    "accepted": False,
                    "reason": reason,
                    "image_stamp_ns": stamp_ns,
                    "tag_id": observation["tag_id"],
                    "reset_epoch": payload["reset_epoch"],
                })
        for stamp_ns, payload in self.pending_infos.items():
            events.append({
                "accepted": False,
                "reason": reason,
                "image_stamp_ns": stamp_ns,
                "keyframe_id": payload["ref_id"],
                "reset_epoch": payload["reset_epoch"],
            })
        self.pending_tags.clear()
        self.pending_infos.clear()
        self.rejected += len(events)
        return events

    def snapshot(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "observer_only": True,
            "graph_edge_added": False,
            "reset_epoch": self.reset_epoch,
            "odom_lost": self.odom_lost,
            "accepted_bindings": self.accepted,
            "rejected_events": self.rejected,
            "duplicate_messages": self.duplicate,
            "pending_tag_stamps": len(self.pending_tags),
            "pending_info_stamps": len(self.pending_infos),
        }


def secure_append(path, payload):
    flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT | O_CLOEXEC | O_NOFOLLOW
    descriptor = os.open(path, flags, 0o600)
    try:
        info = os.fstat(descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_nlink == 1,
                "event log is not a single-link regular file")
        if hasattr(os, "getuid"):
            require(info.st_uid == os.getuid(), "event log owner mismatch")
        os.fchmod(descriptor, 0o600)
        data = (json.dumps(payload, sort_keys=True) + "\n").encode("utf-8")
        offset = 0
        while offset < len(data):
            offset += os.write(descriptor, data[offset:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path, payload):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    require(not path.is_symlink() and not temporary.exists(), "unsafe state path")
    descriptor = os.open(
        temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | O_CLOEXEC | O_NOFOLLOW,
        0o600)
    try:
        data = (json.dumps(payload, sort_keys=True) + "\n").encode("utf-8")
        offset = 0
        while offset < len(data):
            offset += os.write(descriptor, data[offset:])
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o600)
    finally:
        os.close(descriptor)
    os.replace(temporary, path)


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def parse_candidate_metadata(raw):
    """Parse the detector's rich observer message without ROS dependencies."""
    require(isinstance(raw, str) and raw, "candidate metadata must be nonempty text")
    event = json.loads(raw)
    require(isinstance(event, dict), "candidate metadata must be an object")
    require(event.get("schema_version") == 1 and
            event.get("producer") == "wbrov_apriltag_landmark_node",
            "unexpected candidate metadata producer or schema")
    require(event.get("geometry_valid_before_temporal_confirmation") is True and
            event.get("graph_edge_added") is False and
            event.get("usable_as_absolute_anchor") is False,
            "candidate metadata crossed its observer-only boundary")
    stamp_ns = event.get("image_stamp_ns")
    camera_frame = event.get("camera_frame")
    observations = event.get("observations")
    require(isinstance(stamp_ns, int) and stamp_ns > 0,
            "invalid candidate metadata image stamp")
    require(isinstance(camera_frame, str) and camera_frame,
            "invalid candidate metadata camera frame")
    require(isinstance(observations, list) and observations,
            "candidate metadata has no observations")
    normalized = []
    for item in observations:
        require(isinstance(item, dict), "candidate metadata observation must be an object")
        observation = dict(item)
        observation["camera_frame"] = camera_frame
        normalized.append(observation)
    return stamp_ns, normalized


def map_id_from_info(message):
    require(len(message.stats_keys) == len(message.stats_values),
            "Info statistics keys/values length mismatch")
    values = [value for key, value in zip(message.stats_keys, message.stats_values)
              if key == "WBROV/Reference_map_id/"]
    require(len(values) == 1, "Info must provide one reference map ID")
    value = float(values[0])
    require(math.isfinite(value) and 0 <= value <= 2147483647 and value.is_integer(),
            "reference map ID must be a nonnegative integer")
    return int(value)


def run_ros(output, state, max_pending_age_s):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rtabmap_msgs.msg import Info, OdomInfo
    from std_msgs.msg import String

    # Keep the ROS context alive until the executor has stopped taking messages.
    # Humble's asynchronous SIGINT context shutdown can race take_message().
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    stop_signal = 0
    def request_stop(number, _frame):
        nonlocal stop_signal
        if not stop_signal:
            stop_signal = number
    old_handlers = {number: signal.signal(number, request_stop)
                    for number in (signal.SIGINT, signal.SIGTERM)}
    node = Node("apriltag_keyframe_binder")
    binder = ExactKeyframeBinder(max_pending_age_s)
    reliable = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=30)
    best_effort = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=30)

    def record(events):
        for event in events:
            secure_append(output, event)
        snapshot = binder.snapshot()
        snapshot.update({
            "pid": os.getpid(),
            "publish_monotonic_s": time.monotonic(),
            "output": str(output),
        })
        atomic_json(state, snapshot)

    def on_tags(message):
        stamp_ns, observations = parse_candidate_metadata(message.data)
        record(binder.add_tags(stamp_ns, observations, time.monotonic()))

    def on_info(message):
        record(binder.add_info(
            stamp_to_ns(message.header.stamp), int(message.ref_id),
            map_id_from_info(message), time.monotonic()))

    def on_odom(message):
        events = binder.mark_odom(bool(message.lost), time.monotonic())
        if events:
            record(events)

    def on_timer():
        record(binder.expire(time.monotonic()))

    subscriptions = [
        node.create_subscription(
            String, "/apriltag/pose_candidate_metadata", on_tags, reliable),
        node.create_subscription(Info, "/info", on_info, reliable),
        node.create_subscription(OdomInfo, "/odom_info_lite", on_odom, best_effort),
    ]
    timer = node.create_timer(0.25, on_timer)
    record([])
    try:
        while not stop_signal and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        require(bool(stop_signal), "ROS context stopped without a shutdown signal")
    finally:
        del timer, subscriptions
        record(binder._reject_all("observer_shutdown"))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in old_handlers.items():
            signal.signal(number, handler)
    return 128 + stop_signal


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--state", type=Path)
    parser.add_argument("--max-pending-age-s", type=float, default=2.5)
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    arguments = parse_args(sys.argv[1:] if argv is None else argv)
    if arguments.check_only:
        require(arguments.output is None and arguments.state is None,
                "--check-only cannot accept output paths")
        binder = ExactKeyframeBinder(arguments.max_pending_age_s)
        require(binder.snapshot()["graph_edge_added"] is False,
                "observer-only self-check failed")
        print("APRILTAG_KEYFRAME_BINDER_CHECK_OK observer_only=true database_open=false")
        return 0
    require(arguments.output is not None and arguments.state is not None,
            "--output and --state are required")
    output = arguments.output.absolute()
    state = arguments.state.absolute()
    require(output.parent == state.parent and output.parent.is_dir(),
            "outputs must share an existing private directory")
    require(not output.exists() and not state.exists(), "output already exists")
    return run_ros(output, state, arguments.max_pending_age_s)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BinderError, OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        print(f"APRILTAG_KEYFRAME_BINDER_REFUSED: {error}", file=sys.stderr)
        raise SystemExit(1)
