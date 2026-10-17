#!/usr/bin/env python3
"""Detect AprilTag 36h11 tags from the shared ZED ROS image stream.

The node does not open the camera.  It publishes RTAB-Map landmark
observations, an annotated image and RViz markers:

  /landmark_detections       rtabmap_msgs/LandmarkDetections
  /apriltag/pose_candidates  geometry-valid observations before temporal confirmation
  /apriltag/pose_candidate_metadata std_msgs/String with exact-stamp view geometry
  /apriltag/image            sensor_msgs/Image
  /apriltag/markers          current detections in the camera frame
  /apriltag/map_markers      short-lived, exact-image-time diagnostic observations

RTAB-Map requires landmark IDs greater than zero, so raw AprilTag ID N is
published internally as landmark ID N+1.  Labels and logs keep the raw ID.
"""

import csv
from collections import Counter
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rtabmap_msgs.msg import LandmarkDetection, LandmarkDetections
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from tag_pose_guard import GuardConfig, PendingTransformQueue, TemporalPoseGuard, cadence_gap_s
from tag_pose_geometry import solve_square_pose


IMAGE_TOPIC = "/zed/rgb/image_rect_color"
INFO_TOPIC = "/zed/rgb/camera_info"
LANDMARK_TOPIC = "/landmark_detections"
CANDIDATE_TOPIC = "/apriltag/pose_candidates"
CANDIDATE_METADATA_TOPIC = "/apriltag/pose_candidate_metadata"
ANNOTATED_TOPIC = "/apriltag/image"
CURRENT_MARKERS_TOPIC = "/apriltag/markers"
MAP_MARKERS_TOPIC = "/apriltag/map_markers"
FAMILY = "tag36h11"
ID_OFFSET = 1


def parse_landmark_mode(value):
    """Position-only uses RTAB-Map g2o's XYZ landmark covariance convention.

    Keep pose6d for other entry points. The isolated underwater mapping launcher
    must explicitly pair position_g2o with Optimizer/Strategy=1 and a new map.
    Do not apply this XYZ noise convention to GTSAM bearing/range factors.
    """
    if type(value) is not str or value not in ("pose6d", "position_g2o"):
        raise ValueError("APRILTAG_LANDMARK_MODE must be pose6d or position_g2o")
    return value


def env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError as exc:
        raise SystemExit(f"{name} must be numeric") from exc


def env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except ValueError as exc:
        raise SystemExit(f"{name} must be an integer") from exc


def rotation_to_quaternion(rotation):
    """Return x,y,z,w quaternion for a proper 3x3 rotation matrix."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


def quaternion_to_rotation(q):
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def pose_matrix(translation, quaternion):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_rotation(quaternion)
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class AprilTagLandmarkNode(Node):
    def __init__(self):
        super().__init__("apriltag_landmark")
        self.tag_size = env_float("APRILTAG_SIZE_M", 0.080)
        self.min_range = env_float(
            "APRILTAG_MIN_RANGE_M", os.environ.get("DENSE_MIN_M", 0.30))
        self.max_range = env_float(
            "APRILTAG_MAX_RANGE_M", os.environ.get("DENSE_MAX_M", 3.00))
        self.max_reprojection = env_float("APRILTAG_MAX_REPROJ_PX", 3.0)
        self.min_side_px = env_float("APRILTAG_MIN_SIDE_PX", 18.0)
        self.max_hz = env_float("APRILTAG_DETECT_HZ", 6.0)
        self.linear_std = env_float("APRILTAG_LINEAR_STD_M", 0.025)
        self.angular_std_deg = env_float("APRILTAG_ANGULAR_STD_DEG", 6.0)
        self.landmark_mode = parse_landmark_mode(
            os.environ.get("APRILTAG_LANDMARK_MODE", "pose6d"))
        self.map_frame = os.environ.get("APRILTAG_MAP_FRAME", "map")
        for name, value in (("size", self.tag_size), ("max_reprojection", self.max_reprojection),
                            ("min_side_px", self.min_side_px), ("max_hz", self.max_hz),
                            ("linear_std", self.linear_std), ("angular_std", self.angular_std_deg)):
            if not math.isfinite(value) or value <= 0:
                raise SystemExit("AprilTag " + name + " must be finite and positive")
        if not (math.isfinite(self.min_range) and math.isfinite(self.max_range) and
                0.0 <= self.min_range < self.max_range):
            raise SystemExit("invalid AprilTag range")
        self.pose_guard = TemporalPoseGuard(GuardConfig(
            confirm_frames=env_int("APRILTAG_CONFIRM_FRAMES", 3),
            confirm_duration_s=env_float("APRILTAG_CONFIRM_DURATION_S", 0.15),
            max_gap_s=cadence_gap_s(env_float("APRILTAG_INPUT_PERIOD_S", 0.0),
                env_float("APRILTAG_TRACK_MAX_GAP_S", 0.60)),
            max_angular_rate_deg_s=env_float("APRILTAG_MAX_ANGULAR_RATE_DEG_S", 60.0),
            angular_slack_deg=env_float("APRILTAG_ANGULAR_SLACK_DEG", 6.0),
            max_linear_rate_m_s=env_float("APRILTAG_MAX_LINEAR_RATE_M_S", 0.50),
            linear_slack_m=env_float("APRILTAG_LINEAR_SLACK_M", 0.015)))
        self.map_queue = PendingTransformQueue(capacity=32, max_age_s=0.5)
        self.gate_counts = Counter()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params.cornerRefinementWinSize = 5
        self.detector_params.cornerRefinementMaxIterations = 30
        self.detector_params.cornerRefinementMinAccuracy = 0.01

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.landmark_pub = self.create_publisher(
            LandmarkDetections, LANDMARK_TOPIC, qos)
        # This topic is deliberately separate from /landmark_detections.  It is
        # evidence for the old-map initial-pose seeder, never a graph factor.
        self.candidate_pub = self.create_publisher(
            LandmarkDetections, CANDIDATE_TOPIC, qos)
        self.candidate_metadata_pub = self.create_publisher(
            String, CANDIDATE_METADATA_TOPIC, qos)
        self.image_pub = self.create_publisher(Image, ANNOTATED_TOPIC, qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, CURRENT_MARKERS_TOPIC, qos)
        self.map_marker_pub = self.create_publisher(
            MarkerArray, MAP_MARKERS_TOPIC, map_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, INFO_TOPIC, self.info_callback, sensor_qos)
        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, sensor_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_matrix = None
        self.distortion = None
        self.info_size = None
        self.info_frame = ""
        self.info_signature = None
        self.last_detection_time = -math.inf
        self.last_received_image_stamp = -math.inf
        self.last_report = time.monotonic()
        self.frames = 0
        self.accepted = 0
        self.rejected = 0
        self.map_tags = {}
        self.map_dirty = False

        self.log_path = os.environ.get("APRILTAG_LOG", "")
        self.map_path = os.environ.get("APRILTAG_MAP_JSON", "")
        self.latest_path = os.environ.get("APRILTAG_LATEST_JSON", "")
        self.latest_sequence = 0
        self.last_camera_frame = ""
        self.last_image_stamp = 0.0
        try:
            self.boot_id = Path(
                "/proc/sys/kernel/random/boot_id").read_text(
                    encoding="ascii").strip()
        except OSError as exc:
            raise SystemExit(f"cannot read boot_id: {exc}") from exc
        if not self.boot_id:
            raise SystemExit("empty boot_id")
        if self.latest_path:
            latest = Path(self.latest_path).expanduser()
            latest.parent.mkdir(parents=True, exist_ok=True)
            if latest.is_symlink() or latest.parent.is_symlink():
                raise SystemExit("APRILTAG_LATEST_JSON cannot be a symlink")
            self.latest_path = str(latest)
        self.log_file = None
        self.log_writer = None
        self.gate_log_file = None
        self.gate_log_writer = None
        if self.log_path:
            path = Path(self.log_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            new_file = not path.exists() or path.stat().st_size == 0
            self.log_file = path.open("a", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file)
            if new_file:
                self.log_writer.writerow([
                    "stamp", "tag_id", "rtabmap_landmark_id", "range_m",
                    "reprojection_px", "camera_x", "camera_y", "camera_z",
                    "camera_qx", "camera_qy", "camera_qz", "camera_qw",
                    "map_x", "map_y", "map_z",
                    "map_qx", "map_qy", "map_qz", "map_qw",
                ])
                self.log_file.flush()
            gate_path = path.with_name(path.stem + "_pose_gate.csv")
            new_gate_file = not gate_path.exists() or gate_path.stat().st_size == 0
            self.gate_log_file = gate_path.open("a", newline="", encoding="utf-8")
            self.gate_log_writer = csv.writer(self.gate_log_file)
            if new_gate_file:
                self.gate_log_writer.writerow([
                    "stamp", "tag_id", "accepted", "reason", "range_m", "reprojection_px",
                    "image_u_norm", "image_v_norm", "mean_side_px",
                    "camera_x", "camera_y", "camera_z", "camera_qx", "camera_qy", "camera_qz", "camera_qw",
                    "translation_delta_m", "angular_delta_deg", "confirmation_samples",
                    "geometry_reason", "alternate_reprojection_px", "solution_angle_deg"])
                self.gate_log_file.flush()
        # Zero-wait lookups on a timer allow TF callbacks to run in this same executor.
        self.map_timer = self.create_timer(0.05, self.process_map_queue)

        self.get_logger().info(
            f"AprilTag {FAMILY}: black-square size={self.tag_size:.3f} m, "
            f"range={self.min_range:.2f}-{self.max_range:.2f} m, "
            f"RTAB landmark ID=tag ID+{ID_OFFSET}; pose confirmation="
            f"{self.pose_guard.config.confirm_frames} frames/"
            f"{self.pose_guard.config.confirm_duration_s:.2f}s, maximum input gap="
            f"{self.pose_guard.config.max_gap_s:.2f}s; IPPE square ambiguity gate "
            f"(15deg, 0.5px or 2x error); landmark mode={self.landmark_mode}; "
            f"exact-time TF diagnostics")

    def info_callback(self, msg):
        matrix = np.asarray(msg.k, dtype=np.float64)
        distortion = np.asarray(msg.d, dtype=np.float64)
        valid = (matrix.size == 9 and np.isfinite(matrix).all() and
                 np.isfinite(distortion).all() and msg.width > 0 and msg.height > 0 and
                 bool(msg.header.frame_id))
        if valid:
            matrix = matrix.reshape(3, 3)
            valid = matrix[0, 0] > 0 and matrix[1, 1] > 0 and abs(matrix[2, 2]-1.0) < 1e-6
        if not valid:
            self.camera_matrix = None
            self.info_signature = None
            self.clear_observation_state("invalid_camera_info")
            self.write_latest_json(None, [], active=True)
            return
        signature = (str(msg.header.frame_id), int(msg.width), int(msg.height),
                     tuple(matrix.reshape(-1)), tuple(distortion.reshape(-1)))
        if signature != self.info_signature:
            self.clear_observation_state("camera_info_changed")
            self.write_latest_json(None, [], active=True)
        self.info_signature = signature
        self.camera_matrix = matrix
        self.distortion = distortion.reshape(-1, 1)
        self.info_size = (int(msg.width), int(msg.height))
        self.info_frame = str(msg.header.frame_id)

    def clear_observation_state(self, reason):
        self.pose_guard.reset()
        self.map_queue.clear()
        self.map_tags.clear()
        self.map_dirty = True
        self.last_detection_time = -math.inf
        self.gate_counts[reason] += 1
        self.process_map_queue()

    @staticmethod
    def decode_image(msg):
        if msg.encoding not in ("bgr8", "rgb8", "mono8"):
            raise ValueError(f"unsupported image encoding {msg.encoding}")
        channels = 1 if msg.encoding == "mono8" else 3
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw.reshape(int(msg.height), int(msg.step))
        image = rows[:, :int(msg.width) * channels]
        if channels == 1:
            return image.reshape(int(msg.height), int(msg.width))
        image = image.reshape(int(msg.height), int(msg.width), channels)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    def image_callback(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            self.clear_observation_state("invalid_image_time")
            self.write_latest_json(msg, [], active=True)
            return
        # Do this BEFORE rate limiting, otherwise a clock reset can suppress all images.
        if stamp < self.last_received_image_stamp:
            self.clear_observation_state("image_time_regression")
            self.last_received_image_stamp = stamp
            self.write_latest_json(msg, [], active=True)
            return
        if stamp == self.last_received_image_stamp:
            return
        self.last_received_image_stamp = stamp
        if self.camera_matrix is None:
            self.get_logger().warning(
                "waiting for CameraInfo", throttle_duration_sec=5.0)
            return
        if msg.header.frame_id != self.info_frame:
            self.clear_observation_state("camera_frame_mismatch")
            self.write_latest_json(msg, [], active=True)
            return
        if stamp - self.last_detection_time < 1.0 / max(self.max_hz, 0.1):
            return
        self.last_detection_time = stamp
        try:
            bgr = self.decode_image(msg)
        except (ValueError, IndexError) as exc:
            self.clear_observation_state("invalid_image")
            self.write_latest_json(msg, [], active=True)
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        if bgr.shape[1::-1] != self.info_size:
            self.clear_observation_state("image_size_mismatch")
            self.write_latest_json(msg, [], active=True)
            self.get_logger().error(
                f"image/CameraInfo size mismatch: {bgr.shape[1::-1]} "
                f"vs {self.info_size}", throttle_duration_sec=5.0)
            return

        gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)
        detections = []
        pose_candidates = []
        geometry_ids = set()
        if ids is not None and len(ids):
            id_counts = Counter(int(value) for value in ids.flatten())
            for index, raw_id in enumerate(ids.flatten()):
                if id_counts[int(raw_id)] != 1:
                    self.rejected += 1
                    self.gate_counts["duplicate_tag_id"] += 1
                    continue
                try:
                    geometry = solve_square_pose(
                        corners[index], self.camera_matrix, self.distortion, self.tag_size,
                        max_reprojection_px=self.max_reprojection, min_side_px=self.min_side_px)
                    if not geometry["accepted"]:
                        self.rejected += 1
                        self.gate_counts[geometry["reason"]] += 1
                        best = geometry.get("best")
                        if best is not None:
                            rejected_pose = self.validate_pose(
                                int(raw_id), corners[index], best["rvec"], best["translation"])
                            if rejected_pose is not None:
                                self.write_gate_decision(msg.header.stamp, rejected_pose,
                                    {"accepted": False, "reason": geometry["reason"],
                                     "geometry": geometry})
                        # No geometric pose enters temporal confirmation. Repeated
                        # ambiguous frames must never become a "stable" old branch.
                        continue
                    best = geometry["best"]
                    result = self.validate_pose(
                        int(raw_id), corners[index], best["rvec"], best["translation"])
                except (ValueError, cv2.error, OverflowError):
                    result = None
                if result is not None:
                    # Publish geometry-valid PnP before the tracking guard.  The
                    # mapping stream is intentionally low-rate (normally 1 Hz),
                    # so its 0.60 s tracking-gap gate cannot accumulate three
                    # samples.  A separate localization-only seeder performs
                    # its own exact-stamp odometry and multi-frame checks.
                    pose_candidates.append(result)
                    geometry_ids.add(int(raw_id))
                    decision = self.pose_guard.update(
                        int(raw_id), stamp, result["translation"], result["quaternion"],
                        context=self.info_signature)
                    decision["geometry"] = geometry
                    self.gate_counts[decision["reason"]] += 1
                    self.write_gate_decision(msg.header.stamp, result, decision)
                    if decision["accepted"]:
                        # Keep this observation exactly as estimated; never substitute an old pose.
                        detections.append(result)
                    else:
                        self.rejected += 1
                else:
                    self.rejected += 1
                    self.gate_counts["geometry_rejected"] += 1
        self.pose_guard.finish_frame(stamp, geometry_ids)

        if pose_candidates:
            candidate_collection = LandmarkDetections()
            candidate_collection.header = msg.header
            for result in pose_candidates:
                candidate_collection.landmarks.append(
                    self.make_landmark(msg, result))
            self.candidate_pub.publish(candidate_collection)
            metadata = String()
            metadata.data = json.dumps({
                "schema_version": 1,
                "producer": "wbrov_apriltag_landmark_node",
                "image_stamp_ns": (
                    int(msg.header.stamp.sec) * 1_000_000_000 +
                    int(msg.header.stamp.nanosec)),
                "camera_frame": str(msg.header.frame_id),
                "geometry_valid_before_temporal_confirmation": True,
                "graph_edge_added": False,
                "usable_as_absolute_anchor": False,
                "observations": [{
                    "tag_id": int(result["tag_id"]),
                    "rtabmap_landmark_id": int(result["tag_id"]) + ID_OFFSET,
                    "camera_translation_m": [
                        float(value) for value in result["translation"]],
                    "camera_quaternion_xyzw": [
                        float(value) for value in result["quaternion"]],
                    "range_m": float(result["range"]),
                    "reprojection_px": float(result["reprojection"]),
                    "image_u_norm": float(result["image_u_norm"]),
                    "image_v_norm": float(result["image_v_norm"]),
                    "mean_side_px": float(result["mean_side_px"]),
                } for result in pose_candidates],
            }, sort_keys=True, separators=(",", ":"))
            self.candidate_metadata_pub.publish(metadata)

        accepted_ids = {item["tag_id"] for item in detections}
        display_ids = set(self.map_tags) | {item["tag_id"] for item in self.map_queue.items}
        removed_ids = display_ids - accepted_ids
        self.map_queue.discard_tags(removed_ids)
        for tag_id in removed_ids:
            if self.map_tags.pop(tag_id, None) is not None:
                self.map_dirty = True

        collection = LandmarkDetections()
        collection.header = msg.header
        for result in detections:
            landmark = self.make_landmark(msg, result)
            collection.landmarks.append(landmark)
            self.accepted += 1
        if detections:
            # Raw camera-frame SLAM observations never wait on a map display transform.
            self.landmark_pub.publish(collection)

        for result in detections:
            self.update_map_pose(msg.header.frame_id, msg.header.stamp, result)
        self.process_map_queue()
        for result in detections:
            # map columns are populated only if that exact-time TF was already available.
            self.write_detection(msg.header.stamp, result)

        self.write_latest_json(msg, detections, active=True)
        self.publish_current_markers(msg, detections)
        self.publish_annotated(msg, bgr, corners, ids, detections)
        self.frames += 1
        now = time.monotonic()
        if now - self.last_report >= 5.0:
            ids_text = ",".join(str(item["tag_id"]) for item in detections)
            self.get_logger().info(
                f"frames={self.frames}, accepted={self.accepted}, "
                f"rejected={self.rejected}, current IDs=[{ids_text}], "
                f"gate={dict(self.gate_counts)}, tf_pending={len(self.map_queue.items)}, "
                f"tf_expired={self.map_queue.expired}, tf_overflow={self.map_queue.overflow}")
            self.frames = self.accepted = self.rejected = 0
            self.gate_counts.clear()
            if self.gate_log_file is not None:
                self.gate_log_file.flush()
            self.last_report = now

    def validate_pose(self, tag_id, corner, rvec, tvec):
        image_points = np.asarray(corner, dtype=np.float64).reshape(4, 2)
        if not np.isfinite(image_points).all():
            return None
        side_lengths = [
            float(np.linalg.norm(image_points[(i + 1) % 4] - image_points[i]))
            for i in range(4)]
        if min(side_lengths) < self.min_side_px:
            return None
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
        if not (np.isfinite(rvec).all() and np.isfinite(tvec).all()):
            return None
        distance = float(np.linalg.norm(tvec))
        if tvec[2] <= 0.0 or distance < self.min_range or distance > self.max_range:
            return None
        half = self.tag_size * 0.5
        object_points = np.asarray([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.distortion)
        projected = projected.reshape(4, 2)
        reprojection = float(np.sqrt(np.mean(np.sum(
            (projected - image_points) ** 2, axis=1))))
        if not math.isfinite(reprojection) or reprojection > self.max_reprojection:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        quaternion = rotation_to_quaternion(rotation)
        image_center = np.mean(image_points, axis=0)
        return {
            "tag_id": tag_id,
            "landmark_id": tag_id + ID_OFFSET,
            "rvec": rvec,
            "translation": tvec,
            "quaternion": quaternion,
            "range": distance,
            "reprojection": reprojection,
            "image_u_norm": float(image_center[0] / float(self.info_size[0])),
            "image_v_norm": float(image_center[1] / float(self.info_size[1])),
            "mean_side_px": float(np.mean(side_lengths)),
        }

    def make_landmark(self, image_msg, result):
        landmark = LandmarkDetection()
        landmark.header = image_msg.header
        landmark.landmark_frame_id = f"apriltag_{FAMILY}_{result['tag_id']}"
        landmark.id = result["landmark_id"]
        landmark.size = float(self.tag_size)
        translation = result["translation"]
        quaternion = result["quaternion"]
        landmark.pose.pose.position.x = float(translation[0])
        landmark.pose.pose.position.y = float(translation[1])
        landmark.pose.pose.position.z = float(translation[2])
        landmark.pose.pose.orientation.x = float(quaternion[0])
        landmark.pose.pose.orientation.y = float(quaternion[1])
        landmark.pose.pose.orientation.z = float(quaternion[2])
        landmark.pose.pose.orientation.w = float(quaternion[3])
        distance_scale = max(1.0, result["range"])
        linear_variance = (self.linear_std * distance_scale) ** 2
        angular_variance = math.radians(
            self.angular_std_deg * distance_scale) ** 2
        if self.landmark_mode == "position_g2o":
            # g2o selects PointXYZ when an angular variance is >=9999.
            # Use 10000 to stay above that sentinel after numeric inversion.
            # Translation noise stays in metres; robot poses stay full SE(3).
            angular_variance = 10000.0
        covariance = [0.0] * 36
        for index in (0, 7, 14):
            covariance[index] = linear_variance
        for index in (21, 28, 35):
            covariance[index] = angular_variance
        landmark.pose.covariance = covariance
        return landmark

    def update_map_pose(self, camera_frame, stamp, result):
        image_stamp = TimeMsg(sec=int(stamp.sec), nanosec=int(stamp.nanosec))
        self.map_queue.enqueue(result["tag_id"],
            int(stamp.sec)*1_000_000_000 + int(stamp.nanosec),
            {"camera_frame": camera_frame, "stamp": image_stamp, "result": result},
            time.monotonic())

    def resolve_map_observation(self, payload):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, payload["camera_frame"], Time.from_msg(payload["stamp"]),
                timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        raw = [translation.x, translation.y, translation.z,
               rotation.x, rotation.y, rotation.z, rotation.w]
        if not all(math.isfinite(float(x)) for x in raw):
            return None
        norm = math.sqrt(sum(float(x)*float(x) for x in raw[3:]))
        if norm < 1e-12 or abs(norm-1.0) > 0.05:
            return None
        map_from_camera = pose_matrix(
            raw[:3], raw[3:])
        result = payload["result"]
        camera_from_tag = pose_matrix(
            result["translation"], result["quaternion"])
        map_from_tag = map_from_camera @ camera_from_tag
        result["map_translation"] = map_from_tag[:3, 3].copy()
        result["map_quaternion"] = rotation_to_quaternion(map_from_tag[:3, :3])
        result["map_stamp"] = stamp_seconds(payload["stamp"])
        return {"landmark_id": result["landmark_id"],
                "translation": result["map_translation"].tolist(),
                "quaternion": result["map_quaternion"].tolist(),
                "stamp": result["map_stamp"], "stamp_sec": int(payload["stamp"].sec),
                "stamp_nanosec": int(payload["stamp"].nanosec),
                "range_m": result["range"], "reprojection_px": result["reprojection"],
                "pose_kind": "exact_image_time_observation_not_optimized_landmark"}

    def process_map_queue(self):
        now = time.monotonic()
        for item in self.map_queue.resolve_ready(now, self.resolve_map_observation):
            self.map_tags[item["tag_id"]] = dict(item["resolved"],
                expires_monotonic_s=item["queued_monotonic"] + self.map_queue.max_age_s)
            self.map_dirty = True
        for tag_id, data in list(self.map_tags.items()):
            if now >= data["expires_monotonic_s"]:
                del self.map_tags[tag_id]
                self.map_dirty = True
        if self.map_dirty:
            self.write_map_json()
            self.publish_map_markers(self.get_clock().now().to_msg())
            self.map_dirty = False

    def make_delete_all(self, frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def make_tag_marker(
            self, frame_id, stamp, tag_id, translation, quaternion, stable):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "apriltag_map" if stable else "apriltag_current"
        marker.id = int(tag_id) * 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(translation[0])
        marker.pose.position.y = float(translation[1])
        marker.pose.position.z = float(translation[2])
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        marker.scale.x = self.tag_size
        marker.scale.y = self.tag_size
        marker.scale.z = 0.008
        marker.color.r = 0.0 if stable else 1.0
        marker.color.g = 1.0
        marker.color.b = 0.2 if stable else 0.0
        marker.color.a = 0.90
        marker.lifetime = Duration(seconds=0.5).to_msg()

        text = Marker()
        text.header = marker.header
        text.ns = marker.ns + "_labels"
        text.id = int(tag_id) * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(translation[0])
        text.pose.position.y = float(translation[1])
        text.pose.position.z = float(translation[2]) + self.tag_size
        text.pose.orientation.w = 1.0
        text.scale.z = max(0.045, self.tag_size * 0.55)
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.2
        text.color.a = 1.0
        text.text = f"Tag {tag_id}" + (" observation" if stable else "")
        text.lifetime = Duration(seconds=0.5).to_msg()
        return marker, text

    def publish_current_markers(self, msg, detections):
        array = MarkerArray()
        array.markers.append(self.make_delete_all(
            msg.header.frame_id, msg.header.stamp))
        for result in detections:
            marker, text = self.make_tag_marker(
                msg.header.frame_id, msg.header.stamp, result["tag_id"],
                result["translation"], result["quaternion"], stable=False)
            marker.lifetime = Duration(seconds=0.5).to_msg()
            text.lifetime = Duration(seconds=0.5).to_msg()
            array.markers.extend((marker, text))
        self.marker_pub.publish(array)

    def publish_map_markers(self, stamp):
        array = MarkerArray()
        array.markers.append(self.make_delete_all(self.map_frame, stamp))
        for tag_id, data in sorted(self.map_tags.items()):
            marker, text = self.make_tag_marker(
                self.map_frame, TimeMsg(sec=data["stamp_sec"], nanosec=data["stamp_nanosec"]),
                tag_id, data["translation"],
                data["quaternion"], stable=True)
            array.markers.extend((marker, text))
        self.map_marker_pub.publish(array)

    def publish_annotated(self, msg, bgr, corners, ids, detections):
        if self.image_pub.get_subscription_count() == 0:
            return
        annotated = cv2.cvtColor(
            bgr, cv2.COLOR_GRAY2BGR) if bgr.ndim == 2 else bgr.copy()
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        accepted_by_id = {item["tag_id"]: item for item in detections}
        for tag_id, result in accepted_by_id.items():
            cv2.drawFrameAxes(
                annotated, self.camera_matrix, self.distortion,
                result["rvec"], result["translation"],
                self.tag_size * 0.55, 2)
            corner_index = int(np.where(ids.flatten() == tag_id)[0][0])
            anchor = np.asarray(corners[corner_index]).reshape(4, 2).min(axis=0)
            x, y = int(anchor[0]), max(20, int(anchor[1]) - 8)
            cv2.putText(
                annotated,
                f"ID={tag_id} range={result['range']:.2f}m "
                f"err={result['reprojection']:.1f}px",
                (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(
            annotated,
            f"AprilTag {FAMILY} size={self.tag_size:.3f}m "
            f"accepted={len(detections)}",
            (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (0, 255, 255), 1, cv2.LINE_AA)
        output = Image()
        output.header = msg.header
        output.height, output.width = annotated.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = np.ascontiguousarray(annotated).tobytes()
        self.image_pub.publish(output)

    def write_detection(self, stamp, result):
        if self.log_writer is None:
            return
        map_translation = result.get("map_translation", ["", "", ""])
        map_quaternion = result.get("map_quaternion", ["", "", "", ""])
        self.log_writer.writerow([
            f"{stamp_seconds(stamp):.9f}",
            result["tag_id"],
            result["landmark_id"],
            f"{result['range']:.6f}",
            f"{result['reprojection']:.6f}",
            *[f"{value:.8f}" for value in result["translation"]],
            *[f"{value:.8f}" for value in result["quaternion"]],
            *[f"{value:.8f}" if value != "" else "" for value in map_translation],
            *[f"{value:.8f}" if value != "" else "" for value in map_quaternion],
        ])
        self.log_file.flush()

    def write_gate_decision(self, stamp, result, decision):
        if self.gate_log_writer is None:
            return
        geometry = decision.get("geometry", {})
        self.gate_log_writer.writerow([
            f"{stamp_seconds(stamp):.9f}", result["tag_id"], int(decision["accepted"]),
            decision["reason"], result["range"], result["reprojection"],
            result["image_u_norm"], result["image_v_norm"], result["mean_side_px"],
            *result["translation"], *result["quaternion"],
            decision.get("translation_delta_m", ""), decision.get("angular_delta_deg", ""),
            decision.get("confirmation_samples", ""), geometry.get("reason", ""),
            geometry.get("second_reprojection_px", ""), geometry.get("solution_angle_deg", "")])

    def write_map_json(self):
        if not self.map_path:
            return
        path = Path(self.map_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "family": FAMILY,
            "tag_size_m": self.tag_size,
            "id_mapping": "rtabmap_landmark_id = apriltag_id + 1",
            "frame_id": self.map_frame,
            "tags": {
                str(tag_id): data for tag_id, data in sorted(self.map_tags.items())
            },
        }
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        os.replace(temporary, path)

    def write_latest_json(self, image_msg, detections, active):
        """Atomically publish same-boot camera-frame tag poses for teleop."""
        if not self.latest_path:
            return
        if image_msg is not None:
            self.last_camera_frame = str(image_msg.header.frame_id)
            self.last_image_stamp = stamp_seconds(image_msg.header.stamp)
        self.latest_sequence += 1
        payload = {
            "schema": 1,
            "producer": "wbrov_apriltag_landmark",
            "boot_id": self.boot_id,
            "active": bool(active),
            "sequence": self.latest_sequence,
            "updated_monotonic_s": time.monotonic(),
            "image_stamp_s": self.last_image_stamp,
            "camera_frame": self.last_camera_frame,
            "detections": [{
                "tag_id": int(result["tag_id"]),
                "camera_translation_m": [
                    float(value) for value in result["translation"]],
                "camera_quaternion_xyzw": [
                    float(value) for value in result["quaternion"]],
                "range_m": float(result["range"]),
                "reprojection_px": float(result["reprojection"]),
            } for result in detections],
        }
        path = Path(self.latest_path)
        temporary = path.with_name(
            f"{path.name}.tmp.{os.getpid()}")
        try:
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                + "\n",
                encoding="utf-8")
            os.replace(temporary, path)
        except OSError as exc:
            try:
                temporary.unlink()
            except OSError:
                pass
            self.get_logger().error(
                f"cannot publish APRILTAG_LATEST_JSON: {exc}",
                throttle_duration_sec=5.0)

    def close(self):
        self.map_timer.cancel()
        self.pose_guard.reset()
        self.map_queue.clear()
        self.map_tags.clear()
        try:
            self.write_map_json()
            if rclpy.ok():
                self.publish_map_markers(self.get_clock().now().to_msg())
        finally:
            try:
                self.write_latest_json(None, [], active=False)
            finally:
                if self.log_file is not None:
                    self.log_file.close()
                    self.log_file = None
                if self.gate_log_file is not None:
                    self.gate_log_file.close()
                    self.gate_log_file = None


def self_test():
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    marker = np.zeros((200, 200), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, 3, 200, marker, 1)
    canvas = np.full((360, 480), 255, dtype=np.uint8)
    canvas[80:280, 140:340] = marker
    corners, ids, _ = cv2.aruco.detectMarkers(
        canvas, dictionary,
        parameters=cv2.aruco.DetectorParameters_create())
    if ids is None or ids.flatten().tolist() != [3] or len(corners) != 1:
        raise SystemExit("APRILTAG_SELF_TEST_FAILED")
    print("APRILTAG_SELF_TEST_OK: tag36h11 ID=3")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return
    rclpy.init()
    node = None
    try:
        node = AprilTagLandmarkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
