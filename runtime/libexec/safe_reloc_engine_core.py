#!/usr/bin/env python3
"""Exact-keyframe, multi-view AprilTag relocalization proposal producer.

The node never changes an RTAB-Map graph directly.  It publishes three
independent kUserClosure proposals only after exact image-stamp binding,
view-diverse tag observations and a consistent old-map alignment.  A patched
RTAB-Map CoreWrapper owns the final all-or-nothing graph transaction and emits
an explicit status acknowledgement.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import time


SCHEMA_VERSION = 2
PRODUCER = "wbrov_apriltag_safe_relocalizer"
CORE_PRODUCER = "wbrov_rtabmap_safe_relocalization_core"
ZERO_SHA256 = "0" * 64
LINK_USER_CLOSURE = 4
PROPOSAL_RETRY_INTERVAL_S = 0.5
PROPOSAL_MAX_ATTEMPTS = 12
PROPOSAL_TERMINAL_TIMEOUT_S = 7.0
O_CLOEXEC = getattr(os, "O_CLOEXEC", 0)
O_NOFOLLOW = getattr(os, "O_NOFOLLOW", 0)


class RelocalizationError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise RelocalizationError(message)


class StampPayloadLedger:
    """Make one measurement stamp immutable for a bounded join epoch."""

    def __init__(self, max_entries=262144):
        self.entries = {}
        self.max_entries = int(max_entries)
        require(self.max_entries > 0, "stamp ledger capacity must be positive")

    @staticmethod
    def fingerprint(value):
        return hashlib.sha256(json.dumps(
            value, sort_keys=True, separators=(",", ":"),
            allow_nan=False).encode("utf-8")).hexdigest()

    def remember(self, stamp_ns, value, now, stream):
        fingerprint = self.fingerprint(value)
        previous = self.entries.get(stamp_ns)
        if previous is not None:
            require(previous[1] == fingerprint,
                    f"{stream} reused one stamp with different payload")
            return False
        require(len(self.entries) < self.max_entries,
                f"{stream} immutable stamp ledger capacity exhausted")
        self.entries[stamp_ns] = (float(now), fingerprint)
        return True

    def clear(self):
        self.entries.clear()


def strict_json_loads(raw):
    def reject_duplicates(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, f"duplicate JSON key: {key}")
            result[key] = value
        return result
    def reject_constant(value):
        raise RelocalizationError(f"non-finite JSON constant: {value}")
    return json.loads(raw, object_pairs_hook=reject_duplicates,
                      parse_constant=reject_constant)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def finite_vector(value, count):
    return (isinstance(value, (list, tuple)) and len(value) == count and
            all(finite_number(item) for item in value))


def normalize_quaternion(value):
    require(finite_vector(value, 4), "invalid quaternion")
    norm = math.sqrt(sum(float(item) ** 2 for item in value))
    require(0.95 <= norm <= 1.05, "quaternion is not near unit")
    result = tuple(float(item) / norm for item in value)
    # q and -q are the same rotation.  Canonicalization keeps comparisons stable.
    if result[3] < 0.0 or (result[3] == 0.0 and result[:3] < (0.0, 0.0, 0.0)):
        result = tuple(-item for item in result)
    return result


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def quaternion_conjugate(value):
    x, y, z, w = normalize_quaternion(value)
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = (float(item) for item in vector)
    # Unit-quaternion vector rotation without constructing temporary quaternions.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def pose(translation, quaternion):
    require(finite_vector(translation, 3), "invalid translation")
    return {
        "translation": tuple(float(item) for item in translation),
        "quaternion_xyzw": normalize_quaternion(quaternion),
    }


def pose_multiply(left, right):
    rotated = rotate_vector(left["quaternion_xyzw"], right["translation"])
    return pose(
        tuple(left["translation"][index] + rotated[index] for index in range(3)),
        quaternion_multiply(left["quaternion_xyzw"], right["quaternion_xyzw"]),
    )


def pose_inverse(value):
    inverse_q = quaternion_conjugate(value["quaternion_xyzw"])
    inverse_t = rotate_vector(inverse_q, tuple(-item for item in value["translation"]))
    return pose(inverse_t, inverse_q)


def pose_delta(left, right):
    translation = math.dist(left["translation"], right["translation"])
    dot = abs(sum(a * b for a, b in zip(
        normalize_quaternion(left["quaternion_xyzw"]),
        normalize_quaternion(right["quaternion_xyzw"]))))
    dot = max(-1.0, min(1.0, dot))
    return translation, math.degrees(2.0 * math.acos(dot))


def medoid(poses, translation_scale=0.05, rotation_scale_deg=5.0):
    require(poses, "cannot select medoid of empty poses")
    costs = []
    for left in poses:
        cost = 0.0
        for right in poses:
            linear, angular = pose_delta(left, right)
            cost += linear / translation_scale + angular / rotation_scale_deg
        costs.append(cost)
    return poses[min(range(len(poses)), key=costs.__getitem__)]


def view_bin(u, v):
    require(math.isfinite(u) and math.isfinite(v) and 0.0 <= u <= 1.0 and
            0.0 <= v <= 1.0, "invalid normalized image position")
    return int(min(2, math.floor(3.0 * u))), int(min(2, math.floor(3.0 * v)))


def validate_binding(raw):
    require(isinstance(raw, dict) and raw.get("accepted") is True,
            "binding is not accepted")
    for name in ("image_stamp_ns", "tag_id", "keyframe_id", "map_id",
                 "reset_epoch"):
        require(type(raw.get(name)) is int, f"invalid binding {name}")
    require(raw["image_stamp_ns"] > 0 and raw["tag_id"] >= 0 and
            raw["keyframe_id"] > 0 and raw["map_id"] >= 0 and
            raw["reset_epoch"] >= 0, "binding identifiers out of range")
    camera = pose(raw.get("camera_translation_m"),
                  raw.get("camera_quaternion_xyzw"))
    require(raw.get("camera_frame") == "zed_left_camera_optical_frame",
            "binding camera frame mismatch")
    for name in ("range_m", "reprojection_px", "image_u_norm",
                 "image_v_norm", "mean_side_px"):
        require(finite_number(raw.get(name)), f"invalid binding {name}")
    require(0.0 < raw["range_m"] <= 1.5 and
            0.0 <= raw["reprojection_px"] <= 3.0 and
            raw["mean_side_px"] >= 18.0,
            "binding view geometry outside safe limits")
    result = dict(raw)
    result["camera_T_tag"] = camera
    result["view_bin"] = view_bin(raw["image_u_norm"], raw["image_v_norm"])
    return result


def validate_graph_pose(raw):
    require(isinstance(raw, dict), "graph pose must be an object")
    return pose(raw.get("translation"), raw.get("quaternion_xyzw"))


class SafeRelocalizationEngine:
    """Pure state machine.  Positive output is still only a core proposal."""

    def __init__(self, base_t_camera, lost_count=8, window_s=8.0,
                 max_alignment_translation_m=0.08,
                 max_alignment_rotation_deg=8.0,
                 session_token=ZERO_SHA256):
        require(isinstance(lost_count, int) and 2 <= lost_count <= 100,
                "lost_count must be in [2,100]")
        require(1.0 <= window_s <= 30.0, "window must be in [1,30] seconds")
        require(isinstance(session_token, str) and len(session_token) == 64 and
                all(character in "0123456789abcdef" for character in session_token),
                "session token must be 64 lowercase hexadecimal characters")
        self.base_t_camera = base_t_camera
        self.lost_count = lost_count
        self.window_s = float(window_s)
        self.max_alignment_translation_m = float(max_alignment_translation_m)
        self.max_alignment_rotation_deg = float(max_alignment_rotation_deg)
        self.session_token = session_token
        self.reset_epoch = 0
        self.epoch_start_stamp_ns = 0
        self.lost_streak = 0
        self.recovery_armed = False
        self.awaiting_authoritative_epoch = False
        self.graph_poses = {}
        self.map_graph_message_count = 0
        self.map_graph_synchronized_with_core = False
        self.core_graph_revision = None
        self.core_state_reset_epoch = None
        self.core_database_binding_valid = False
        self.anchor_views = {}
        # Materialized while the trusted pre-reset component is still visible.
        # Unlike anchor_views, these records no longer depend on a future
        # /mapGraph containing the old component.
        self.same_run_anchors = {}
        self.persistent_anchors = {}
        # A new, not-yet-frozen live database has no stable byte digest.  The
        # launcher and Core bind that lifecycle explicitly with this sentinel.
        # Continued maps replace it with the exact stopped source DB digest.
        self.anchor_database_sha256 = ZERO_SHA256
        self.current_candidates = {}
        self.pending_transaction = None
        self.merged_epochs = set()
        self.sequence = 0

    def load_anchor_library(self, data, expected_database_sha256=None):
        require(isinstance(data, dict) and data.get("producer") ==
                "wbrov_apriltag_multiview_anchor_builder",
                "unexpected anchor producer")
        schema_version = data.get("schema_version")
        require(schema_version in (3, 4),
                "only exact-binding anchor schema 3 or 4 is accepted")
        source_sha256 = data.get("source_database_sha256")
        require(isinstance(source_sha256, str) and len(source_sha256) == 64 and
                all(character in "0123456789abcdef" for character in source_sha256),
                "anchor library lacks a canonical source database digest")
        if expected_database_sha256 is not None:
            require(source_sha256 == expected_database_sha256,
                    "anchor library does not belong to the continued database")
        source_kind = data.get("candidate_source_kind")
        if schema_version == 3:
            require(source_kind == "runtime_exact_keyframe_bindings_jsonl",
                    "schema 3 anchors require runtime exact keyframe bindings")
        else:
            require(source_kind in (
                        "runtime_exact_keyframe_bindings_jsonl",
                        "previous_exact_anchors_only",
                        "previous_exact_anchors_plus_runtime_exact_keyframe_bindings_jsonl"),
                    "schema 4 anchors require an exact continued-anchor source")
        require(data.get("map_frame") == "map",
                "anchor library map frame mismatch")
        policy = data.get("policy")
        require(isinstance(policy, dict) and
                policy.get("runtime_exact_keyframe_binding_required") is True and
                policy.get("required_distinct_current_frames") == 3,
                "anchor policy is not safe for online relocalization")
        if schema_version == 4:
            lineage = data.get("lineage")
            require(isinstance(lineage, dict),
                    "schema 4 anchor lineage is missing")
            if source_kind == "runtime_exact_keyframe_bindings_jsonl":
                require(lineage.get("mode") == "new" and
                        lineage.get("parent_schema_version") is None and
                        lineage.get("parent_database_sha256") is None and
                        lineage.get("parent_anchor_sha256") is None and
                        lineage.get("all_parent_nodes_present") is None and
                        lineage.get("parent_anchor_mutated") is None,
                        "new schema 4 anchor lineage is not empty")
            else:
                require(lineage.get("mode") == "continued" and
                        type(lineage.get("parent_schema_version")) is int and
                        lineage.get("parent_schema_version") in (3, 4) and
                        isinstance(lineage.get("parent_database_sha256"), str) and
                        len(lineage["parent_database_sha256"]) == 64 and
                        all(character in "0123456789abcdef"
                            for character in lineage["parent_database_sha256"]) and
                        isinstance(lineage.get("parent_anchor_sha256"), str) and
                        len(lineage["parent_anchor_sha256"]) == 64 and
                        all(character in "0123456789abcdef"
                            for character in lineage["parent_anchor_sha256"]) and
                        lineage.get("all_parent_nodes_present") is True and
                        lineage.get("parent_anchor_mutated") is False,
                        "continued schema 4 anchor lineage is incomplete or unsafe")
        require(data.get("base_frame") == "base_link" and
                data.get("camera_frame") == "zed_left_camera_optical_frame",
                "anchor frame contract mismatch")
        library_base = pose(data["base_T_camera"]["translation"],
                            data["base_T_camera"]["quaternion_xyzw"])
        linear, angular = pose_delta(self.base_t_camera, library_base)
        require(linear <= 1e-6 and angular <= 1e-4,
                "anchor camera extrinsic mismatch")
        parsed = {}
        for raw_tag, record in data.get("tags", {}).items():
            require(isinstance(raw_tag, str) and
                    re.fullmatch(r"0|[1-9][0-9]*", raw_tag) is not None,
                    "anchor tag key is not canonical non-negative decimal")
            tag_id = int(raw_tag)
            require(tag_id not in parsed and isinstance(record, dict) and
                    type(record.get("rtabmap_landmark_id")) is int and
                    record.get("rtabmap_landmark_id") == tag_id + 1,
                    f"tag {tag_id} has an unexpected landmark ID")
            record_map_t_tag = pose(record["map_T_tag"]["translation"],
                                    record["map_T_tag"]["quaternion_xyzw"])
            exemplars = []
            for item in record.get("view_exemplars", []):
                node_id = item.get("saved_node_id")
                require(type(node_id) is int and node_id > 0,
                        "anchor exemplar has invalid saved node")
                camera_t_tag = pose(item["camera_T_tag"]["translation"],
                                    item["camera_T_tag"]["quaternion_xyzw"])
                exemplar_map_t_tag = pose(item["map_T_tag"]["translation"],
                                          item["map_T_tag"]["quaternion_xyzw"])
                linear, angular = pose_delta(
                    record_map_t_tag, exemplar_map_t_tag)
                require(linear <= 0.03 and angular <= 5.0,
                        f"tag {tag_id} exemplar disagrees with its anchor")
                require(type(item.get("image_stamp_ns")) is int and
                        item["image_stamp_ns"] > 0 and
                        isinstance(item.get("view_bin"), str) and
                        item["view_bin"] != "unknown" and
                        finite_number(item.get("range_m")) and
                        0.0 < item["range_m"] <= 1.5 and
                        finite_number(item.get("reprojection_px")) and
                        0.0 <= item["reprojection_px"] <= 3.0,
                        f"tag {tag_id} exemplar lacks exact safe geometry")
                node_t_tag = pose_multiply(self.base_t_camera, camera_t_tag)
                exemplars.append({
                    "keyframe_id": node_id,
                    "map_T_node": pose_multiply(
                        exemplar_map_t_tag, pose_inverse(node_t_tag)),
                    "camera_T_tag": camera_t_tag,
                })
            require(len({item["keyframe_id"] for item in exemplars}) >= 3,
                    f"tag {tag_id} lacks three exact saved-node views")
            parsed[tag_id] = {
                "map_T_tag": record_map_t_tag,
                "exemplars": exemplars,
                "source": "persistent_exact_anchor_library",
            }
        require(parsed, "anchor library contains no eligible tag")
        self.persistent_anchors.update(parsed)
        self.anchor_database_sha256 = source_sha256

    def update_graph(self, graph_poses):
        """Test helper for pre-Core-snapshot fixtures; runtime uses apply_core_state."""
        require(isinstance(graph_poses, dict), "graph poses must be a dict")
        normalized = {}
        for node_id, value in graph_poses.items():
            require(isinstance(node_id, int) and node_id > 0,
                    "invalid graph node id")
            normalized[node_id] = validate_graph_pose(value)
        self.graph_poses = normalized
        self.map_graph_message_count += 1
        # A standalone graph has no authoritative revision and cannot be used
        # by runtime proposal generation until an atomic Core snapshot arrives.
        self.map_graph_synchronized_with_core = False
        if self.reset_epoch == 0:
            for tag_id in tuple(self.anchor_views):
                self._materialize_same_run_anchor(tag_id)

    def apply_core_state(self, state):
        require(isinstance(state, dict), "core state must be an object")
        expected_fields = {
            "schema_version", "producer", "reset_epoch",
            "epoch_start_stamp_ns", "graph_revision",
            "database_sha256", "database_binding_valid",
            "session_token_sha256", "graph_snapshot_valid", "graph_poses",
        }
        require(set(state) == expected_fields,
                "core state does not match the strict schema")
        require(state.get("schema_version") == 3 and
                state.get("producer") == CORE_PRODUCER,
                "unexpected core state schema or producer")
        epoch = state.get("reset_epoch")
        epoch_start_stamp_ns = state.get("epoch_start_stamp_ns")
        revision = state.get("graph_revision")
        require(type(epoch) is int and epoch >= 0 and
                type(epoch_start_stamp_ns) is int and epoch_start_stamp_ns >= 0 and
                type(revision) is int and revision >= 0,
                "core epoch, cutoff stamp and graph revision must be unsigned integers")
        require((epoch == 0 and epoch_start_stamp_ns == 0) or
                (epoch > 0 and epoch_start_stamp_ns > 0),
                "core epoch cutoff does not match the epoch lifecycle")
        if self.core_graph_revision is not None:
            require(revision >= self.core_graph_revision,
                    "authoritative core graph revision cannot go backwards")
            require(epoch >= self.core_state_reset_epoch,
                    "authoritative core state epoch cannot go backwards")
            if epoch > self.core_state_reset_epoch:
                require(revision > self.core_graph_revision,
                        "a new core epoch must advance graph revision")
            else:
                require(epoch_start_stamp_ns == self.epoch_start_stamp_ns,
                        "one core epoch cannot change its measurement cutoff")
        database_sha256 = state.get("database_sha256")
        require(isinstance(database_sha256, str) and
                len(database_sha256) == 64 and
                all(character in "0123456789abcdef"
                    for character in database_sha256),
                "core database identity is not lowercase SHA256")
        require(type(state.get("database_binding_valid")) is bool,
                "core database binding flag must be boolean")
        session_token_sha256 = state.get("session_token_sha256")
        expected_session_sha256 = hashlib.sha256(
            self.session_token.encode("ascii")).hexdigest()
        require(session_token_sha256 == expected_session_sha256,
                "core state belongs to a different launcher session")
        require(state["database_binding_valid"] is True and
                database_sha256 == self.anchor_database_sha256,
                "core state is not bound to the observer database identity")
        require(state.get("graph_snapshot_valid") is True and
                isinstance(state.get("graph_poses"), list) and
                len(state["graph_poses"]) <= 20000,
                "core graph snapshot is invalid or unbounded")
        graph = {}
        for item in state["graph_poses"]:
            require(isinstance(item, dict) and set(item) == {
                        "id", "translation", "quaternion_xyzw"},
                    "core graph pose has an unexpected schema")
            node_id = item.get("id")
            require(type(node_id) is int and node_id > 0 and node_id not in graph,
                    "core graph pose ID is invalid or duplicated")
            graph[node_id] = validate_graph_pose(item)
        if self.core_graph_revision is not None and revision == self.core_graph_revision:
            require(epoch == self.core_state_reset_epoch and graph == self.graph_poses,
                    "one core graph revision cannot describe different state")
        self.set_authoritative_epoch(epoch, epoch_start_stamp_ns)
        self.graph_poses = graph
        self.map_graph_message_count += 1
        self.core_state_reset_epoch = epoch
        self.core_graph_revision = revision
        self.core_database_binding_valid = True
        self.map_graph_synchronized_with_core = True

    def ready_proposals(self, now):
        if self.pending_transaction is not None:
            return []
        for tag_id in sorted(self.current_candidates):
            proposals = self._build_proposals(tag_id, now)
            if proposals:
                return proposals
        return []

    def mark_odom(self, lost):
        require(isinstance(lost, bool), "lost flag must be bool")
        if lost:
            self.lost_streak += 1
            if self.lost_streak >= self.lost_count:
                self.recovery_armed = True
            return False
        if self.recovery_armed:
            self.current_candidates.clear()
            self.pending_transaction = None
            self.recovery_armed = False
            self.awaiting_authoritative_epoch = True
            self.lost_streak = 0
            return True
        self.lost_streak = 0
        return False

    def set_authoritative_epoch(self, epoch, epoch_start_stamp_ns=None):
        require(isinstance(epoch, int) and epoch >= 0,
                "invalid authoritative reset epoch")
        if epoch_start_stamp_ns is None:
            epoch_start_stamp_ns = 0 if epoch == 0 else None
        require(isinstance(epoch_start_stamp_ns, int) and
                epoch_start_stamp_ns >= 0,
                "invalid authoritative epoch cutoff stamp")
        require((epoch == 0 and epoch_start_stamp_ns == 0) or
                (epoch > 0 and epoch_start_stamp_ns > 0),
                "authoritative epoch cutoff does not match the epoch")
        require(epoch >= self.reset_epoch, "reset epoch cannot go backwards")
        if epoch > self.reset_epoch:
            self.reset_epoch = epoch
            self.epoch_start_stamp_ns = epoch_start_stamp_ns
            self.current_candidates.clear()
            self.pending_transaction = None
            self.lost_streak = 0
            self.recovery_armed = False
            self.awaiting_authoritative_epoch = False
            self.map_graph_synchronized_with_core = False
        else:
            require(epoch_start_stamp_ns == self.epoch_start_stamp_ns,
                    "authoritative cutoff changed within one epoch")

    def add_binding(self, raw, now):
        require(math.isfinite(now), "binding receipt time must be finite")
        binding = validate_binding(raw)
        require(not self.awaiting_authoritative_epoch,
                "binding arrived before authoritative reset epoch")
        require(binding["image_stamp_ns"] >= self.epoch_start_stamp_ns,
                "binding predates the authoritative epoch cutoff")
        # Core epoch is authoritative when available.  A stale binder event may
        # never be relabelled into the current epoch.
        require(binding["reset_epoch"] == self.reset_epoch,
                "binding reset epoch mismatch")
        tag_id = binding["tag_id"]
        if self.reset_epoch == 0 and tag_id not in self.persistent_anchors:
            views = self.anchor_views.setdefault(tag_id, [])
            if not any(item["keyframe_id"] == binding["keyframe_id"] for item in views):
                views.append(binding)
            self._materialize_same_run_anchor(tag_id)
            return []
        self.current_candidates.setdefault(tag_id, []).append((float(now), binding))
        self._expire(now)
        return self._build_proposals(tag_id, now)

    def _expire(self, now):
        for tag_id in list(self.current_candidates):
            rows = [(stamp, item) for stamp, item in self.current_candidates[tag_id]
                    if now - stamp <= self.window_s]
            if rows:
                self.current_candidates[tag_id] = rows
            else:
                del self.current_candidates[tag_id]

    def _materialize_same_run_anchor(self, tag_id):
        """Freeze a three-view exact anchor before a reset hides old poses."""
        if tag_id in self.same_run_anchors or tag_id in self.persistent_anchors:
            return
        rows = [item for item in self.anchor_views.get(tag_id, [])
                if item["keyframe_id"] in self.graph_poses]
        # Bound combinatorics without weakening the exact-node contract.
        rows = sorted(rows, key=lambda item: (
            item["reprojection_px"], item["image_stamp_ns"]))[:24]
        for selected in itertools.combinations(rows, 3):
            if any(not self._views_independent(left, right, self.graph_poses)
                   for left, right in itertools.combinations(selected, 2)):
                continue
            map_t_tags = []
            exemplars = []
            for item in selected:
                node_t_tag = pose_multiply(self.base_t_camera,
                                           item["camera_T_tag"])
                map_t_node = self.graph_poses[item["keyframe_id"]]
                map_t_tags.append(pose_multiply(map_t_node, node_t_tag))
                exemplars.append({
                    "keyframe_id": item["keyframe_id"],
                    "map_T_node": map_t_node,
                    "camera_T_tag": item["camera_T_tag"],
                })
            center = medoid(map_t_tags)
            if all(pose_delta(center, candidate)[0] <= 0.05 and
                   pose_delta(center, candidate)[1] <= 7.0
                   for candidate in map_t_tags):
                self.same_run_anchors[tag_id] = {
                    "map_T_tag": center,
                    "exemplars": exemplars,
                    "source": "frozen_same_run_exact_pre_reset_views",
                }
                return

    def _anchor_for_tag(self, tag_id):
        if tag_id in self.persistent_anchors:
            return self.persistent_anchors[tag_id]
        return self.same_run_anchors.get(tag_id)

    @staticmethod
    def _views_independent(left, right, graph_poses):
        if left["keyframe_id"] == right["keyframe_id"]:
            return False
        if left["view_bin"] != right["view_bin"]:
            return True
        if left["keyframe_id"] in graph_poses and right["keyframe_id"] in graph_poses:
            linear, angular = pose_delta(graph_poses[left["keyframe_id"]],
                                         graph_poses[right["keyframe_id"]])
            return linear >= 0.03 or angular >= 3.0
        linear, angular = pose_delta(left["camera_T_tag"], right["camera_T_tag"])
        return linear >= 0.03 or angular >= 3.0

    def _select_current_views(self, rows):
        unique = []
        seen_nodes = set()
        for _, item in sorted(rows, key=lambda pair: pair[0]):
            if item["keyframe_id"] in seen_nodes or item["keyframe_id"] not in self.graph_poses:
                continue
            if all(self._views_independent(item, old, self.graph_poses) for old in unique):
                unique.append(item)
                seen_nodes.add(item["keyframe_id"])
            if len(unique) == 3:
                return unique
        return []

    def _build_proposals(self, tag_id, now):
        if (not self.core_database_binding_valid or
                self.core_state_reset_epoch != self.reset_epoch or
                self.core_graph_revision is None or
                not self.map_graph_synchronized_with_core):
            return []
        if self.reset_epoch in self.merged_epochs or self.pending_transaction is not None:
            return []
        anchor = self._anchor_for_tag(tag_id)
        if anchor is None:
            return []
        current = self._select_current_views(self.current_candidates.get(tag_id, []))
        old = []
        for item in anchor["exemplars"]:
            if item["keyframe_id"] not in {old_item["keyframe_id"] for old_item in old}:
                old.append(item)
            if len(old) == 3:
                break
        if len(current) < 3 or len(old) < 3:
            return []

        alignments = []
        current_absolute = []
        for item in current:
            current_t_tag = pose_multiply(self.base_t_camera, item["camera_T_tag"])
            map_old_t_current = pose_multiply(anchor["map_T_tag"],
                                              pose_inverse(current_t_tag))
            current_absolute.append(map_old_t_current)
            current_graph_t_current = self.graph_poses[item["keyframe_id"]]
            alignments.append(pose_multiply(
                map_old_t_current, pose_inverse(current_graph_t_current)))
        for left, right in itertools.combinations(alignments, 2):
            linear, angular = pose_delta(left, right)
            if (linear > self.max_alignment_translation_m or
                    angular > self.max_alignment_rotation_deg):
                return []

        proposals = []
        for current_item, map_old_t_current, old_item in zip(current, current_absolute, old):
            current_t_old = pose_multiply(pose_inverse(map_old_t_current),
                                          old_item["map_T_node"])
            proposals.append({
                "from_id": current_item["keyframe_id"],
                "to_id": old_item["keyframe_id"],
                "type": LINK_USER_CLOSURE,
                "transform": current_t_old,
                "translation_sigma_m": 0.05,
                "rotation_sigma_deg": 5.0,
            })
        self.sequence += 1
        created_monotonic_s = float(now)
        canonical = json.dumps({
            "session_token": self.session_token,
            "sequence": self.sequence,
            "tag_id": tag_id,
            "reset_epoch": self.reset_epoch,
            "graph_revision": self.core_graph_revision,
            "anchor_database_sha256": self.anchor_database_sha256,
            "created_monotonic_s": created_monotonic_s,
            "evidence": [{
                "from_id": item["from_id"],
                "to_id": item["to_id"],
                "translation": list(item["transform"]["translation"]),
                "quaternion_xyzw": list(item["transform"]["quaternion_xyzw"]),
                "translation_sigma_m": item["translation_sigma_m"],
                "rotation_sigma_deg": item["rotation_sigma_deg"],
            } for item in proposals],
        }, sort_keys=True, separators=(",", ":"))
        transaction_token = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
        self.pending_transaction = {
            "transaction_token": transaction_token,
            "sequence": self.sequence,
            "tag_id": tag_id,
            "reset_epoch": self.reset_epoch,
            "graph_revision": self.core_graph_revision,
            "awaiting_authoritative_epoch": self.awaiting_authoritative_epoch,
            "created_monotonic_s": created_monotonic_s,
            "pairs": [(item["from_id"], item["to_id"]) for item in proposals],
        }
        return proposals

    def apply_core_status(self, status):
        require(isinstance(status, dict), "core status must be an object")
        expected_fields = {
            "schema_version", "producer", "result", "reason",
            "transaction_token", "tag_id", "source_reset_epoch",
            "observer_graph_revision", "core_reset_epoch",
            "core_graph_revision", "database_sha256",
            "session_token_sha256", "evidence_pairs",
        }
        require(set(status) == expected_fields and
                status.get("schema_version") == 1 and
                status.get("producer") == CORE_PRODUCER and
                isinstance(status.get("reason"), str),
                "unexpected core status schema or producer")
        if self.pending_transaction is None:
            return False
        expected_session_sha256 = hashlib.sha256(
            self.session_token.encode("ascii")).hexdigest()
        if (status.get("transaction_token") !=
                self.pending_transaction["transaction_token"] or
                status.get("tag_id") != self.pending_transaction["tag_id"] or
                status.get("source_reset_epoch") != self.pending_transaction["reset_epoch"] or
                status.get("observer_graph_revision") !=
                self.pending_transaction["graph_revision"] or
                status.get("database_sha256") != self.anchor_database_sha256 or
                status.get("session_token_sha256") != expected_session_sha256):
            return False
        result = status.get("result")
        require(result in ("committed", "rejected"), "invalid terminal core result")
        core_revision = status.get("core_graph_revision")
        core_epoch = status.get("core_reset_epoch")
        evidence_pairs = status.get("evidence_pairs")
        expected_pairs = [list(item) for item in self.pending_transaction["pairs"]]
        require(type(core_revision) is int and type(core_epoch) is int,
                "terminal core epoch or revision is not an integer")
        if result == "committed":
            if (core_epoch != self.pending_transaction["reset_epoch"] or
                    core_revision != self.pending_transaction["graph_revision"] + 1 or
                    evidence_pairs != expected_pairs):
                return False
        elif evidence_pairs == expected_pairs:
            if (core_epoch != self.pending_transaction["reset_epoch"] or
                    core_revision < self.pending_transaction["graph_revision"]):
                return False
        elif evidence_pairs == []:
            # Core may reject freshness or database/session invariants before it
            # parses the evidence array.  The surrounding transaction identity
            # is still exact, so this is a terminal rejection, never a commit.
            if (core_epoch < self.pending_transaction["reset_epoch"] or
                    core_revision < self.pending_transaction["graph_revision"]):
                return False
        else:
            return False
        if result == "committed":
            self.merged_epochs.add(self.reset_epoch)
        self.pending_transaction = None
        return result == "committed"

    def snapshot(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "producer": PRODUCER,
            "reset_epoch": self.reset_epoch,
            "core_graph_revision": self.core_graph_revision,
            "core_state_reset_epoch": self.core_state_reset_epoch,
            "core_database_binding_valid": self.core_database_binding_valid,
            "map_graph_message_count": self.map_graph_message_count,
            "map_graph_synchronized_with_core": self.map_graph_synchronized_with_core,
            "persistent_tag_count": len(self.persistent_anchors),
            "same_run_observed_tag_count": len(self.anchor_views),
            "same_run_frozen_anchor_tag_count": len(self.same_run_anchors),
            "session_token_sha256": hashlib.sha256(
                self.session_token.encode("ascii")).hexdigest(),
            "pending_transaction": self.pending_transaction,
            "merged_epochs": sorted(self.merged_epochs),
            "graph_mutation_owned_by_this_node": False,
        }


def write_all(descriptor, data):
    offset = 0
    while offset < len(data):
        written = os.write(descriptor, data[offset:])
        require(written > 0, "short file write")
        offset += written


def secure_append(path, payload):
    descriptor = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT |
                         O_CLOEXEC | O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_nlink == 1,
                "event log is not a single-link regular file")
        if hasattr(os, "getuid"):
            require(info.st_uid == os.getuid(), "event log owner mismatch")
        os.fchmod(descriptor, 0o600)
        data = (json.dumps(payload, sort_keys=True) + "\n").encode("utf-8")
        write_all(descriptor, data)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path, payload):
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.{time.monotonic_ns()}.tmp")
    require(not path.is_symlink() and not temporary.exists(), "unsafe state path")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                         O_CLOEXEC | O_NOFOLLOW, 0o600)
    try:
        data = (json.dumps(payload, sort_keys=True) + "\n").encode("utf-8")
        write_all(descriptor, data)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o600)
    finally:
        os.close(descriptor)
    try:
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | O_CLOEXEC |
                            getattr(os, "O_DIRECTORY", 0) | O_NOFOLLOW)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def parse_metadata(raw):
    payload = strict_json_loads(raw)
    require(payload.get("schema_version") == 1 and payload.get("producer") ==
            "wbrov_apriltag_landmark_node", "invalid detector metadata")
    require(payload.get("geometry_valid_before_temporal_confirmation") is True and
            payload.get("graph_edge_added") is False,
            "detector crossed observer-only boundary")
    require(payload.get("camera_frame") == "zed_left_camera_optical_frame",
            "detector camera frame mismatch")
    stamp_ns = payload.get("image_stamp_ns")
    require(type(stamp_ns) is int and stamp_ns > 0, "invalid metadata stamp")
    observations = []
    raw_observations = payload.get("observations")
    require(isinstance(raw_observations, list), "metadata observations must be a list")
    for raw_observation in raw_observations:
        require(isinstance(raw_observation, dict), "metadata observation must be an object")
        item = dict(raw_observation)
        item["camera_frame"] = payload.get("camera_frame")
        observations.append(item)
    require(observations, "metadata has no observations")
    return stamp_ns, observations


def map_id_from_info(message):
    for key, value in zip(message.stats_keys, message.stats_values):
        if key == "Loop/Map_id/":
            return int(round(float(value)))
    return -1


def build_transaction_payload(engine, proposals):
    require(len(proposals) == 3 and engine.pending_transaction is not None,
            "proposal publication requires one complete transaction")
    evidence = []
    for proposal in proposals:
        require(proposal.get("type") == LINK_USER_CLOSURE,
                "safe transaction accepts only user closures")
        linear_information = 1.0 / proposal["translation_sigma_m"] ** 2
        angular_information = 1.0 / math.radians(
            proposal["rotation_sigma_deg"]) ** 2
        matrix = [0.0] * 36
        for index in range(3):
            matrix[index * 6 + index] = linear_information
        for index in range(3, 6):
            matrix[index * 6 + index] = angular_information
        evidence.append({
            "from_id": proposal["from_id"],
            "to_id": proposal["to_id"],
            "translation": list(proposal["transform"]["translation"]),
            "quaternion_xyzw": list(
                proposal["transform"]["quaternion_xyzw"]),
            "information_row_major": matrix,
        })
    transaction = engine.pending_transaction
    return {
        "schema_version": 1,
        "producer": PRODUCER,
        "session_token": engine.session_token,
        "transaction_token": transaction["transaction_token"],
        "tag_id": transaction["tag_id"],
        "source_reset_epoch": transaction["reset_epoch"],
        "observer_graph_revision": transaction["graph_revision"],
        "anchor_database_sha256": engine.anchor_database_sha256,
        "created_monotonic_s": transaction["created_monotonic_s"],
        "evidence": evidence,
    }


def proposal_retry_action(wire, pending_transaction, now):
    """Choose the fail-closed action for one immutable wire transaction."""
    if wire is None:
        return "none"
    if (pending_transaction is None or
            pending_transaction.get("transaction_token") != wire.get("token")):
        return "clear"
    age = float(now) - float(wire["created_monotonic_s"])
    if age >= PROPOSAL_TERMINAL_TIMEOUT_S:
        return "timeout"
    if (wire["attempts"] < PROPOSAL_MAX_ATTEMPTS and
            float(now) - float(wire["last_publish_monotonic_s"]) >=
            PROPOSAL_RETRY_INTERVAL_S):
        return "retry"
    return "wait"



PURE_ENGINE_SOURCE_SHA256 = '1491ec42e6c0ef0bda9f74ce017c1b3fde41815289ddffa7e5d9fb993cde1682'
PURE_ENGINE_ROS_NETWORK_DISABLED = True
