#!/usr/bin/env python3
"""Build a source-DB-bound multi-view AprilTag anchor file after SLAM stops."""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import sqlite3
import stat
import struct
import tempfile
import time
import zlib


SCHEMA_VERSION = 4
BASE_T_CAMERA_TRANSLATION = (0.0, 0.0, 0.0)
BASE_T_CAMERA_QUATERNION = (-0.5, 0.5, -0.5, 0.5)


class AnchorBuildError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise AnchorBuildError(message)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_identity(path):
    info = path.lstat()
    require(stat.S_ISREG(info.st_mode) and not stat.S_ISLNK(info.st_mode),
            f"not a regular non-symlink file: {path}")
    require(info.st_nlink == 1, f"file must have one hard link: {path}")
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns,
            info.st_ctime_ns, stat.S_IMODE(info.st_mode), info.st_uid, info.st_nlink)


def require_no_sidecars(database):
    for suffix in ("-wal", "-shm", "-journal"):
        require(not os.path.lexists(str(database) + suffix),
                f"database sidecar present: {suffix}")


def visible_holders(database):
    targets = {str(database.resolve()) + suffix for suffix in ("", "-wal", "-shm", "-journal")}
    holders = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            descriptors = list((process / "fd").iterdir())
        except (FileNotFoundError, PermissionError):
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
            except (FileNotFoundError, PermissionError, OSError):
                continue
            if target.removesuffix(" (deleted)") in targets:
                holders.append({"pid": int(process.name), "fd": descriptor.name,
                                "target": target})
    return holders


def matrix_from_pose(values):
    require(len(values) == 12 and all(math.isfinite(value) for value in values),
            "pose must be a finite 3x4 matrix")
    return [
        [values[0], values[1], values[2], values[3]],
        [values[4], values[5], values[6], values[7]],
        [values[8], values[9], values[10], values[11]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_from_xyzw(translation, quaternion):
    require(len(translation) == 3 and len(quaternion) == 4, "invalid pose dimensions")
    require(all(math.isfinite(value) for value in (*translation, *quaternion)),
            "pose contains a non-finite value")
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    require(norm > 1e-9 and abs(norm - 1.0) <= 0.05, "quaternion is not near unit")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    tx, ty, tz = translation
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), tx],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), ty],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), tz],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_from_json_pose(raw, label):
    require(isinstance(raw, dict), f"{label} must be an object")
    translation_value = raw.get("translation")
    quaternion_value = raw.get("quaternion_xyzw")
    require(isinstance(translation_value, list) and
            isinstance(quaternion_value, list),
            f"{label} must contain translation and quaternion_xyzw arrays")
    return matrix_from_xyzw(translation_value, quaternion_value)


def require_sha256(value, label):
    require(isinstance(value, str) and len(value) == 64 and
            all(character in "0123456789abcdef" for character in value),
            f"{label} must be a lowercase SHA256")
    return value


def unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"duplicate JSON key: {key}")
        result[key] = value
    return result


def multiply(left, right):
    return [[sum(left[row][index] * right[index][column] for index in range(4))
             for column in range(4)] for row in range(4)]


def translation(matrix):
    return [matrix[row][3] for row in range(3)]


def quaternion_xyzw(matrix):
    rotation = matrix
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (0.25*scale, (rotation[2][1]-rotation[1][2])/scale,
                      (rotation[0][2]-rotation[2][0])/scale,
                      (rotation[1][0]-rotation[0][1])/scale)
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0]-rotation[1][1]-rotation[2][2]) * 2.0
        w, x, y, z = ((rotation[2][1]-rotation[1][2])/scale, 0.25*scale,
                      (rotation[0][1]+rotation[1][0])/scale,
                      (rotation[0][2]+rotation[2][0])/scale)
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1]-rotation[0][0]-rotation[2][2]) * 2.0
        w, x, y, z = ((rotation[0][2]-rotation[2][0])/scale,
                      (rotation[0][1]+rotation[1][0])/scale, 0.25*scale,
                      (rotation[1][2]+rotation[2][1])/scale)
    else:
        scale = math.sqrt(1.0 + rotation[2][2]-rotation[0][0]-rotation[1][1]) * 2.0
        w, x, y, z = ((rotation[1][0]-rotation[0][1])/scale,
                      (rotation[0][2]+rotation[2][0])/scale,
                      (rotation[1][2]+rotation[2][1])/scale, 0.25*scale)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    return [x/norm, y/norm, z/norm, w/norm]


def rotation_delta_deg(left, right):
    trace = sum(left[row][column] * right[row][column]
                for row in range(3) for column in range(3))
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def translation_delta(left, right):
    return math.dist(translation(left), translation(right))


def decode_admin(blob_ids, blob_poses):
    ids_raw = zlib.decompress(blob_ids)
    poses_raw = zlib.decompress(blob_poses)
    require(len(ids_raw) % 4 == 0 and len(poses_raw) % 48 == 0,
            "partial Admin optimized-pose blob")
    ids = struct.unpack(f"<{len(ids_raw)//4}i", ids_raw)
    values = struct.unpack(f"<{len(poses_raw)//4}f", poses_raw)
    poses = [matrix_from_pose(values[index:index+12])
             for index in range(0, len(values), 12)]
    require(len(ids) == len(poses), "optimized ID/pose count mismatch")
    return dict(zip(ids, poses))


def load_saved_poses(database):
    uri = f"file:{database.resolve().as_posix()}?mode=ro&immutable=1"
    connection = sqlite3.connect(uri, uri=True)
    connection.execute("PRAGMA query_only=ON")
    try:
        quick_check = connection.execute("PRAGMA quick_check").fetchone()
        require(quick_check == ("ok",), f"database quick_check failed: {quick_check}")
        rows = list(connection.execute(
            "SELECT id,stamp FROM Node WHERE id>0 AND weight>=0 ORDER BY stamp,id"))
        admin = connection.execute("SELECT opt_ids,opt_poses FROM Admin").fetchone()
        require(admin is not None and admin[0] is not None and admin[1] is not None,
                "database has no saved optimized graph")
        optimized = decode_admin(admin[0], admin[1])
    finally:
        connection.close()
    saved = [(int(node_id), float(stamp)) for node_id, stamp in rows
             if int(node_id) in optimized]
    require(saved, "no saved keyframes with optimized poses")
    return optimized, saved


def optional_float(row, name):
    raw = row.get(name, "")
    if raw in (None, ""):
        return None
    value = float(raw)
    require(math.isfinite(value), f"non-finite {name}")
    return value


def view_bin(u, v):
    if u is None or v is None or not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return "unknown"
    return f"{min(2, int(v*3))}_{min(2, int(u*3))}"


def optional_event_float(event, name):
    raw = event.get(name)
    if raw is None:
        return None
    require(isinstance(raw, (int, float)) and math.isfinite(raw),
            f"non-finite binding {name}")
    return float(raw)


def parse_exact_binding_candidates(bindings_jsonl, optimized, saved,
                                   allow_empty=False):
    """Use binder-provided node IDs; never substitute a nearby DB timestamp."""
    saved_stamps = dict(saved)
    candidates = []
    seen = {}
    rejected_not_in_saved_graph = []
    with bindings_jsonl.open(encoding="utf-8") as handle:
        for line_number, raw in enumerate(handle, start=1):
            if not raw.strip():
                continue
            event = json.loads(raw, object_pairs_hook=unique_json_object)
            require(isinstance(event, dict), "binding JSONL row must be an object")
            if event.get("accepted") is not True:
                continue
            require(event.get("reason") == "exact_stamp_keyframe_binding",
                    "accepted binding has an unexpected reason")
            require(event.get("graph_edge_added") is False and
                    event.get("usable_as_absolute_anchor") is False,
                    "binding must remain observer evidence")
            stamp_ns = event.get("image_stamp_ns")
            tag_id = event.get("tag_id")
            node_id = event.get("keyframe_id")
            map_id = event.get("map_id")
            reset_epoch = event.get("reset_epoch")
            require(isinstance(stamp_ns, int) and stamp_ns > 0,
                    "invalid exact binding image stamp")
            require(isinstance(tag_id, int) and tag_id >= 0,
                    "invalid exact binding tag ID")
            require(isinstance(node_id, int) and node_id > 0,
                    "invalid exact binding keyframe ID")
            require(isinstance(map_id, int) and isinstance(reset_epoch, int) and
                    reset_epoch >= 0, "invalid binding map/epoch")
            key = (stamp_ns, tag_id)
            canonical = (node_id, map_id, reset_epoch,
                         event.get("camera_translation_m"),
                         event.get("camera_quaternion_xyzw"))
            if key in seen:
                require(seen[key] == canonical,
                        "conflicting duplicate exact binding")
                continue
            seen[key] = canonical
            if node_id not in optimized or node_id not in saved_stamps:
                rejected_not_in_saved_graph.append({
                    "tag_id": tag_id,
                    "keyframe_id": node_id,
                    "map_id": map_id,
                    "reset_epoch": reset_epoch,
                    "image_stamp_ns": stamp_ns,
                })
                continue
            camera_translation = event.get("camera_translation_m")
            camera_quaternion = event.get("camera_quaternion_xyzw")
            camera_t_tag = matrix_from_xyzw(camera_translation, camera_quaternion)
            base_t_camera = matrix_from_xyzw(
                BASE_T_CAMERA_TRANSLATION, BASE_T_CAMERA_QUATERNION)
            map_t_tag = multiply(multiply(optimized[node_id], base_t_camera), camera_t_tag)
            camera_frame = event.get("camera_frame")
            require(camera_frame == "zed_left_camera_optical_frame",
                    "exact binding camera frame does not match the anchor contract")
            u = optional_event_float(event, "image_u_norm")
            v = optional_event_float(event, "image_v_norm")
            reprojection = optional_event_float(event, "reprojection_px")
            mean_side = optional_event_float(event, "mean_side_px")
            measured_range = optional_event_float(event, "range_m")
            require(u is not None and v is not None and
                    0.0 <= u <= 1.0 and 0.0 <= v <= 1.0,
                    "exact binding has invalid normalized image position")
            require(reprojection is not None and reprojection >= 0.0,
                    "exact binding has invalid reprojection error")
            require(mean_side is not None and mean_side > 0.0,
                    "exact binding has invalid apparent tag size")
            require(measured_range is not None and measured_range > 0.0,
                    "exact binding has invalid measured range")
            candidates.append({
                "tag_id": tag_id,
                "saved_node_id": node_id,
                "image_stamp_s": stamp_ns / 1_000_000_000.0,
                "image_stamp_ns": stamp_ns,
                "saved_node_stamp_s": saved_stamps[node_id],
                "stamp_delta_s": None,
                "binding_policy": "runtime_exact_image_stamp_and_Info.ref_id",
                "map_id": map_id,
                "reset_epoch": reset_epoch,
                "reprojection_px": reprojection,
                "image_u_norm": u,
                "image_v_norm": v,
                "view_bin": view_bin(u, v),
                "mean_side_px": mean_side,
                "range_m": measured_range,
                "camera_frame": camera_frame,
                "camera_T_tag": {
                    "translation": list(camera_translation),
                    "quaternion_xyzw": list(camera_quaternion),
                },
                "map_T_tag": {
                    "translation": translation(map_t_tag),
                    "quaternion_xyzw": quaternion_xyzw(map_t_tag),
                },
                "_camera_matrix": camera_t_tag,
                "_map_matrix": map_t_tag,
                "_sort_reprojection_px": (
                    reprojection if reprojection is not None else 1e9),
                "source_binding_line": line_number,
            })
    if not allow_empty:
        require(seen, "binding log has no accepted exact bindings")
    return candidates, {
        "accepted_exact_binding_keys": len(seen),
        "retained_saved_graph_bindings": len(candidates),
        "rejected_not_in_saved_optimized_graph": rejected_not_in_saved_graph,
        "nearest_timestamp_substitution_used": False,
    }


def finite_number(value, label, minimum=None, maximum=None):
    require(isinstance(value, (int, float)) and not isinstance(value, bool) and
            math.isfinite(value), f"{label} must be finite")
    number = float(value)
    if minimum is not None:
        require(number >= minimum, f"{label} is below its minimum")
    if maximum is not None:
        require(number <= maximum, f"{label} is above its maximum")
    return number


def strict_integer(value, label, minimum=None):
    require(isinstance(value, int) and not isinstance(value, bool),
            f"{label} must be an integer")
    if minimum is not None:
        require(value >= minimum, f"{label} is below its minimum")
    return value


def parse_previous_anchor_candidates(previous_anchors, optimized, saved,
                                     expected_database_sha256,
                                     expected_anchors_sha256,
                                     translation_threshold,
                                     rotation_threshold):
    """Rebind every inherited exact exemplar to the final optimized graph.

    The parent's map_T_tag values are input evidence only.  They are parsed for
    integrity diagnostics, never copied into the child library.
    """
    require_sha256(expected_database_sha256, "previous database SHA256")
    require_sha256(expected_anchors_sha256, "previous anchors SHA256")
    require(sha256(previous_anchors) == expected_anchors_sha256,
            "previous anchor file SHA256 does not match the supplied value")
    with previous_anchors.open(encoding="utf-8") as handle:
        parent = json.load(handle, object_pairs_hook=unique_json_object)
    require(isinstance(parent, dict), "previous anchor root must be an object")
    parent_schema = strict_integer(
        parent.get("schema_version"), "previous schema_version")
    require(parent_schema in (3, 4),
            "previous anchors must use exact-binding schema 3 or 4")
    require(parent.get("producer") == "wbrov_apriltag_multiview_anchor_builder",
            "previous anchors have an unexpected producer")
    require(parent.get("source_database_sha256") == expected_database_sha256,
            "previous anchors are not bound to the supplied parent database SHA256")
    require(parent.get("source_database_mutated") is False,
            "previous anchors do not attest an unmodified stopped source database")
    require(parent.get("camera_frame") == "zed_left_camera_optical_frame" and
            parent.get("base_frame") == "base_link" and
            parent.get("map_frame") == "map",
            "previous anchor frame contract does not match")
    expected_extrinsic = matrix_from_xyzw(
        BASE_T_CAMERA_TRANSLATION, BASE_T_CAMERA_QUATERNION)
    parent_extrinsic = matrix_from_json_pose(
        parent.get("base_T_camera"), "previous base_T_camera")
    require(translation_delta(parent_extrinsic, expected_extrinsic) <= 1e-6 and
            rotation_delta_deg(parent_extrinsic, expected_extrinsic) <= 1e-4,
            "previous base_T_camera does not match the exact binding contract")
    policy = parent.get("policy")
    require(isinstance(policy, dict) and
            policy.get("runtime_exact_keyframe_binding_required") is True,
            "previous anchors do not require runtime exact keyframe binding")
    parent_tags = parent.get("tags")
    require(isinstance(parent_tags, dict) and parent_tags,
            "previous anchors contain no eligible tags")

    saved_stamps = dict(saved)
    inherited = {}
    tag_diagnostics = {}
    all_nodes = []
    require(all(isinstance(tag_key, str) and tag_key.isdigit() and
                str(int(tag_key)) == tag_key for tag_key in parent_tags),
            "previous tag keys must be canonical non-negative integers")
    for tag_key, tag_payload in sorted(parent_tags.items(),
                                       key=lambda item: int(item[0])):
        tag_id = int(tag_key)
        require(isinstance(tag_payload, dict),
                f"previous tag {tag_id} must be an object")
        require(tag_payload.get("rtabmap_landmark_id") == tag_id + 1,
                f"previous tag {tag_id} landmark ID mismatch")
        old_root_map = matrix_from_json_pose(
            tag_payload.get("map_T_tag"), f"previous tag {tag_id} map_T_tag")
        exemplars = tag_payload.get("view_exemplars")
        require(isinstance(exemplars, list) and 3 <= len(exemplars) <= 12,
                f"previous tag {tag_id} must contain 3..12 exact exemplars")
        rows = []
        node_ids = set()
        old_to_recomputed = []
        for exemplar_index, exemplar in enumerate(exemplars):
            label = f"previous tag {tag_id} exemplar {exemplar_index}"
            require(isinstance(exemplar, dict), f"{label} must be an object")
            require(strict_integer(exemplar.get("tag_id"), f"{label} tag_id", 0)
                    == tag_id, f"{label} tag ID mismatch")
            node_id = strict_integer(
                exemplar.get("saved_node_id"), f"{label} saved_node_id", 1)
            require(node_id not in node_ids,
                    f"previous tag {tag_id} repeats saved node {node_id}")
            node_ids.add(node_id)
            require(node_id in optimized and node_id in saved_stamps,
                    f"previous tag {tag_id} saved node {node_id} is absent from "
                    "the final saved optimized graph")
            image_stamp_ns = strict_integer(
                exemplar.get("image_stamp_ns"), f"{label} image_stamp_ns", 1)
            image_stamp_s = finite_number(
                exemplar.get("image_stamp_s"), f"{label} image_stamp_s")
            require(abs(image_stamp_s - image_stamp_ns / 1_000_000_000.0) <= 1e-6,
                    f"{label} image stamp representations disagree")
            saved_node_stamp_s = finite_number(
                exemplar.get("saved_node_stamp_s"),
                f"{label} saved_node_stamp_s")
            require(abs(saved_node_stamp_s - saved_stamps[node_id]) <= 1e-6,
                    f"{label} saved node stamp does not match the final database")
            require(exemplar.get("stamp_delta_s") is None,
                    f"{label} is not an exact keyframe binding")
            require(exemplar.get("binding_policy") ==
                    "runtime_exact_image_stamp_and_Info.ref_id",
                    f"{label} has an unexpected binding policy")
            map_id = strict_integer(exemplar.get("map_id"), f"{label} map_id", 0)
            reset_epoch = strict_integer(
                exemplar.get("reset_epoch"), f"{label} reset_epoch", 0)
            reprojection = finite_number(
                exemplar.get("reprojection_px"), f"{label} reprojection_px", 0.0, 3.0)
            u = finite_number(
                exemplar.get("image_u_norm"), f"{label} image_u_norm", 0.0, 1.0)
            v = finite_number(
                exemplar.get("image_v_norm"), f"{label} image_v_norm", 0.0, 1.0)
            expected_view_bin = view_bin(u, v)
            require(exemplar.get("view_bin") == expected_view_bin,
                    f"{label} view bin does not match its normalized location")
            mean_side = finite_number(
                exemplar.get("mean_side_px"), f"{label} mean_side_px", 18.0)
            measured_range = finite_number(
                exemplar.get("range_m"), f"{label} range_m", 1e-12, 1.5)
            require(exemplar.get("camera_frame") ==
                    "zed_left_camera_optical_frame",
                    f"{label} camera frame does not match")
            source_binding_line = strict_integer(
                exemplar.get("source_binding_line"),
                f"{label} source_binding_line", 1)
            camera_t_tag = matrix_from_json_pose(
                exemplar.get("camera_T_tag"), f"{label} camera_T_tag")
            old_map_t_tag = matrix_from_json_pose(
                exemplar.get("map_T_tag"), f"{label} map_T_tag")
            recomputed_map_t_tag = multiply(
                multiply(optimized[node_id], expected_extrinsic), camera_t_tag)
            old_to_recomputed.append({
                "saved_node_id": node_id,
                "translation_m": translation_delta(old_map_t_tag,
                                                    recomputed_map_t_tag),
                "rotation_deg": rotation_delta_deg(old_map_t_tag,
                                                   recomputed_map_t_tag),
            })
            rows.append({
                "tag_id": tag_id,
                "saved_node_id": node_id,
                "image_stamp_s": image_stamp_s,
                "image_stamp_ns": image_stamp_ns,
                "saved_node_stamp_s": saved_node_stamp_s,
                "stamp_delta_s": None,
                "binding_policy": "runtime_exact_image_stamp_and_Info.ref_id",
                "map_id": map_id,
                "reset_epoch": reset_epoch,
                "reprojection_px": reprojection,
                "image_u_norm": u,
                "image_v_norm": v,
                "view_bin": expected_view_bin,
                "mean_side_px": mean_side,
                "range_m": measured_range,
                "camera_frame": "zed_left_camera_optical_frame",
                "camera_T_tag": {
                    "translation": translation(camera_t_tag),
                    "quaternion_xyzw": quaternion_xyzw(camera_t_tag),
                },
                "map_T_tag": {
                    "translation": translation(recomputed_map_t_tag),
                    "quaternion_xyzw": quaternion_xyzw(recomputed_map_t_tag),
                },
                "source_binding_line": source_binding_line,
                "provenance": {
                    "kind": "previous_exact_anchor_recomputed",
                    "parent_database_sha256": expected_database_sha256,
                    "parent_anchor_sha256": expected_anchors_sha256,
                    "map_T_tag_recomputed_from_final_saved_pose": True,
                },
                "_camera_matrix": camera_t_tag,
                "_map_matrix": recomputed_map_t_tag,
                "_parent_map_matrix": old_map_t_tag,
                "_sort_reprojection_px": reprojection,
                "_inherited": True,
            })
            all_nodes.append(node_id)
        for left, right in itertools.combinations(rows, 2):
            require(translation_delta(left["_parent_map_matrix"],
                                      right["_parent_map_matrix"])
                    <= translation_threshold and
                    rotation_delta_deg(left["_parent_map_matrix"],
                                       right["_parent_map_matrix"])
                    <= rotation_threshold,
                    f"previous tag {tag_id} parent exemplars are not a rigid "
                    "all-pairs consensus")
            require(compatible(left, right, translation_threshold,
                               rotation_threshold),
                    f"previous tag {tag_id} becomes non-rigid under the final "
                    "optimized graph")
        for row in rows:
            require(translation_delta(old_root_map, row["_parent_map_matrix"])
                    <= translation_threshold and
                    rotation_delta_deg(old_root_map,
                                       row["_parent_map_matrix"])
                    <= rotation_threshold,
                    f"previous tag {tag_id} root pose conflicts with its parent "
                    "exemplars")
        require(len(diverse_views(rows)) >= 3,
                f"previous tag {tag_id} no longer has three diverse exact views")
        recomputed_anchor = medoid(rows, translation_threshold, rotation_threshold)
        inherited[tag_id] = rows
        tag_diagnostics[str(tag_id)] = {
            "inherited_exemplar_count": len(rows),
            "inherited_saved_node_ids": sorted(node_ids),
            "old_root_to_recomputed_medoid_translation_m": translation_delta(
                old_root_map, recomputed_anchor["_map_matrix"]),
            "old_root_to_recomputed_medoid_rotation_deg": rotation_delta_deg(
                old_root_map, recomputed_anchor["_map_matrix"]),
            "old_exemplar_to_recomputed_pose_deltas": old_to_recomputed,
        }
    return inherited, {
        "parent_schema_version": parent_schema,
        "inherited_tag_ids": sorted(inherited),
        "inherited_exemplar_count": sum(len(rows) for rows in inherited.values()),
        "inherited_saved_node_ids": sorted(set(all_nodes)),
        "all_parent_nodes_present": True,
        "parent_anchor_mutated": False,
    }, tag_diagnostics


def parse_candidates(gate_csv, optimized, saved, max_stamp_delta_s):
    base_t_camera = matrix_from_xyzw(
        BASE_T_CAMERA_TRANSLATION, BASE_T_CAMERA_QUATERNION)
    candidates = []
    with gate_csv.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"stamp", "tag_id", "reprojection_px", "camera_x", "camera_y",
                    "camera_z", "camera_qx", "camera_qy", "camera_qz", "camera_qw",
                    "geometry_reason"}
        require(reader.fieldnames is not None and required.issubset(reader.fieldnames),
                "pose gate CSV schema is incomplete")
        for row_index, row in enumerate(reader, start=2):
            if row.get("geometry_reason") != "square_pose_unambiguous":
                continue
            stamp_value = float(row["stamp"])
            nearest_id, nearest_stamp = min(saved, key=lambda item: abs(item[1]-stamp_value))
            stamp_delta = abs(nearest_stamp - stamp_value)
            if stamp_delta > max_stamp_delta_s:
                continue
            camera_translation = [float(row[name]) for name in
                                  ("camera_x", "camera_y", "camera_z")]
            camera_quaternion = [float(row[name]) for name in
                                 ("camera_qx", "camera_qy", "camera_qz", "camera_qw")]
            camera_t_tag = matrix_from_xyzw(camera_translation, camera_quaternion)
            map_t_tag = multiply(multiply(optimized[nearest_id], base_t_camera), camera_t_tag)
            u = optional_float(row, "image_u_norm")
            v = optional_float(row, "image_v_norm")
            candidates.append({
                "tag_id": int(row["tag_id"]), "saved_node_id": nearest_id,
                "image_stamp_s": stamp_value, "saved_node_stamp_s": nearest_stamp,
                "stamp_delta_s": stamp_delta,
                "reprojection_px": float(row["reprojection_px"]),
                "image_u_norm": u, "image_v_norm": v, "view_bin": view_bin(u, v),
                "mean_side_px": optional_float(row, "mean_side_px"),
                "camera_T_tag": {"translation": camera_translation,
                                 "quaternion_xyzw": camera_quaternion},
                "map_T_tag": {"translation": translation(map_t_tag),
                              "quaternion_xyzw": quaternion_xyzw(map_t_tag)},
                "_camera_matrix": camera_t_tag, "_map_matrix": map_t_tag,
                "source_csv_row": row_index,
            })
    return candidates


def compatible(left, right, translation_threshold, rotation_threshold):
    return (translation_delta(left["_map_matrix"], right["_map_matrix"])
            <= translation_threshold and
            rotation_delta_deg(left["_map_matrix"], right["_map_matrix"])
            <= rotation_threshold)


def reprojection_rank(row):
    value = row.get("_sort_reprojection_px", row.get("reprojection_px"))
    return float(value) if isinstance(value, (int, float)) else 1e9


def greedy_all_pairs_consensus(rows, translation_threshold, rotation_threshold):
    """Return a deterministic all-pairs-consistent subset in polynomial time.

    This may conservatively miss a larger clique, but it never admits a pair
    outside the thresholds and remains bounded when a tag is seen hundreds of
    times.
    """
    count = len(rows)
    adjacency = [set() for _ in rows]
    for left in range(count):
        for right in range(left + 1, count):
            if compatible(rows[left], rows[right], translation_threshold, rotation_threshold):
                adjacency[left].add(right)
                adjacency[right].add(left)
    seed_order = sorted(range(count), key=lambda index: (
        reprojection_rank(rows[index]), rows[index]["image_stamp_s"]))[:128]
    best = []
    for seed in seed_order:
        indices = [seed]
        candidates = set(adjacency[seed])
        while candidates:
            selected = max(candidates, key=lambda index: (
                len(adjacency[index] & candidates),
                -reprojection_rank(rows[index]),
                -rows[index]["image_stamp_s"]))
            indices.append(selected)
            candidates &= adjacency[selected]
        cluster = [rows[index] for index in indices]
        if (len(cluster), -sum(reprojection_rank(item) for item in cluster)) > (
                len(best), -sum(reprojection_rank(item) for item in best)):
            best = cluster
    return best if len(best) >= 3 else []


def medoid(rows, translation_threshold, rotation_threshold):
    costs = []
    for left in rows:
        costs.append(sum(
            translation_delta(left["_map_matrix"], right["_map_matrix"]) /
            translation_threshold +
            rotation_delta_deg(left["_map_matrix"], right["_map_matrix"]) /
            rotation_threshold for right in rows))
    return rows[min(range(len(rows)), key=lambda index: costs[index])]


def duplicates_selected_view(row, selected):
    if any(row["saved_node_id"] == old["saved_node_id"] for old in selected):
        return True
    for old in selected:
        same_bin = row["view_bin"] == old["view_bin"] and row["view_bin"] != "unknown"
        range_delta = abs(math.dist([0, 0, 0], row["camera_T_tag"]["translation"]) -
                          math.dist([0, 0, 0], old["camera_T_tag"]["translation"]))
        rotation = rotation_delta_deg(row["_camera_matrix"], old["_camera_matrix"])
        bearing = math.dist(row["camera_T_tag"]["translation"],
                            old["camera_T_tag"]["translation"])
        if same_bin and range_delta < 0.08 and rotation < 12.0 and bearing < 0.08:
            return True
    return False


def append_diverse_views(selected, rows, maximum=12):
    result = list(selected)
    require(len(result) <= maximum, "selected view backbone exceeds its maximum")
    if len(result) == maximum:
        return result
    for row in sorted(rows, key=lambda item: (
            reprojection_rank(item), item["image_stamp_s"])):
        if not duplicates_selected_view(row, result):
            result.append(row)
        if len(result) >= maximum:
            break
    return result


def diverse_views(rows, maximum=12):
    return append_diverse_views([], rows, maximum)


def strongest_session_consensus(rows, translation_threshold, rotation_threshold):
    """Never manufacture one anchor from observations across reset sessions."""
    session_groups = {}
    for row in rows:
        session_key = (row.get("map_id"), row.get("reset_epoch"))
        session_groups.setdefault(session_key, []).append(row)
    require(session_groups, "tag has no source session")
    candidates = []
    for session_key, session_rows in session_groups.items():
        consensus = greedy_all_pairs_consensus(
            session_rows, translation_threshold, rotation_threshold)
        candidates.append((
            len(consensus),
            -sum(reprojection_rank(item) for item in consensus),
            session_key,
            consensus,
        ))
    _, _, selected_session, consensus = max(
        candidates, key=lambda item: (item[0], item[1], item[2]))
    return selected_session, consensus, session_groups


def strip_private(row):
    return {key: value for key, value in row.items() if not key.startswith("_")}


def pairwise_spreads(rows):
    pairs = list(itertools.combinations(rows, 2))
    require(pairs, "anchor consensus has fewer than two exemplars")
    return (
        max(translation_delta(left["_map_matrix"], right["_map_matrix"])
            for left, right in pairs),
        max(rotation_delta_deg(left["_map_matrix"], right["_map_matrix"])
            for left, right in pairs),
    )


def eligible_session_consensuses(rows, translation_threshold, rotation_threshold):
    session_groups = {}
    eligible = []
    for row in rows:
        session_key = (row.get("map_id"), row.get("reset_epoch"))
        session_groups.setdefault(session_key, []).append(row)
    for session_key, session_rows in session_groups.items():
        consensus = greedy_all_pairs_consensus(
            session_rows, translation_threshold, rotation_threshold)
        selected = diverse_views(consensus)
        if (len({row["saved_node_id"] for row in consensus}) >= 3 and
                len(selected) >= 3):
            eligible.append((session_key, consensus, selected))
    return session_groups, eligible


def build_new_tag(tag_id, rows, translation_threshold, rotation_threshold):
    selected_session, consensus, session_groups = strongest_session_consensus(
        rows, translation_threshold, rotation_threshold)
    selected = diverse_views(consensus)
    distinct_nodes = sorted({row["saved_node_id"] for row in consensus})
    eligible = len(distinct_nodes) >= 3 and len(selected) >= 3
    diagnostic = {
        "raw_exact_saved_candidate_count": len(rows),
        "consensus_count": len(consensus),
        "distinct_saved_nodes": distinct_nodes,
        "selected_view_count": len(selected),
        "selected_view_bins": sorted({row["view_bin"] for row in selected}),
        "session_candidate_counts": {
            f"map_{key[0]}_epoch_{key[1]}": len(value)
            for key, value in sorted(session_groups.items())
        },
        "selected_source_session": {
            "map_id": selected_session[0],
            "reset_epoch": selected_session[1],
        },
        "reset_epochs": sorted({row.get("reset_epoch") for row in rows
                                if row.get("reset_epoch") is not None}),
        "map_ids": sorted({row.get("map_id") for row in rows
                           if row.get("map_id") is not None}),
        "localization_eligible": eligible,
        "inherited": False,
    }
    if not eligible:
        return None, diagnostic
    anchor = medoid(consensus, translation_threshold, rotation_threshold)
    max_translation, max_rotation = pairwise_spreads(consensus)
    return {
        "rtabmap_landmark_id": tag_id + 1,
        "anchor_pose_policy": "medoid of largest all-pairs map_T_tag consensus",
        "map_T_tag": anchor["map_T_tag"],
        "view_exemplars": [strip_private(row) for row in selected],
        "source_consensus": {
            "distinct_saved_nodes": len(distinct_nodes),
            "source_map_id": selected_session[0],
            "source_reset_epoch": selected_session[1],
            "max_pairwise_translation_m": max_translation,
            "max_pairwise_rotation_deg": max_rotation,
            "inherited_exact_exemplars": 0,
            "fresh_exact_consensus": len(consensus),
            "field_validated_as_absolute_anchor": False,
        },
    }, diagnostic


def merge_inherited_tag(tag_id, inherited_rows, current_rows,
                        translation_threshold, rotation_threshold,
                        previous_diagnostic):
    session_groups, eligible = eligible_session_consensuses(
        current_rows, translation_threshold, rotation_threshold)
    compatible_cohorts = []
    inherited_node_ids = {row["saved_node_id"] for row in inherited_rows}
    for session_key, consensus, selected in eligible:
        if not all(compatible(fresh, old, translation_threshold, rotation_threshold)
                   for fresh in consensus for old in inherited_rows):
            raise AnchorBuildError(
                f"current exact cohort map={session_key[0]} epoch={session_key[1]} "
                f"conflicts with inherited tag {tag_id}")
        novel_selected = [row for row in selected
                          if row["saved_node_id"] not in inherited_node_ids]
        compatible_cohorts.append((session_key, consensus, novel_selected))
    selected_session = None
    fresh_consensus = []
    if compatible_cohorts:
        selected_session, _, fresh_consensus = max(
            compatible_cohorts,
            key=lambda item: (len(item[2]), len(item[1]),
                              -sum(reprojection_rank(row) for row in item[1]),
                              item[0]))
    consensus = list(inherited_rows) + list(fresh_consensus)
    for left, right in itertools.combinations(consensus, 2):
        require(compatible(left, right, translation_threshold, rotation_threshold),
                f"merged tag {tag_id} is not an all-pairs rigid consensus")
    inherited_selected = diverse_views(inherited_rows)
    require(len(inherited_selected) == len(inherited_rows),
            f"inherited tag {tag_id} cannot retain every parent exemplar")
    selected = append_diverse_views(inherited_selected, fresh_consensus)
    require(sum(bool(row.get("_inherited")) for row in selected) >= 3,
            f"merged tag {tag_id} lost its inherited three-view backbone")
    anchor = medoid(consensus, translation_threshold, rotation_threshold)
    max_translation, max_rotation = pairwise_spreads(consensus)
    distinct_nodes = sorted({row["saved_node_id"] for row in consensus})
    diagnostic = dict(previous_diagnostic)
    diagnostic.update({
        "raw_exact_saved_candidate_count": len(current_rows),
        "eligible_current_session_count": len(eligible),
        "compatible_current_session_count": len(compatible_cohorts),
        "fresh_consensus_count": len(fresh_consensus),
        "selected_view_count": len(selected),
        "selected_view_bins": sorted({row["view_bin"] for row in selected}),
        "session_candidate_counts": {
            f"map_{key[0]}_epoch_{key[1]}": len(value)
            for key, value in sorted(session_groups.items())
        },
        "selected_source_session": (
            {"map_id": selected_session[0], "reset_epoch": selected_session[1]}
            if selected_session is not None else None),
        "localization_eligible": True,
        "inherited": True,
    })
    return {
        "rtabmap_landmark_id": tag_id + 1,
        "anchor_pose_policy": (
            "medoid after final-pose recomputation of inherited exact exemplars "
            "and compatible fresh exact consensus"),
        "map_T_tag": anchor["map_T_tag"],
        "view_exemplars": [strip_private(row) for row in selected],
        "source_consensus": {
            "distinct_saved_nodes": len(distinct_nodes),
            "source_map_id": selected_session[0] if selected_session else None,
            "source_reset_epoch": selected_session[1] if selected_session else None,
            "max_pairwise_translation_m": max_translation,
            "max_pairwise_rotation_deg": max_rotation,
            "inherited_exact_exemplars": len(inherited_rows),
            "fresh_exact_consensus": len(fresh_consensus),
            "field_validated_as_absolute_anchor": False,
        },
    }, diagnostic


def build(database, gate_csv, max_stamp_delta_s, translation_threshold,
          rotation_threshold, bindings_jsonl=None, previous_anchors=None,
          previous_database_sha256=None, previous_anchors_sha256=None):
    database = database.resolve()
    gate_csv = gate_csv.resolve() if gate_csv is not None else None
    bindings_jsonl = bindings_jsonl.resolve() if bindings_jsonl is not None else None
    previous_anchors = (previous_anchors.resolve()
                        if previous_anchors is not None else None)
    previous_values = (previous_anchors, previous_database_sha256,
                       previous_anchors_sha256)
    previous_mode = all(value is not None for value in previous_values)
    require(previous_mode or all(value is None for value in previous_values),
            "--previous-anchors and both previous SHA256 values are all-or-none")
    if previous_mode:
        require(gate_csv is None,
                "previous anchors can only be continued through exact bindings")
    else:
        require((gate_csv is None) != (bindings_jsonl is None),
                "select exactly one candidate source: gate CSV or exact bindings JSONL")
    source = bindings_jsonl if bindings_jsonl is not None else gate_csv
    if previous_mode and bindings_jsonl is None:
        source_kind = "previous_exact_anchors_only"
    elif previous_mode:
        source_kind = "previous_exact_anchors_plus_runtime_exact_keyframe_bindings_jsonl"
    else:
        source_kind = ("runtime_exact_keyframe_bindings_jsonl"
                       if bindings_jsonl is not None else
                       "legacy_nearest_stamp_gate_csv")
    input_paths = [path for path in (source, previous_anchors) if path is not None]
    require(all(database != path for path in input_paths),
            "database and anchor inputs cannot be the same file")
    require(len(set(input_paths)) == len(input_paths),
            "candidate source and previous anchors cannot be the same file")
    database_identity_before = file_identity(database)
    source_identity_before = file_identity(source) if source is not None else None
    previous_identity_before = (
        file_identity(previous_anchors) if previous_anchors is not None else None)
    require_no_sidecars(database)
    holders = visible_holders(database)
    require(not holders, f"database has visible open holders: {holders}")
    database_sha_before = sha256(database)
    source_sha_before = sha256(source) if source is not None else None
    optimized, saved = load_saved_poses(database)
    if bindings_jsonl is not None:
        candidates, binding_diagnostics = parse_exact_binding_candidates(
            bindings_jsonl, optimized, saved, allow_empty=previous_mode)
    elif gate_csv is not None:
        candidates = parse_candidates(gate_csv, optimized, saved, max_stamp_delta_s)
        binding_diagnostics = {
            "nearest_timestamp_substitution_used": True,
            "legacy_source": True,
        }
    else:
        candidates = []
        binding_diagnostics = {
            "accepted_exact_binding_keys": 0,
            "retained_saved_graph_bindings": 0,
            "rejected_not_in_saved_optimized_graph": [],
            "nearest_timestamp_substitution_used": False,
            "current_exact_bindings_supplied": False,
        }

    inherited = {}
    parent_audit = {
        "parent_schema_version": None,
        "inherited_tag_ids": [],
        "inherited_exemplar_count": 0,
        "inherited_saved_node_ids": [],
        "all_parent_nodes_present": None,
        "parent_anchor_mutated": None,
    }
    previous_diagnostics = {}
    if previous_mode:
        inherited, parent_audit, previous_diagnostics = (
            parse_previous_anchor_candidates(
                previous_anchors, optimized, saved,
                previous_database_sha256, previous_anchors_sha256,
                translation_threshold, rotation_threshold))

    tags = {}
    diagnostics = {}
    tag_ids = sorted(set(inherited) | {row["tag_id"] for row in candidates})
    for tag_id in tag_ids:
        rows = [row for row in candidates if row["tag_id"] == tag_id]
        if tag_id in inherited:
            tag, diagnostic = merge_inherited_tag(
                tag_id, inherited[tag_id], rows,
                translation_threshold, rotation_threshold,
                previous_diagnostics[str(tag_id)])
        else:
            tag, diagnostic = build_new_tag(
                tag_id, rows, translation_threshold, rotation_threshold)
        diagnostics[str(tag_id)] = diagnostic
        if tag is not None:
            tags[str(tag_id)] = tag

    require_no_sidecars(database)
    require(file_identity(database) == database_identity_before,
            "database identity changed during read-only analysis")
    require(sha256(database) == database_sha_before,
            "database bytes changed during read-only analysis")
    if source is not None:
        require(file_identity(source) == source_identity_before,
                "candidate source identity changed during analysis")
        require(sha256(source) == source_sha_before,
                "candidate source bytes changed during analysis")
    if previous_anchors is not None:
        require(file_identity(previous_anchors) == previous_identity_before,
                "previous anchor identity changed during analysis")
        require(sha256(previous_anchors) == previous_anchors_sha256,
                "previous anchor bytes changed during analysis")
    return {
        "schema_version": SCHEMA_VERSION,
        "producer": "wbrov_apriltag_multiview_anchor_builder",
        "created_unix_ns": time.time_ns(),
        "source_database": str(database),
        "source_database_sha256": database_sha_before,
        "candidate_source_kind": source_kind,
        "source_gate_csv": str(gate_csv) if gate_csv is not None else None,
        "source_gate_csv_sha256": (
            source_sha_before if gate_csv is not None else None),
        "source_exact_bindings_jsonl": (
            str(bindings_jsonl) if bindings_jsonl is not None else None),
        "source_exact_bindings_jsonl_sha256": (
            source_sha_before if bindings_jsonl is not None else None),
        "lineage": {
            "mode": "continued" if previous_mode else "new",
            "parent_database_sha256": (
                previous_database_sha256 if previous_mode else None),
            "parent_anchor_sha256": (
                previous_anchors_sha256 if previous_mode else None),
            **parent_audit,
        },
        "database_open_contract": "stopped; no visible holders; mode=ro; immutable=1; query_only=ON",
        "source_database_mutated": False,
        "camera_frame": "zed_left_camera_optical_frame",
        "base_frame": "base_link",
        "map_frame": "map",
        "base_T_camera": {"translation": list(BASE_T_CAMERA_TRANSLATION),
                          "quaternion_xyzw": list(BASE_T_CAMERA_QUATERNION)},
        "policy": {
            "required_distinct_current_frames": 3,
            "current_window_s": 8.0,
            "max_map_T_odom_translation_spread_m": 0.05,
            "max_map_T_odom_rotation_spread_deg": 7.0,
            "source_max_stamp_delta_s": (
                0.0 if bindings_jsonl is not None or previous_mode else
                max_stamp_delta_s),
            "runtime_exact_keyframe_binding_required": (
                bindings_jsonl is not None or previous_mode),
            "source_consensus_translation_m": translation_threshold,
            "source_consensus_rotation_deg": rotation_threshold,
            "one_shot": True,
        },
        "tags": tags,
        "binding_diagnostics": binding_diagnostics,
        "diagnostics": diagnostics,
    }


def atomic_write(path, payload):
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.is_symlink() and not path.parent.is_symlink(), "unsafe output symlink")
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def main(arguments=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--database", type=Path)
    parser.add_argument("--gate-csv", type=Path)
    parser.add_argument("--bindings-jsonl", type=Path)
    parser.add_argument("--previous-anchors", type=Path)
    parser.add_argument("--previous-database-sha256")
    parser.add_argument("--previous-anchors-sha256")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-stamp-delta-s", type=float, default=0.10)
    parser.add_argument("--source-translation-consensus-m", type=float, default=0.03)
    parser.add_argument("--source-rotation-consensus-deg", type=float, default=5.0)
    args = parser.parse_args(arguments)
    if args.check_only:
        require(args.database is None and args.gate_csv is None and
                args.bindings_jsonl is None and args.previous_anchors is None and
                args.previous_database_sha256 is None and
                args.previous_anchors_sha256 is None and args.output is None,
                "--check-only cannot open inputs")
        identity = matrix_from_xyzw((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        require(translation(identity) == [0.0, 0.0, 0.0], "transform self-check failed")
        print("APRILTAG_ANCHOR_BUILDER_CHECK_OK no_database_opened=true")
        return
    require(args.database is not None and args.output is not None,
            "--database and --output are required")
    previous_values = (args.previous_anchors,
                       args.previous_database_sha256,
                       args.previous_anchors_sha256)
    previous_mode = all(value is not None for value in previous_values)
    require(previous_mode or all(value is None for value in previous_values),
            "--previous-anchors and both previous SHA256 values are all-or-none")
    if previous_mode:
        require(args.gate_csv is None,
                "--previous-anchors cannot be combined with --gate-csv")
        require_sha256(args.previous_database_sha256,
                       "--previous-database-sha256")
        require_sha256(args.previous_anchors_sha256,
                       "--previous-anchors-sha256")
    else:
        require((args.gate_csv is None) != (args.bindings_jsonl is None),
                "select exactly one of --gate-csv or --bindings-jsonl")
    for name in ("max_stamp_delta_s", "source_translation_consensus_m",
                 "source_rotation_consensus_deg"):
        value = getattr(args, name)
        require(math.isfinite(value) and value > 0, f"invalid {name}")
    input_paths = [args.database.resolve()]
    input_paths.extend(path.resolve() for path in
                       (args.bindings_jsonl, args.gate_csv,
                        args.previous_anchors) if path is not None)
    require(args.output.resolve() not in set(input_paths),
            "output cannot replace an input")
    result = build(args.database, args.gate_csv, args.max_stamp_delta_s,
                   args.source_translation_consensus_m,
                   args.source_rotation_consensus_deg,
                   bindings_jsonl=args.bindings_jsonl,
                   previous_anchors=args.previous_anchors,
                   previous_database_sha256=args.previous_database_sha256,
                   previous_anchors_sha256=args.previous_anchors_sha256)
    require(result["tags"],
            "no tag has three consistent, distinct saved-node view exemplars")
    atomic_write(args.output, result)
    print(json.dumps({
        "anchor_file": str(args.output.resolve()),
        "eligible_tag_ids": sorted(int(tag_id) for tag_id in result["tags"]),
        "source_database_sha256": result["source_database_sha256"],
        "lineage": result["lineage"],
        "source_database_mutated": False,
    }, sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except (AnchorBuildError, OSError, sqlite3.Error, ValueError, TypeError,
            KeyError, struct.error, zlib.error) as error:
        print(f"APRILTAG_ANCHOR_BUILD_REJECTED: {error}", file=os.sys.stderr)
        raise SystemExit(1)
