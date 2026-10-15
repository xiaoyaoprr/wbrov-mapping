"""Pure display contract for independently framed submaps and Tag history.

Coordinates are never transformed here. Registration and localization are
producer decisions; drawing a Tag is never evidence of relocalization.
"""
from dataclasses import dataclass
import math
import re

import graph_display_geometry as graph


REGISTRY_SCHEMA = "wbrov.submaps.v1"
TAG_SCHEMA = "wbrov.tag-visualization.v1"
MAX_SUBMAPS = 4096
PALETTE = ((1.0, .58, .12, 1.0), (.65, .42, 1.0, 1.0),
           (.1, .8, .95, 1.0), (1.0, .38, .62, 1.0))


class DisplayValidationError(ValueError):
    pass


def integer(value, name, minimum=0):
    if type(value) is not int or not minimum <= value < 2**63:
        raise DisplayValidationError(name + " must be a bounded integer")
    return value


def text(value, name, maximum=160):
    if not isinstance(value, str) or not value or len(value) > maximum or any(ord(c) < 32 for c in value):
        raise DisplayValidationError(name + " must be bounded text")
    return value


def point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise DisplayValidationError("point must have three coordinates")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise DisplayValidationError("point must contain finite coordinates")
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class Submap:
    key: str
    map_id: int
    reset_epoch: int
    component_id: int
    frame_id: str
    graph_topic: str
    cloud_topic: str
    display_topic: str
    active: bool
    status: str
    registered: bool
    nodes: tuple
    cloud_truncated: bool


@dataclass(frozen=True)
class Registry:
    session_short_hash: str
    session_token_sha256: str
    revision: int
    submaps: tuple


def parse_registry(row):
    if not isinstance(row, dict) or row.get("schema") != REGISTRY_SCHEMA:
        raise DisplayValidationError("unknown submap registry schema")
    session = row.get("session_short_hash")
    if not isinstance(session, str) or not re.fullmatch(r"[0-9a-f]{16}", session):
        raise DisplayValidationError("invalid session short hash")
    full_session = row.get("session_token_sha256")
    if not isinstance(full_session, str) or not re.fullmatch(r"[0-9a-f]{64}", full_session) or not full_session.startswith(session):
        raise DisplayValidationError("session hash binding differs")
    revision = integer(row.get("revision"), "revision")
    entries = row.get("submaps")
    if not isinstance(entries, list):
        raise DisplayValidationError("submaps must be a list")
    if len(entries) > MAX_SUBMAPS:
        raise DisplayValidationError("子图显示容量超出 4096，当前显示不受支持；原始数据仍由建图端保留")
    if row.get("main_frame_id") != "map":
        raise DisplayValidationError("main frame contract differs")
    parsed, seen, global_node_ids = [], set(), set()
    for item in entries:
        if not isinstance(item, dict):
            raise DisplayValidationError("submap must be an object")
        map_id = integer(item.get("map_id"), "map_id")
        epoch = integer(item.get("reset_epoch"), "reset_epoch")
        component = integer(item.get("component_id"), "component_id", 1)
        key = f"m{map_id}_e{epoch}_c{component}"
        if item.get("key") != key or key in seen:
            raise DisplayValidationError("submap key identity mismatch or duplicate")
        registered = item.get("registered")
        if type(registered) is not bool or type(item.get("active")) is not bool:
            raise DisplayValidationError("submap registration/activity must be explicit")
        frame = text(item.get("frame_id"), "frame_id")
        expected_frame = "map" if registered else f"wbrov_submap_{session}_{key}"
        if frame != expected_frame:
            raise DisplayValidationError("submap frame identity mismatch")
        status = item.get("status")
        if status not in (("registered_main",) if registered else ("local_tracking", "local_paused")):
            raise DisplayValidationError("submap status and registration disagree")
        topics = []
        for key, suffix in (("graph_topic", "graph"), ("cloud_topic", "cloud"), ("display_topic", "display")):
            topic = item.get(key)
            if topic != f"/wbrov/submaps/{item['key']}/{suffix}":
                raise DisplayValidationError("submap topic identity mismatch")
            topics.append(topic)
        nodes, ids = [], set()
        if not isinstance(item.get("nodes"), list) or len(item["nodes"]) > 100000:
            raise DisplayValidationError("submap nodes must be a bounded list")
        for node in item["nodes"]:
            node_id = integer(node.get("id"), "node id", 1)
            if node_id in ids or node_id in global_node_ids:
                raise DisplayValidationError("duplicate node within or across submaps")
            translation = point(node.get("translation"))
            q = node.get("quaternion_xyzw")
            if not isinstance(q, list) or len(q) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) for v in q) or abs(math.hypot(*q)-1) > .01:
                raise DisplayValidationError("invalid node quaternion")
            nodes.append((node_id, translation, tuple(q)))
            ids.add(node_id)
            global_node_ids.add(node_id)
        if type(item.get("cloud_truncated", False)) is not bool:
            raise DisplayValidationError("cloud truncation flag must be boolean")
        parsed.append(Submap(item["key"], map_id, epoch, component, frame, *topics, item["active"], status, registered, tuple(nodes), item.get("cloud_truncated", False)))
        seen.add(item["key"])
    return Registry(session, full_session, revision, tuple(parsed))


def continuity_matches(row, registry, epoch):
    return (isinstance(row, dict) and registry is not None and
            row.get("schema") == "wbrov.mapping-continuity.v1" and
            row.get("session_token_sha256") == registry.session_token_sha256 and
            type(row.get("epoch")) is int and row["epoch"] == epoch and
            type(row.get("local_tracking_healthy")) is bool and
            type(row.get("local_submap_write_allowed")) is bool)


def parse_tags(row, registry):
    """Return visible, placed history plus non-spatial diagnostic tag labels."""
    if not isinstance(row, dict) or row.get("schema") != TAG_SCHEMA:
        raise DisplayValidationError("unknown Tag display schema")
    if row.get("session_short_hash") != registry.session_short_hash or row.get("session_token_sha256") != registry.session_token_sha256 or row.get("registry_revision") != registry.revision:
        raise DisplayValidationError("Tag placements do not match the current registry revision")
    items = row.get("tags")
    if not isinstance(items, list) or len(items) > 4096:
        raise DisplayValidationError("Tag list is absent or too large")
    if row.get("visualization_valid", True) is not True:
        if items:
            raise DisplayValidationError("invalid Tag snapshot cannot retain placements")
        return (), ()
    by_key = {s.key: s for s in registry.submaps}
    placed, unplaced, seen = [], [], set()
    for tag in items:
        if not isinstance(tag, dict):
            raise DisplayValidationError("Tag must be an object")
        tag_id = integer(tag.get("tag_id"), "tag_id")
        family = text(tag.get("family"), "family", 64)
        status = tag.get("placement_status")
        if status == "unplaced":
            if tag.get("center") is not None or tag.get("corners3d") is not None:
                raise DisplayValidationError("unplaced Tag must not carry drawable coordinates")
            unplaced.append(f"{family}:{tag_id} ({tag.get('geometry_status', 'unplaced')})")
            continue
        map_id = integer(tag.get("map_id"), "map_id")
        epoch = integer(tag.get("reset_epoch"), "reset_epoch")
        submap_key = text(tag.get("submap_key"), "submap_key")
        key = (submap_key, map_id, epoch, family, tag_id)
        if key in seen:
            raise DisplayValidationError("duplicate Tag placement identity")
        seen.add(key)
        submap = by_key.get(submap_key)
        if submap is None or (submap.map_id, submap.reset_epoch, submap.frame_id) != (map_id, epoch, tag.get("frame_id")):
            raise DisplayValidationError("Tag frame/map/epoch/key is not in this registry")
        if status not in ("local_submap", "main_registered"):
            raise DisplayValidationError("unknown Tag placement status")
        if (status == "main_registered") != submap.registered:
            raise DisplayValidationError("Tag placement and registry registration disagree")
        if tag.get("relocalization_verified") is not False:
            raise DisplayValidationError("display Tag must not claim relocalization authorization")
        node_id = integer(tag.get("node_id"), "node_id", 1)
        if node_id not in {n[0] for n in submap.nodes}:
            raise DisplayValidationError("Tag source node is absent from registry")
        raw_stamp = tag.get("source_stamp_ns")
        if isinstance(raw_stamp, str) and re.fullmatch(r"[0-9]{1,19}", raw_stamp):
            raw_stamp = int(raw_stamp)
        stamp = integer(raw_stamp, "source_stamp_ns", 1)
        center = point(tag.get("center"))
        corners = tag.get("corners3d")
        if not isinstance(corners, list) or len(corners) != 4:
            raise DisplayValidationError("placed Tag requires four measured corners")
        corners = tuple(point(p) for p in corners)
        placed.append(dict(key=key, family=family, tag_id=tag_id, frame_id=tag["frame_id"],
                           status=status, center=center, corners=corners, source_stamp_ns=stamp))
    return tuple(placed), tuple(unplaced)


def tag_specs(tags, frame):
    specs = []
    for index, tag in enumerate(sorted((t for t in tags if t["frame_id"] == frame), key=lambda t: t["key"])):
        corners = tag["corners"]
        color = (.15, 1.0, .4, 1.0) if tag["status"] == "main_registered" else (1.0, .65, .15, 1.0)
        common = dict(frame_id=frame, stamp_sec=0, stamp_nanosec=0)
        specs.append(graph.MarkerSpec("line_list", namespace="tag_estimated_corners", marker_id=index,
            points=tuple(p for i in range(4) for p in (corners[i], corners[(i+1) % 4])),
            scale=(.008, 0.0, 0.0), color=color, **common))
        specs.append(graph.MarkerSpec("sphere", namespace="tag_estimated_center", marker_id=index,
            position=tag["center"], scale=(.025, .025, .025), color=color, **common))
        x, y, z = tag["center"]
        suffix = "主图历史观测" if tag["status"] == "main_registered" else "临时子图 / 未对齐"
        specs.append(graph.MarkerSpec("text", namespace="tag_placement_labels", marker_id=index,
            position=(x, y, z+.05), scale=(0.0, 0.0, .045), color=color,
            text=f"Tag {tag['tag_id']} [{tag['family']}] · 位置估计\n{suffix}；不是重定位通过凭据", **common))
    return tuple(specs)


def submap_specs(submap, snapshot, tags=(), registry_fresh=True, local_tracking_healthy=False, local_write_allowed=False):
    """One independently framed full replacement; never joins sorted IDs."""
    color = PALETTE[submap.map_id % len(PALETTE)]
    common = dict(frame_id=submap.frame_id)
    specs = [graph.MarkerSpec("clear", **common)]
    if submap.registered:
        return tuple(specs) # The existing main map display owns registered geometry.
    node_count = 0
    if snapshot is not None:
        built = graph.build_graph_geometry(snapshot)
        if built.frame_id != submap.frame_id:
            raise DisplayValidationError("graph frame differs from registry")
        actual = {node: tuple(p["position"][axis] for axis in ("x", "y", "z"))
                  for node, p in zip(snapshot["poses_id"], snapshot["poses"]) if node > 0}
        if set(actual) != {n[0] for n in submap.nodes} or any(
                max(abs(a-b) for a, b in zip(actual[node], position)) > 1e-6
                for node, position, _ in submap.nodes):
            raise DisplayValidationError("graph positions do not match the current registry snapshot")
        node_count = built.robot_node_count
        if built.segments:
            specs.append(graph.MarkerSpec("line_list", namespace="temporary_submap_edges", marker_id=0,
                points=tuple(p for segment in built.segments for p in (segment.start, segment.end)),
                scale=(.022, 0, 0), color=color, **common))
    specs.extend(tag_specs(tags, submap.frame_id))
    # A registry heartbeat only says its frozen geometry is available. Actual
    # current tracking/write permission comes from the independent live gate.
    if submap.active and local_tracking_healthy:
        mode = "当前局部建图" if local_write_allowed else "局部跟踪正常 / 等待恢复验证，写入暂停"
    else:
        mode = "局部跟踪暂停或状态未知 / 保留快照"
    stale = "\n状态未更新：仅显示保留快照" if not registry_fresh else ""
    capped = "\n显示点云达到容量上限（原始图数据保留）" if submap.cloud_truncated else ""
    specs.append(graph.MarkerSpec("text", namespace="temporary_submap_status", marker_id=0,
        position=(0, 0, .35), scale=(0, 0, .07), color=color,
        text=f"临时子图 {submap.map_id} / {mode}\n未对齐主图 · 独立坐标 · 稀疏显示 · {node_count} 个节点\n{submap.frame_id}{stale}{capped}", **common))
    return tuple(specs)


def main_tag_specs(tags, unplaced=()):
    specs = [graph.MarkerSpec("clear", frame_id="map")]
    specs.extend(tag_specs(tags, "map"))
    if unplaced:
        specs.append(graph.MarkerSpec("text", frame_id="map", namespace="unplaced_tag_status", marker_id=0,
            position=(0, 0, .65), scale=(0, 0, .05), color=(1.0, .7, .2, 1.0),
            text="已识别但无法标位置（以下文字不是Tag坐标）：\n"+"\n".join(unplaced[:8])))
    return tuple(specs)
