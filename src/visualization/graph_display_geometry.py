"""Stateless graph-to-display geometry; standard library only, no ROS or I/O.

RTAB-Map poses are already expressed in the graph header frame. Do not apply
map_to_odom again. Only positive-node neighbor links (0/6) are trajectory edges;
sorted IDs, loop links, proximity links, and landmark links are not a route.
"""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from numbers import Real


NEIGHBOR_LINK_TYPES = frozenset((0, 6))
TRAJECTORY_NAMESPACE = "trajectory"
TAG_LABEL_NAMESPACE = "historical_tag_labels"


class GraphValidationError(ValueError):
    """The whole snapshot must be cleared instead of drawing ambiguous data."""


@dataclass(frozen=True)
class Segment:
    from_id: int
    to_id: int
    start: tuple
    end: tuple


@dataclass(frozen=True)
class HistoricalTag:
    graph_id: int
    printed_id: int
    position: tuple


@dataclass(frozen=True)
class GraphGeometry:
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    robot_node_count: int
    segments: tuple
    historical_tags: tuple
    ignored_links: int
    missing_endpoint_links: int
    duplicate_neighbor_links: int


@dataclass(frozen=True)
class MarkerSpec:
    kind: str
    namespace: str = ""
    marker_id: int = 0
    frame_id: str = "map"
    stamp_sec: int = 0
    stamp_nanosec: int = 0
    points: tuple = ()
    position: tuple = (0.0, 0.0, 0.0)
    scale: tuple = (0.0, 0.0, 0.0)
    color: tuple = (0.0, 0.0, 0.0, 0.0)
    text: str = ""


def _mapping(value, name):
    if not isinstance(value, Mapping):
        raise GraphValidationError(name + " must be an object")
    return value


def _sequence(value, name):
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise GraphValidationError(name + " must be an array")
    return value


def _int32(value, name):
    if type(value) is not int or not -(2**31) <= value < 2**31:
        raise GraphValidationError(name + " must be an int32")
    return value


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise GraphValidationError(name + " must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise GraphValidationError(name + " must be a finite number")
    return result


def _point(value):
    value = _mapping(value, "pose")
    p = _mapping(value.get("position"), "position")
    q = _mapping(value.get("orientation"), "orientation")
    point = tuple(_number(p.get(k), "position."+k) for k in ("x", "y", "z"))
    quaternion = tuple(_number(q.get(k), "orientation."+k) for k in ("x", "y", "z", "w"))
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or abs(norm-1.0) > 0.01:
        raise GraphValidationError("pose quaternion must be approximately unit length")
    return point


def build_graph_geometry(snapshot):
    """Build one complete snapshot, never filling missing edges from history.

    Missing link endpoints are normal in some RTAB-Map graph snapshots and are
    skipped. Duplicate pose IDs and invalid poses reject the entire snapshot.
    Extra graph fields, including map_to_odom and link transforms, are not used.
    """
    snapshot = _mapping(snapshot, "graph")
    header = _mapping(snapshot.get("header"), "header")
    frame = header.get("frame_id")
    if not isinstance(frame, str) or not frame or frame != frame.strip():
        raise GraphValidationError("graph header.frame_id must be nonempty")
    stamp = _mapping(header.get("stamp"), "header.stamp")
    sec = _int32(stamp.get("sec"), "stamp.sec")
    nano = stamp.get("nanosec")
    if type(nano) is not int or not 0 <= nano < 1_000_000_000:
        raise GraphValidationError("stamp.nanosec must be in [0, 1000000000)")
    ids = _sequence(snapshot.get("poses_id"), "poses_id")
    poses = _sequence(snapshot.get("poses"), "poses")
    links = _sequence(snapshot.get("links"), "links")
    if len(ids) != len(poses):
        raise GraphValidationError("poses_id and poses must have equal lengths")
    positions = {}
    for raw_id, value in zip(ids, poses):
        node_id = _int32(raw_id, "pose ID")
        if node_id in positions:
            raise GraphValidationError("duplicate pose ID: " + str(node_id))
        positions[node_id] = _point(value)

    pairs = set()
    ignored, missing, duplicates = 0, 0, 0
    for raw in links:
        edge = _mapping(raw, "link")
        a = _int32(edge.get("from_id"), "link.from_id")
        b = _int32(edge.get("to_id"), "link.to_id")
        kind = _int32(edge.get("type"), "link.type")
        if kind not in NEIGHBOR_LINK_TYPES or a <= 0 or b <= 0 or a == b:
            ignored += 1
            continue
        if a not in positions or b not in positions:
            missing += 1
            continue
        pair = (min(a, b), max(a, b))
        if pair in pairs:
            duplicates += 1
        pairs.add(pair)

    segments = tuple(Segment(a, b, positions[a], positions[b]) for a, b in sorted(pairs))
    tags = tuple(HistoricalTag(node_id, -node_id-1, positions[node_id])
                 for node_id in sorted((i for i in positions if i < 0), reverse=True))
    return GraphGeometry(frame, sec, nano, sum(i > 0 for i in positions),
                         segments, tags, ignored, missing, duplicates)


def marker_specs(geometry):
    """DELETEALL plus full replacement, suitable for one latched MarkerArray.

    LINE_LIST is intentional: separate graph edges cannot be joined into a
    false connecting segment. Historical tags have labels, not robot axes.
    """
    common = dict(frame_id=geometry.frame_id, stamp_sec=geometry.stamp_sec,
                  stamp_nanosec=geometry.stamp_nanosec)
    result = [MarkerSpec("clear", **common)]
    if geometry.segments:
        result.append(MarkerSpec("line_list", namespace=TRAJECTORY_NAMESPACE,
            marker_id=0, points=tuple(p for s in geometry.segments for p in (s.start, s.end)),
            scale=(0.02, 0.0, 0.0), color=(0.05, 0.9, 0.2, 0.95), **common))
    for tag in geometry.historical_tags:
        x, y, z = tag.position
        result.append(MarkerSpec("text", namespace=TAG_LABEL_NAMESPACE,
            marker_id=tag.printed_id, position=(x, y, z+0.05),
            scale=(0.0, 0.0, 0.04), color=(0.4, 0.75, 1.0, 1.0),
            text=f"Tag {tag.printed_id}\nmap history (not live)", **common))
    return tuple(result)


def clear_marker_specs():
    """On an invalid snapshot, remove our previous display in RViz's map frame."""
    return (MarkerSpec("clear"),)
