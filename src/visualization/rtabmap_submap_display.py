#!/usr/bin/env python3
"""Passive registry/graph/Tag display. No TF, SLAM service or control writes."""
import sys
sys.dont_write_bytecode = True

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rtabmap_msgs.msg import MapGraph
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

import graph_display_geometry as graph_geometry
import submap_display_geometry as geometry
import submap_rviz_config as rviz
from rtabmap_graph_display import graph_snapshot, marker_array


REGISTRY_TOPIC = "/wbrov/submaps/registry"
TAG_TOPIC = "/wbrov/tag_visualization/state"
MAIN_TAG_TOPIC = "/wbrov/tags/main_markers"


class SubmapDisplayNode(Node):
    def __init__(self, rviz_directory=None):
        super().__init__("rtabmap_submap_display", start_parameter_services=False)
        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.registry = None
        self.registry_error = ""
        self.last_registry = None
        self.registry_arrival = 0.0
        self.retired_sessions = set()
        self.entries = {}
        self.graph_subs = {}
        self.submap_publishers = {}
        self.graphs = {}
        self.dirty_entries = set()
        self.last_render = {}
        self.main_dirty = False
        self.tags = ()
        self.unplaced = ()
        self.tag_row = None
        self.future_tag_row = None
        self.continuity_status = None
        self.continuity_arrival = 0.0
        self.rviz_directory = rviz_directory
        self.main_pub = self.create_publisher(MarkerArray, MAIN_TAG_TOPIC, self.qos)
        self.registry_sub = self.create_subscription(String, REGISTRY_TOPIC, self.on_registry, self.qos)
        self.tag_sub = self.create_subscription(String, TAG_TOPIC, self.on_tags, self.qos)
        self.continuity_sub = self.create_subscription(String, "/wbrov/mapping_continuity/status", self.on_continuity_status, self.qos)
        # Status can arrive at odometry rate. Coalesce display work so a history
        # of frozen submaps cannot multiply each status into dozens of messages.
        self.timer = self.create_timer(.2, self.render_pending)
        self.get_logger().info("Display only: independent submap frames; no TF broadcast, no registration or control output")

    def clear_entry(self, key):
        if key in self.submap_publishers:
            frame = self.entries[key].frame_id
            self.submap_publishers[key].publish(marker_array((graph_geometry.MarkerSpec("clear", frame_id=frame),)))

    def clear_all(self):
        for key in list(self.entries):
            self.clear_entry(key)
            self.destroy_subscription(self.graph_subs.pop(key))
            self.destroy_publisher(self.submap_publishers.pop(key))
        self.entries.clear()
        self.graphs.clear()
        self.dirty_entries.clear()
        self.last_render.clear()
        self.tags = self.unplaced = ()
        self.tag_row = None
        self.continuity_status = None
        self.continuity_arrival = 0.0
        self.main_pub.publish(marker_array(geometry.main_tag_specs(())))

    def on_registry(self, message):
        try:
            if len(message.data) > 16 * 1024 * 1024:
                raise geometry.DisplayValidationError("registry display input exceeds 16MiB")
            incoming = geometry.parse_registry(json.loads(message.data))
            if incoming.session_token_sha256 in self.retired_sessions:
                return
            previous = self.last_registry
            if previous and incoming.session_token_sha256 == previous.session_token_sha256:
                if incoming.revision < previous.revision:
                    return
                if incoming.revision == previous.revision:
                    if incoming != previous:
                        raise geometry.DisplayValidationError("registry changed without a new revision")
                    if self.registry is not None:
                        self.registry_arrival = time.monotonic()
                        return
            if previous and incoming.session_token_sha256 != previous.session_token_sha256:
                self.retired_sessions.add(previous.session_token_sha256)
                self.clear_all()
            # A new graph revision invalidates every old Tag placement until a
            # producer recomputes it against this same revision's node poses.
            self.tags = self.unplaced = ()
            self.tag_row = None
            self.main_pub.publish(marker_array(geometry.main_tag_specs(())))
            new_entries = {entry.key: entry for entry in incoming.submaps}
            for key in list(self.entries):
                if key not in new_entries:
                    self.clear_entry(key)
                    self.destroy_subscription(self.graph_subs.pop(key))
                    self.destroy_publisher(self.submap_publishers.pop(key))
                    self.entries.pop(key)
                    self.graphs.pop(key, None)
                    self.dirty_entries.discard(key)
                    self.last_render.pop(key, None)
                elif self.entries[key].frame_id != new_entries[key].frame_id:
                    self.clear_entry(key)
                    self.graphs.pop(key, None)
            self.registry = incoming
            self.registry_error = ""
            self.last_registry = incoming
            self.registry_arrival = time.monotonic()
            for key, entry in new_entries.items():
                if key not in self.entries:
                    self.submap_publishers[key] = self.create_publisher(MarkerArray, entry.display_topic, self.qos)
                    def callback(message, bound_key=key):
                        self.on_graph(bound_key, message)
                    self.graph_subs[key] = self.create_subscription(MapGraph, entry.graph_topic, callback, self.qos)
                self.entries[key] = entry
            if self.rviz_directory:
                rviz.write_configurations(self.rviz_directory, incoming)
            if self.future_tag_row is not None and self.future_tag_row.get("session_token_sha256") == incoming.session_token_sha256 and self.future_tag_row.get("registry_revision") == incoming.revision:
                try:
                    self.tags, self.unplaced = geometry.parse_tags(self.future_tag_row, incoming)
                    self.tag_row = self.future_tag_row
                except (ValueError, TypeError, AttributeError):
                    self.tags = self.unplaced = ()
                self.future_tag_row = None
            self.render_all()
        except (ValueError, TypeError, AttributeError, OverflowError, OSError) as error:
            self.get_logger().warning("Submap registry display rejected: " + str(error))
            # Clear placements rather than showing coordinates from unknown
            # authority. A later valid complete registry can repopulate them.
            self.clear_all()
            self.registry = None
            self.registry_error = "瀛愬浘鏄剧ず宸叉竻绌猴細" + str(error)
            self.render_all()

    def on_graph(self, key, message):
        if key not in self.entries:
            return
        try:
            snapshot = graph_snapshot(message)
            self.graphs[key] = snapshot
            self.dirty_entries.add(key)
        except (ValueError, TypeError, AttributeError, OverflowError):
            self.graphs.pop(key, None)
            self.dirty_entries.add(key)

    def on_tags(self, message):
        try:
            if len(message.data) > 4 * 1024 * 1024:
                raise geometry.DisplayValidationError("Tag display input exceeds 4MiB")
            row = json.loads(message.data)
            if not isinstance(row, dict):
                raise geometry.DisplayValidationError("Tag snapshot must be an object")
            if self.registry is None:
                self.future_tag_row = row
                return
            # Ignore delayed previous revisions without deleting current data.
            if row.get("session_token_sha256") != self.registry.session_token_sha256 or row.get("registry_revision") != self.registry.revision:
                if (row.get("session_token_sha256") not in self.retired_sessions and
                        type(row.get("registry_revision")) is int and
                        (row.get("session_token_sha256") != self.registry.session_token_sha256 or row["registry_revision"] > self.registry.revision)):
                    self.future_tag_row = row
                return
            self.tags, self.unplaced = geometry.parse_tags(row, self.registry)
            self.tag_row = row
        except (ValueError, TypeError, AttributeError, OverflowError) as error:
            self.tags = self.unplaced = ()
            self.tag_row = None
            self.get_logger().warning("Tag placements cleared: " + str(error))
        self.render_all()

    def on_continuity_status(self, message):
        old = self.continuity_status
        try:
            row = json.loads(message.data)
            if row.get("schema") != "wbrov.mapping-continuity.v1" or type(row.get("epoch")) is not int or type(row.get("local_tracking_healthy")) is not bool or type(row.get("local_submap_write_allowed")) is not bool:
                raise ValueError("invalid live local continuity status")
            self.continuity_status = row
            self.continuity_arrival = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.continuity_status = None
        fields = ('session_token_sha256', 'epoch', 'local_tracking_healthy', 'local_submap_write_allowed')
        current = self.continuity_status
        if any((old or {}).get(k) != (current or {}).get(k) for k in fields):
            self.dirty_entries.update(self.entries)

    def render_entry(self, key):
        entry = self.entries[key]
        fresh = 0 <= time.monotonic() - self.registry_arrival <= 2.0
        status = self.continuity_status
        live = geometry.continuity_matches(status, self.registry, entry.reset_epoch) and 0 <= time.monotonic() - self.continuity_arrival <= 1.0
        healthy = live and status.get("local_tracking_healthy") is True
        write_allowed = live and status.get("local_submap_write_allowed") is True
        try:
            specs = geometry.submap_specs(entry, self.graphs.get(key), self.tags, fresh, healthy, write_allowed)
        except (ValueError, TypeError, AttributeError, OverflowError):
            specs = geometry.submap_specs(entry, None, self.tags, fresh, healthy, write_allowed)
        self.submap_publishers[key].publish(marker_array(specs))
        self.last_render[key] = time.monotonic()

    def render_all(self):
        self.dirty_entries.update(self.entries)
        self.main_dirty = True

    def render_pending(self):
        now = time.monotonic()
        due = self.dirty_entries | {key for key in self.entries if now - self.last_render.get(key, 0) >= 1.0}
        self.dirty_entries.clear()
        for key in sorted(due & self.entries.keys()):
            self.render_entry(key)
        if not self.main_dirty:
            return
        self.main_dirty = False
        specs = list(geometry.main_tag_specs(self.tags, self.unplaced))
        if self.registry_error:
            specs.append(graph_geometry.MarkerSpec("text", frame_id="map", namespace="submap_display_error", marker_id=0,
                position=(0, 0, .8), scale=(0, 0, .05), color=(1.0, .35, .1, 1.0), text=self.registry_error))
        self.main_pub.publish(marker_array(specs))


def check_only():
    for module in (Path(__file__), Path(geometry.__file__), Path(rviz.__file__)):
        compile(module.read_bytes(), str(module), "exec", dont_inherit=True)
    specs = [graph_geometry.MarkerSpec("clear"),
             graph_geometry.MarkerSpec("sphere", position=(0, 1, 2), color=(1, 1, 1, 1), scale=(.02, .02, .02)),
             graph_geometry.MarkerSpec("line_list", points=((0, 1, 2), (1, 2, 3)), color=(1, 0, 0, 1), scale=(.02, 0, 0)),
             graph_geometry.MarkerSpec("text", position=(0, 0, 1), color=(1, 1, 1, 1), scale=(0, 0, .05), text="Tag 7")]
    result = marker_array(specs)
    assert len(result.markers) == 4
    assert result.markers[1].pose.position.y == 1.0 and result.markers[2].points[1].z == 3.0
    print(json.dumps(dict(status="PASS", mode="check-only", node_created=False, rclpy_initialized=False,
                         hardware_started=False, tf_published=False, registry_topic=REGISTRY_TOPIC,
                         tag_topic=TAG_TOPIC, main_tag_topic=MAIN_TAG_TOPIC)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--rviz-output-dir", type=Path)
    options, ros_args = parser.parse_known_args()
    if options.check_only:
        if options.rviz_output_dir or ros_args:
            parser.error("--check-only accepts no runtime options")
        check_only()
        return
    if options.rviz_output_dir and (not options.rviz_output_dir.is_absolute() or not options.rviz_output_dir.is_dir()):
        parser.error("--rviz-output-dir must be an existing absolute directory")
    rclpy.init(args=ros_args)
    node = None
    try:
        node = SubmapDisplayNode(options.rviz_output_dir)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
