import copy
import json
from pathlib import Path
import tempfile
import unittest

import graph_display_geometry as graph
import submap_display_geometry as display
import submap_rviz_config as rviz


SESSION = "0123456789abcdef" * 4


def registry_fixture(registered=False, revision=1):
    key = "m2_e3_c40"
    return dict(schema=display.REGISTRY_SCHEMA, session_short_hash=SESSION[:16], session_token_sha256=SESSION,
                revision=revision, main_frame_id="map", submaps=[dict(key=key, map_id=2, reset_epoch=3, component_id=40,
                frame_id="map" if registered else f"wbrov_submap_{SESSION[:16]}_{key}", registered=registered,
                active=True, status="registered_main" if registered else "local_tracking",
                graph_topic=f"/wbrov/submaps/{key}/graph", cloud_topic=f"/wbrov/submaps/{key}/cloud",
                display_topic=f"/wbrov/submaps/{key}/display", nodes=[
                    dict(id=40, source_stamp_ns="1000000000", translation=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1]),
                    dict(id=42, source_stamp_ns="1100000000", translation=[.1, 0, 0], quaternion_xyzw=[0, 0, 0, 1])])])


def graph_fixture(registry):
    entry = registry.submaps[0]
    return dict(header=dict(frame_id=entry.frame_id, stamp=dict(sec=1, nanosec=0)), poses_id=[40, 42],
                poses=[dict(position=dict(zip(("x", "y", "z"), p)), orientation=dict(x=0, y=0, z=0, w=1))
                       for p in ((0, 0, 0), (.1, 0, 0))],
                links=[dict(from_id=40, to_id=42, type=0)])


def tag_fixture(registry):
    entry = registry.submaps[0]
    return dict(schema=display.TAG_SCHEMA, session_short_hash=SESSION[:16], session_token_sha256=SESSION,
                registry_revision=registry.revision, tags=[dict(submap_key=entry.key, component_id=entry.component_id,
                map_id=entry.map_id, reset_epoch=entry.reset_epoch, family="tag36h11", tag_id=7,
                placement_status="main_registered" if entry.registered else "local_submap", frame_id=entry.frame_id,
                center=[.2, 0, 1], corners3d=[[.1, -.1, 1], [.3, -.1, 1], [.3, .1, 1], [.1, .1, 1]],
                node_id=40, source_stamp_ns="1000000000", geometry_status="measured", relocalization_verified=False)])


class DisplayTests(unittest.TestCase):
    def setUp(self):
        self.registry = display.parse_registry(registry_fixture())

    def test_local_frame_never_becomes_main(self):
        tags, unplaced = display.parse_tags(tag_fixture(self.registry), self.registry)
        specs = display.submap_specs(self.registry.submaps[0], graph_fixture(self.registry), tags)
        self.assertTrue(all(s.frame_id == self.registry.submaps[0].frame_id for s in specs))
        self.assertTrue(any("未对齐" in s.text for s in specs))
        self.assertEqual(len(display.main_tag_specs(tags)), 1) # only DELETEALL
        self.assertFalse(unplaced)

    def test_cannot_claim_registered_by_renaming_frame(self):
        row = registry_fixture()
        row["submaps"][0]["frame_id"] = "map"
        with self.assertRaises(display.DisplayValidationError):
            display.parse_registry(row)
        row = tag_fixture(self.registry)
        row["tags"][0].update(placement_status="main_registered", frame_id="map")
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(row, self.registry)

    def test_registration_clears_local_view_and_uses_true_main_values(self):
        registered = display.parse_registry(registry_fixture(True, 2))
        row = tag_fixture(registered)
        row["tags"][0]["center"] = [5, 6, 7]
        tags, _ = display.parse_tags(row, registered)
        local = display.submap_specs(registered.submaps[0], None, tags)
        self.assertEqual([s.kind for s in local], ["clear"])
        center = next(s for s in display.main_tag_specs(tags) if s.kind == "sphere")
        self.assertEqual(center.frame_id, "map")
        self.assertEqual(center.position, (5., 6., 7.))

    def test_graph_revision_pose_mismatch_rejected(self):
        snapshot = graph_fixture(self.registry)
        snapshot["poses"][1]["position"]["x"] = 8
        with self.assertRaises(display.DisplayValidationError):
            display.submap_specs(self.registry.submaps[0], snapshot)
        row = tag_fixture(self.registry)
        row["registry_revision"] += 1
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(row, self.registry)

    def test_no_invented_edge_across_disconnected_nodes(self):
        snapshot = graph_fixture(self.registry)
        snapshot["links"] = []
        specs = display.submap_specs(self.registry.submaps[0], snapshot)
        self.assertFalse(any(s.kind == "line_list" for s in specs))

    def test_unplaced_has_no_spatial_tag_marker(self):
        row = tag_fixture(self.registry)
        row["tags"] = [dict(tag_id=8, family="tag36h11", placement_status="unplaced", submap_key=None,
                            geometry_status="ambiguous_pose", center=None, corners3d=None)]
        tags, unknown = display.parse_tags(row, self.registry)
        self.assertFalse(tags)
        specs = display.main_tag_specs(tags, unknown)
        self.assertTrue(all(s.kind in ("clear", "text") for s in specs))
        self.assertIn("不是Tag坐标", specs[-1].text)
        row["tags"][0]["center"] = [0, 0, 0]
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(row, self.registry)

    def test_same_tag_id_in_different_components_is_kept_separate(self):
        row = registry_fixture()
        other = copy.deepcopy(row["submaps"][0])
        other.update(key="m2_e3_c90", component_id=90, frame_id=f"wbrov_submap_{SESSION[:16]}_m2_e3_c90", active=False,
                     graph_topic="/wbrov/submaps/m2_e3_c90/graph", cloud_topic="/wbrov/submaps/m2_e3_c90/cloud",
                     display_topic="/wbrov/submaps/m2_e3_c90/display", nodes=[dict(id=90, translation=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1])])
        row["submaps"].append(other)
        reg = display.parse_registry(row)
        t = tag_fixture(reg)
        second = copy.deepcopy(t["tags"][0])
        second.update(submap_key=other["key"], component_id=90, frame_id=other["frame_id"], node_id=90)
        t["tags"].append(second)
        tags, _ = display.parse_tags(t, reg)
        self.assertEqual(len(tags), 2)
        self.assertNotEqual(tags[0]["frame_id"], tags[1]["frame_id"])
        t["tags"].append(second)
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(t, reg)

    def test_missing_source_node_and_nonfinite_values_rejected(self):
        row = tag_fixture(self.registry)
        row["tags"][0]["node_id"] = 999
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(row, self.registry)
        row = tag_fixture(self.registry)
        row["tags"][0]["center"][0] = float("nan")
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(row, self.registry)

    def test_full_snapshots_delete_removed_tags(self):
        tags, _ = display.parse_tags(tag_fixture(self.registry), self.registry)
        self.assertGreater(len(display.tag_specs(tags, self.registry.submaps[0].frame_id)), 0)
        empty = display.submap_specs(self.registry.submaps[0], graph_fixture(self.registry), ())
        self.assertEqual(empty[0].kind, "clear")
        self.assertFalse(any(s.namespace.startswith("tag_") for s in empty))
        invalid = tag_fixture(self.registry)
        invalid["visualization_valid"] = False
        with self.assertRaises(display.DisplayValidationError):
            display.parse_tags(invalid, self.registry)
        invalid["tags"] = []
        self.assertEqual(display.parse_tags(invalid, self.registry), ((), ()))

    def test_independent_rviz_and_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = rviz.write_configurations(Path(folder).resolve(), self.registry)
            config = Path(manifest["active"]["path"]).read_text(encoding="utf-8")
            self.assertIn(self.registry.submaps[0].frame_id, config)
            self.assertNotIn("/cloud_map", config)
            self.assertNotIn("SetGoal", config)
            self.assertNotIn("InitialPose", config)
            self.assertEqual(json.loads((Path(folder)/"submap_views.json").read_text())["tf_published"], False)
            registered = display.parse_registry(registry_fixture(True, 2))
            self.assertIsNone(rviz.write_configurations(Path(folder).resolve(), registered)["active"])

    def test_full_session_continuity_binding(self):
        row = dict(schema="wbrov.mapping-continuity.v1", session_token_sha256=SESSION, epoch=3,
                   local_tracking_healthy=True, local_submap_write_allowed=True)
        self.assertTrue(display.continuity_matches(row, self.registry, 3))
        self.assertFalse(display.continuity_matches(row, self.registry, 4))
        row["session_token_sha256"] = SESSION[:16] + "f" * 48
        self.assertFalse(display.continuity_matches(row, self.registry, 3))
        row.pop("session_token_sha256")
        self.assertFalse(display.continuity_matches(row, self.registry, 3))

    def test_explicit_submap_capacity(self):
        row = registry_fixture()
        prototype = row["submaps"][0]
        row["submaps"] = []
        for component in range(1, 130):
            entry = copy.deepcopy(prototype)
            key = f"m2_e3_c{component}"
            entry.update(key=key, component_id=component, nodes=[], frame_id=f"wbrov_submap_{SESSION[:16]}_{key}")
            for field, suffix in (("graph_topic", "graph"), ("cloud_topic", "cloud"), ("display_topic", "display")):
                entry[field] = f"/wbrov/submaps/{key}/{suffix}"
            row["submaps"].append(entry)
        self.assertEqual(len(display.parse_registry(row).submaps), 129)
        row["submaps"] = [prototype] * 4097
        with self.assertRaisesRegex(display.DisplayValidationError, "显示容量超出 4096"):
            display.parse_registry(row)

    def test_registry_heartbeat_is_not_live_tracking(self):
        entry = self.registry.submaps[0]
        snapshot = graph_fixture(self.registry)
        frozen = display.submap_specs(entry, snapshot, registry_fresh=True)
        self.assertIn("局部跟踪暂停或状态未知", frozen[-1].text)
        observing = display.submap_specs(entry, snapshot, registry_fresh=True, local_tracking_healthy=True, local_write_allowed=False)
        self.assertIn("写入暂停", observing[-1].text)
        active = display.submap_specs(entry, snapshot, registry_fresh=True, local_tracking_healthy=True, local_write_allowed=True)
        self.assertIn("当前局部建图", active[-1].text)
        row = registry_fixture()
        row["submaps"][0]["cloud_truncated"] = True
        truncated = display.parse_registry(row).submaps[0]
        self.assertIn("显示点云达到容量上限", display.submap_specs(truncated, snapshot)[-1].text)


if __name__ == "__main__":
    unittest.main()
