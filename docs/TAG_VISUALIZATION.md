# Receipt-bound Tag visualization

This overlay adds display evidence to the existing authenticated Tag consumer. It uses that consumer's actual accepted RGB image, CameraInfo, verified AprilTag 36h11 corners, and accepted-image receipt. It does not start a camera, publish a pose/TF, authorize localization, or change Tag recovery proposals.

## Deployment

The implementation is in `runtime/libexec/wbrov_corner_recovery/`. Its principal display files are:

- `consumer.py`: exact inherited consumer plus display hook.
- `tag_visualization.py`: ROS adapter inside the existing owner node.
- `tag_visualization_geometry.py`: bounded evidence cache, IPPE and coordinate placement.

The matching `live_bridge.py` and `bridge_status.py` are also required for nonterminal `observing` ACK behavior. The repository includes their matching dependencies in the same runtime directory. See `provenance/submap-tag-20261015/source-map.json` for the complete native runtime Python pins; copying only the three display files is insufficient.

The inherited consumer is from `tag_retention_20260923/tag/consumer.py`, SHA-256 `3af0e7299087e3aba0a5fa53c154f401d9bc77f66affdd732f22d68c9a701f3b`. The candidate consumer SHA-256 is `6b7b4b44526c3e9b472b0772531ec4292725ecbe83e7c30d1c27354d8dcee36f`. Source-package preparation checked the historical pins; this repository preserves the resulting implementation. Updating these sources does not switch the robot's production launcher.

## Input and identity

`/wbrov/submaps/registry` is a reliable, transient-local `std_msgs/String`, schema `wbrov.submaps.v1`. The publisher endpoint must match the configured Core state node and namespace; it is separately authenticated for the registry topic. The full session hash and its short prefix must match the consumer configuration. Revisions must advance, or repeat the exact same payload as a heartbeat. A changed payload at the same revision, an older revision, invalid graph geometry, a missing/replaced endpoint, or a registry age above 3 seconds clears the display snapshot. A registry heartbeat only means the frozen graph is available; it does not prove current tracking health.

Display parsing accepts up to 4096 components, 20000 total nodes and an 8 MiB registry message. These are bounded display/parser capacities; they do not limit original graph writes. Evidence is cached on the actual consumer's `bound` event independently of registry arrival. A later registry, or a later revision that supplies a formerly missing exact node, can place the retained observation. A registry arriving before a binding is also supported; the next display timer processes the binding against that snapshot.

Each stored observation is checked with the real `core_pixel_join.compare_accepted_image`, including the image SHA, dimensions, stamp, camera K/D, actual `body_from_camera`, session, database and publisher binding. Its node, map ID, epoch and source stamp must exactly match a node in the current registry snapshot. A missing pose produces an unplaced entry, never an extrapolated coordinate. The actual detector dictionary is checked against OpenCV's `DICT_APRILTAG_36h11` before the family label is accepted.

## Output

`/wbrov/tag_visualization/state` is a reliable, transient-local `std_msgs/String` containing a complete snapshot:

```json
{
  "schema": "wbrov.tag-visualization.v1",
  "session_short_hash": "first 16 hex characters",
  "session_token_sha256": "full 64 lowercase hex characters",
  "registry_revision": 12,
  "visualization_valid": true,
  "tags": [{
    "family": "tag36h11",
    "tag_id": 7,
    "map_id": 1,
    "reset_epoch": 1,
    "component_id": 23,
    "submap_key": "m1_e1_c23",
    "frame_id": "wbrov_submap_<session16>_m1_e1_c23",
    "node_id": 24,
    "source_stamp_ns": "123456789000",
    "placement_status": "local_submap",
    "geometry_status": "single_view_position_estimate",
    "relocalization_verified": false,
    "observation_is_historical": true,
    "pixel_record_sha256": "64 lowercase hex characters",
    "center": [0.1, 0.2, 1.0],
    "corners3d": [[0.06, 0.16, 1.0], [0.14, 0.16, 1.0], [0.14, 0.24, 1.0], [0.06, 0.24, 1.0]],
    "observation_count": 1,
    "reprojection_rms_at_960": 0.1,
    "planar_branch_spread_m": 0.002
  }]
}
```

`placement_status` is `local_submap`, `main_registered` or `unplaced`. Only the first two contain `center` and `corners3d`. `main_registered` requires the registry explicitly to mark that component registered, with the registry's main frame. Unplaced entries may have null component/key/frame if their receipt node is absent. They carry a reason in `geometry_status`, and must never generate spatial markers. A top-level invalidation publishes `visualization_valid=false`, `reason`, and `tags=[]` to clear previously shown markers.

The display consumer must bind full session hash, revision, submap key, map/epoch, frame and node ID to the same registry snapshot. It must not transform an unregistered submap into `map`. The display is historical map evidence, not a live robot pose or a vote for map registration.

## Geometry and retention

OpenCV IPPE_SQUARE uses the measured ordered corners, nominal Tag side length, and actual CameraInfo K/D. Both planar branches are refined and checked for finite geometry, positive camera depth and reprojection RMS at most 3 pixels normalized to width 960. If surviving branches differ by more than 2 cm at any corner, the observation is unplaced. This is conservative single-view position display; it is not multiview localization validation.

The transform order is exactly `registry_frame_from_body(node) * receipt_body_from_camera * camera_corner`. Cached camera/body measurements are transformed again for each registry revision, so graph optimization and later registration update positions. The cache is bounded to 4096 node/Tag records and 64 observations per map/epoch/ID. One output entry per component/family/ID is selected by latest exact source stamp; the same ID in different unaligned components remains separate. Repeated identical observations do not append markers. Contradictory same-node evidence or observed centers spread more than 10 cm within a component produces an unplaced conflict. These display thresholds are candidate settings, not underwater calibration claims.

## Validation

Run from the repository root:

```sh
python3 -B -m unittest discover -s runtime/libexec/wbrov_corner_recovery -p test_tag_visualization.py -v
```

The actual receipt and pixel validators are included beside the tests. The 22 standard-library behavior tests cover identity, missing poses, component separation, replacement/deduplication, optimization, registration, exact extrinsic direction, conflicts, source authentication, stale clearing, revision heartbeat semantics, parser capacity and asynchronous registry/binding order. Three additional tests execute real `cv2.projectPoints` and IPPE (no mocked pose), including camera-to-body-to-submap, promotion to main, optimization updates and ambiguous-pose rejection. Local Windows Python lacks NumPy/OpenCV and skips those three. All 25 tests passed without skips in the isolated Jetson candidate runtime; see the candidate validation record.

Native projection fixture correction: the original 0.8 m / 0.2 rad tilt positive fixture has two plausible poses separated by 0.0217148 m on Jetson OpenCV 4.5.4. It correctly fails the existing 0.02 m ambiguity threshold. That view is now a dedicated negative test; the two placement tests use 0.35 m / 0.7 rad tilt, where the other branch fails the unchanged reprojection checks. There are now 22 standard-library tests and 3 real OpenCV tests. No estimator or acceptance threshold was loosened.

Recorded replay may contain no Tag. A tag-enabled replay with zero verified detections is `notobserved`, not a successful Tag placement test. The projection tests are synthetic numerical validation; they do not replace recorded detections or field validation.
