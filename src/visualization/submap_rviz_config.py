"""Create one RViz view using a real local fixed frame. Never publishes TF."""
import json
from pathlib import Path
from submap_display_geometry import PALETTE


def configuration(submap):
    # JSON quoted strings are valid YAML scalars and prevent labels becoming YAML.
    quote = json.dumps
    color = "; ".join(str(round(v*255)) for v in PALETTE[submap.map_id % len(PALETTE)][:3])
    return f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Local_Grid_Not_Main_Map
      Reference Frame: <Fixed Frame>
      Cell Size: 0.5
      Plane Cell Count: 20
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: Temporary_Submap_Unaligned
      Topic:
        Value: {quote(submap.cloud_topic)}
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        History Policy: Keep Last
      Color Transformer: FlatColor
      Color: {color}
      Style: Points
      Size (Pixels): 2
      Position Transformer: XYZ
      Decay Time: 0
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Local_Trajectory_Status_And_Tags
      Topic:
        Value: {quote(submap.display_topic)}
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        History Policy: Keep Last
  Global Options:
    Background Color: 30; 25; 20
    Fixed Frame: {quote(submap.frame_id)}
    Frame Rate: 20
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 4
      Pitch: 0.55
      Yaw: 0.785
Window Geometry:
  Width: 1000
  Height: 760
'''


def write_configurations(directory, registry):
    """Write only bounded, identity-derived filenames under the requested dir.

    This does not launch or switch RViz. Launcher may open active_submap.rviz in
    a separate window, or the operator can select a per-submap configuration.
    """
    directory = Path(directory)
    if not directory.is_absolute() or not directory.is_dir():
        raise ValueError("RViz output must be an existing absolute directory")
    records = []
    active = None
    for submap in registry.submaps:
        if submap.registered:
            continue
        path = directory / (submap.frame_id + ".rviz")
        content = configuration(submap)
        if not path.exists() or path.read_text(encoding="utf-8") != content:
            path.write_text(content, encoding="utf-8", newline="\n")
        record = dict(map_id=submap.map_id, reset_epoch=submap.reset_epoch, frame_id=submap.frame_id,
                      graph_topic=submap.graph_topic, cloud_topic=submap.cloud_topic,
                      display_topic=submap.display_topic, active=submap.active, path=str(path))
        records.append(record)
        if submap.active:
            active = record
    manifest = dict(schema="wbrov.submap-views.v1", session_short_hash=registry.session_short_hash,
                    registry_revision=registry.revision, views=records, active=active,
                    tf_published=False, main_frame_unchanged=True)
    (directory / "submap_views.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    if active:
        # Copying configuration does not relabel cloud coordinates. The fixed
        # frame and topics still identify this exact local submap.
        (directory / "active_submap.rviz").write_text(Path(active["path"]).read_text(encoding="utf-8"), encoding="utf-8")
    return manifest
