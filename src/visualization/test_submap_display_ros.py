"""Native ROS smoke: real Node lifecycle and DDS marker frame transitions."""
import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rtabmap_submap_display import SubmapDisplayNode, MAIN_TAG_TOPIC
from test_submap_display_geometry import registry_fixture, tag_fixture
from submap_display_geometry import parse_registry


def main():
    rclpy.init()
    display = observer = executor = None
    try:
        display = SubmapDisplayNode()
        observer = Node('submap_display_smoke_observer', start_parameter_services=False)
        executor = SingleThreadedExecutor()
        executor.add_node(display)
        executor.add_node(observer)
        main_messages, local_messages = [], []
        observer.create_subscription(MarkerArray, MAIN_TAG_TOPIC, main_messages.append, display.qos)
        local = registry_fixture()
        entry = local['submaps'][0]
        observer.create_subscription(MarkerArray, entry['display_topic'], local_messages.append, display.qos)

        def spin_until(predicate):
            deadline = time.monotonic() + 5
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            assert predicate(), 'Expected DDS marker message did not arrive'

        display.on_registry(String(data=json.dumps(local)))
        display.on_tags(String(data=json.dumps(tag_fixture(parse_registry(local)))))
        spin_until(lambda: any(any(m.ns == 'tag_estimated_center' for m in a.markers) for a in local_messages) and bool(main_messages))
        placed = next(a for a in local_messages if any(m.ns == 'tag_estimated_center' for m in a.markers))
        assert all(m.header.frame_id == entry['frame_id'] for m in placed.markers)
        assert all(not any(m.ns == 'tag_estimated_center' for m in a.markers) for a in main_messages)

        # A burst of duplicate odometry status must not flood the display.
        before_local, before_main = len(local_messages), len(main_messages)
        status = String(data=json.dumps(dict(schema='wbrov.mapping-continuity.v1',
            session_token_sha256=local['session_token_sha256'], epoch=3,
            local_tracking_healthy=True, local_submap_write_allowed=True)))
        for _ in range(1000):
            display.on_continuity_status(status)
        deadline = time.monotonic() + 1.1
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.03)
        assert len(local_messages) - before_local <= 7, 'Duplicate status flooded local markers'
        assert len(main_messages) - before_main <= 2, 'Duplicate status flooded main markers'

        registered = registry_fixture(True, 2)
        row = tag_fixture(parse_registry(registered))
        row['tags'][0]['center'] = [5, 6, 7]
        display.on_registry(String(data=json.dumps(registered)))
        display.on_tags(String(data=json.dumps(row)))
        spin_until(lambda: any(any(m.ns == 'tag_estimated_center' and m.pose.position.x == 5.0 for m in a.markers) for a in main_messages))
        center = next(m for a in main_messages for m in a.markers if m.ns == 'tag_estimated_center' and m.pose.position.x == 5.0)
        assert center.header.frame_id == 'map' and center.pose.position.y == 6.0
        display.clear_all()
        display.render_all()
        spin_until(lambda: bool(main_messages) and len(main_messages[-1].markers) == 1 and main_messages[-1].markers[0].action == 3)
        assert not display.entries and not display.submap_publishers
        print(json.dumps(dict(status='PASS', actual_node_created=True, actual_dds_markers=True,
            local_tag_never_in_main=True, registered_tag_uses_main=True, full_clear=True, status_burst_coalesced=True,
            hardware_started=False, field_validated=False)))
    finally:
        if executor is not None:
            executor.shutdown()
        if observer is not None:
            observer.destroy_node()
        if display is not None:
            display.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
