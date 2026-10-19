"""Native DDS check of the docked status image and stale-session handling."""
import json
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rtabmap_graph_display import GraphDisplayNode
from test_submap_display_geometry import registry_fixture


def main():
    rclpy.init()
    display = GraphDisplayNode()
    observer = Node('continuity_display_observer', start_parameter_services=False)
    executor = SingleThreadedExecutor()
    executor.add_node(display)
    executor.add_node(observer)
    images, markers = [], []
    qos = display.mapping_image_pub.qos_profile
    observer.create_subscription(Image, '/mapping/continuity_image', images.append, qos)
    observer.create_subscription(MarkerArray, '/mapping/continuity_display', markers.append, qos)

    def wait_for(predicate):
        deadline = time.monotonic() + 4
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert predicate(), 'Expected DDS status did not arrive'

    try:
        registry = registry_fixture()
        display.on_submap_registry(String(data=json.dumps(registry)))
        row = dict(schema='wbrov.mapping-continuity.v1', state='PAUSED', generation=3,
                   epoch=3, session_token_sha256=registry['session_token_sha256'],
                   main_map_write_allowed=False, quality_verification_pending=True,
                   map_observation_count=2, quality_trigger_reason='cells=1<6')
        display.on_mapping_status(String(data=json.dumps(row)))
        wait_for(lambda: bool(images) and bool(markers))
        paused = bytes(images[-1].data)
        assert (images[-1].width, images[-1].height, images[-1].encoding) == (820, 260, 'bgr8')
        assert len(paused) == 820 * 260 * 3
        assert 'cells=1<6' in markers[-1].markers[0].text
        assert '未对齐子图：1' in markers[-1].markers[0].text

        display.on_reference_readiness(String(data=json.dumps(dict(schema='wbrov.reference-readiness.v1',
            graph_update_authorized=False,source_valid=True,session_token_sha256=registry['session_token_sha256'],
            tags=[dict(tag_id=8,distinct_saved_nodes=4,independent_consistent_triplet=True)]))))
        row.update(localization_status='LOCALIZED',localization_valid=True)
        display.on_mapping_status(String(data=json.dumps(row)))
        wait_for(lambda: '当前位置已定位' in markers[-1].markers[0].text and '三组有效参考齐备' in markers[-1].markers[0].text)
        assert '缓慢退回' not in markers[-1].markers[0].text
        assert row['main_map_write_allowed'] is False
        display.reference_readiness_arrival -= 3
        wait_for(lambda: 'Tag 8' not in markers[-1].markers[0].text)

        row.update(state='NORMAL', main_map_write_allowed=True, quality_verification_pending=False)
        display.on_mapping_status(String(data=json.dumps(row)))
        wait_for(lambda: bool(images) and bytes(images[-1].data) != paused and
                 '持续建图' in markers[-1].markers[0].text)
        normal = bytes(images[-1].data)
        # A retained status cannot keep the dock green after freshness expires.
        display.mapping_status_arrival -= 2
        wait_for(lambda: bytes(images[-1].data) != normal and '状态未更新' in markers[-1].markers[0].text)
        stale = bytes(images[-1].data)
        row['session_token_sha256'] = 'f' * 64
        display.on_mapping_status(String(data=json.dumps(row)))
        display.publish_mapping_status()
        wait_for(lambda: '状态未更新' in markers[-1].markers[0].text)
        assert bytes(images[-1].data) == stale
        print(json.dumps(dict(status='PASS', actual_dds_image=True, dimensions_valid=True,
                              weak_cause_visible=True, unaligned_count_visible=True,
                              stale_and_wrong_session_fail_closed=True, hardware_started=False)))
    finally:
        executor.shutdown()
        observer.destroy_node()
        display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
