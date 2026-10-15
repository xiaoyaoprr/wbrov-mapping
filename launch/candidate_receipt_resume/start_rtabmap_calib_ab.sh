#!/usr/bin/bash
set -eo pipefail
umask 077

# Establish a closed command/import environment before the first external
# command.  Caller-provided search paths must never select a different helper,
# ROS package or Python module than the reviewed release.
readonly managed_system_path='/opt/ros/humble/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
export PATH="$managed_system_path"
if [[ -n "${BASH_ENV:-}" || -n "${ENV:-}" ]]; then
    printf '%s\n' '[启动拒绝] Bash启动环境不是由8号受审入口清理的最小环境。' >&2
    exit 1
fi
if [[ -n "${LD_PRELOAD:-}" || -n "${LD_AUDIT:-}" ]]; then
    printf '%s\n' '[启动拒绝] 调用方设置了LD_PRELOAD或LD_AUDIT。' >&2
    exit 1
fi
unset BASH_ENV ENV AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
# Never inherit a private Core diagnostic output from another session.
unset WBROV_PRIVATE_OPTIMIZER_TRACE
unset RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
export ROS_DOMAIN_ID=0
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1

# BEGIN PINNED_DDS_TRANSPORT
readonly wbrov_dds_profile='/home/exocean/.cache/wbrov_dds_profiles/sha256-2ea911a3e61a1f9adc9afed60e44a4e3f57444ef4ffa3f0ebba410c584cd7a7f/fastdds.xml'
readonly wbrov_dds_profile_sha256='2ea911a3e61a1f9adc9afed60e44a4e3f57444ef4ffa3f0ebba410c584cd7a7f'
wbrov_check_dds_profile() {
    local actual
    [[ -f "$wbrov_dds_profile" && ! -L "$wbrov_dds_profile" ]] || {
        echo 'DDS 图像通信配置缺失或不是普通文件。' >&2; return 1;
    }
    [[ "$(readlink -f -- "$wbrov_dds_profile")" == "$wbrov_dds_profile" &&
       "$(stat -c '%u:%h:%a' -- "$wbrov_dds_profile")" == "$(id -u):1:444" ]] || {
        echo 'DDS 图像通信配置的路径、属主或权限不匹配。' >&2; return 1;
    }
    actual="$(sha256sum -- "$wbrov_dds_profile")" || return 1
    [[ "${actual%% *}" == "$wbrov_dds_profile_sha256" ]] || {
        echo 'DDS 图像通信配置校验失败。' >&2; return 1;
    }
}
wbrov_enable_dds_profile() {
    wbrov_check_dds_profile || return 1
    unset FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML CYCLONEDDS_URI
    export FASTRTPS_DEFAULT_PROFILES_FILE="$wbrov_dds_profile"
    export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
}
# END PINNED_DDS_TRANSPORT
wbrov_enable_dds_profile || exit 1
if [[ $# -eq 1 && "$1" == --check-dds-transport ]]; then
    printf '{"shared_memory_bytes":67108864,"profile":"%s","sha256":"%s","hardware_started":false}\n' "$wbrov_dds_profile" "$wbrov_dds_profile_sha256"
    exit 0
fi

# Capture every caller-owned mode before any ROS or legacy configuration is
# sourced.  Sourced files may provide implementation defaults, but cannot
# silently change which database, calibration, light policy or lifetime the
# operator selected for this invocation.  `${name-default}` preserves an
# explicitly empty value so validation can reject it.
readonly caller_profile="${CALIB_AB_PROFILE-custom}"
readonly caller_calibration_file="${CALIB_AB_CALIBRATION_FILE-/home/exocean/calibration/zedx_charuco/water_calib_trial_20260827_from_unique86/results/run_c2ce22daf28d/zed_opencv_calibration_40408468.yml}"
readonly caller_light_mode="${CALIB_AB_LIGHT_MODE-teleop}"
readonly caller_light_hook="${CALIB_AB_LIGHT_HOOK-}"
readonly caller_continuation_request="${CALIB_AB_CONTINUE_RUN_DIR-new}"
readonly caller_duration_s="${CALIB_AB_DURATION_S-0}"
readonly caller_enable_imu="${RTAB_ENABLE_IMU-1}"

# BEGIN PURE_CAMERA_PROFILE_CONFIG
# Capture the caller's profile before any legacy environment file is sourced.
# An explicit empty value is rejected instead of silently selecting a default.
readonly default_camera_profile='dual_hd1200_60_odom384x240_20_map960x600_1_odom_ack'
readonly rollback_camera_profile='single_hd1200_30_640x400'
readonly dual_stream_binary='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/frontend_candidates/accel_gravity_2b1634ae850a3405/zed_rgbd_publisher_rate_ab'
# Frozen ARM64 dual-stream bridge candidate.  The managed runtime observer is
# still an independent fail-closed deployment prerequisite.
readonly dual_stream_binary_sha256='2b1634ae850a340590257545e89f382b1d29625b494af318123fef6430fb9f50'
readonly private_runtime_root='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-0003cd4f818852d584959c6ee2602e92117d4c80ac194a42ad0e87051cf566b2'
readonly camera_health_observer="$private_runtime_root/bin/header_health_observer"
readonly camera_health_observer_sha256='a04aa06117bae16265f4d836406af274b1de105dff9c53e3a6b6bad7c0886018'
readonly apriltag_anchor_builder='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/apriltag_anchor_builder_exact/80612ee432f9655a1970739b432cebbc8f50627026abdea2e8b2c15a885771a6/build_apriltag_multiview_anchors.py'
readonly apriltag_anchor_builder_sha256='80612ee432f9655a1970739b432cebbc8f50627026abdea2e8b2c15a885771a6'
readonly apriltag_detector='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/apriltag_detector_rich/63481b730560e5037dec5671759fb4f751b2b695cbff841627590bf622242320/apriltag_landmark_node.py'
readonly apriltag_detector_sha256='63481b730560e5037dec5671759fb4f751b2b695cbff841627590bf622242320'
readonly apriltag_pose_guard="${apriltag_detector%/*}/tag_pose_guard.py"
readonly apriltag_pose_guard_sha256='dfd3bb8406281d6898b9dca9565f3b68ef2e5a6ecc850c020a70670554da6c45'
readonly apriltag_pose_geometry="${apriltag_detector%/*}/tag_pose_geometry.py"
readonly apriltag_pose_geometry_sha256='0a8d8d6fc977bb512e2d1f3cc638a3f188986cf6a1f966105e6c34ea9b69f810'
readonly apriltag_keyframe_binder='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/apriltag_keyframe_binding/1b4262ede8f17dc15e03c7a9e93951136e2b77bf1b81f0f4a467e1fa5a56c41d/apriltag_keyframe_binder.py'
readonly apriltag_keyframe_binder_sha256='1b4262ede8f17dc15e03c7a9e93951136e2b77bf1b81f0f4a467e1fa5a56c41d'
readonly safe_reloc_gid_frontend="$private_runtime_root/bin/safe_reloc_gid_frontend"
readonly safe_reloc_gid_frontend_sha256='a09155c9aa28dcf297c3373b27730ab0775ba9b50ebef1d00cc8d855611e39bd'
readonly safe_reloc_ipc_engine="$private_runtime_root/libexec/safe_reloc_ipc_engine.py"
readonly safe_reloc_ipc_engine_sha256='e0568a37c397f99e1ae492119c6f3a41f763708ce4122e47024fa5e1c2611ba9'
readonly safe_reloc_engine_core="$private_runtime_root/libexec/safe_reloc_engine_core.py"
readonly safe_reloc_engine_core_sha256='314314f138e4057c1b5e074cc31fc98e3c3652bfab26cb0fb9551e3d6464f555'
readonly safe_reloc_identity_writer="$private_runtime_root/libexec/write_process_identity.py"
readonly safe_reloc_identity_writer_sha256='06b99b3f26955770a6bb3cfb7c6054f8e2fef6655c90ce513e4371d9161143f5'
readonly corner_recovery_root="$private_runtime_root/libexec/wbrov_corner_recovery"
readonly corner_recovery_consumer="$corner_recovery_root/consumer.py"
readonly corner_recovery_consumer_sha256='6b7b4b44526c3e9b472b0772531ec4292725ecbe83e7c30d1c27354d8dcee36f'
readonly corner_recovery_renderer="$corner_recovery_root/render_config.py"
readonly corner_recovery_renderer_sha256='74957100aa6f959868b55efa5c8effadb536342f7774e95fb1099aa3a9ca620e'
readonly corner_recovery_template="$corner_recovery_root/production_config_template.json"
readonly corner_recovery_template_sha256='32646c8505394de531af3041fad50578ee7ab57ccfca2f0e23cf87efe2165063'
readonly corner_recovery_message_info="$corner_recovery_root/libwbrov_message_info.so"
readonly corner_recovery_message_info_sha256='d70b03be476f1beb0b6727d6bb76885aee568160cb9a56852b122ee21c7264b2'
# Compatibility name retained only in the continuation contract.  The actual
# ROS subscriber is the native GID frontend; this file is the pure IPC engine.
readonly apriltag_safe_relocalizer="$corner_recovery_consumer"
readonly apriltag_safe_relocalizer_sha256="$corner_recovery_consumer_sha256"
run_apriltag_anchor_builder='' run_apriltag_detector=''
run_apriltag_keyframe_binder=''
readonly visualization_runtime_root="$private_runtime_root/libexec"
readonly graph_display_script="$visualization_runtime_root/rtabmap_graph_display.py"
readonly submap_display_script="$visualization_runtime_root/rtabmap_submap_display.py"
readonly submap_display_script_sha256='843a002677a2cba18b796f43815e57ff3b0956fd47c7e1929864db0d6e03efdc'
readonly graph_display_script_sha256='125772c67130460b82a6f6a38e0a764d41995314c327a7975ec709471403c238'
readonly graph_display_module="$visualization_runtime_root/graph_display_geometry.py"
readonly submap_display_module="$visualization_runtime_root/submap_display_geometry.py"
readonly submap_display_module_sha256='21827d469bde4ab74e2892b79e7819ffb9b33d1e982274df6608cd7bd3bf69b0'
readonly graph_display_module_sha256='5bd28861322e8c98afedd9d61b313a2f740d143ebfd5e48429f39ba7025977df'
readonly test_rviz="$visualization_runtime_root/rtabmap_dense_graph_optimized.rviz"
readonly test_rviz_sha256='86cf66fcc8f4f8c85b1653dbcd5dd9c93876ff807a1b92e779d598471e8b7345'
readonly python3_runtime='/usr/bin/python3.10'
readonly python3_runtime_sha256='03bb5d246e83c44204ef38044b062fdc46c2835881874e763c44c48b4304c490'
readonly static_tf_binary='/opt/ros/humble/lib/tf2_ros/static_transform_publisher'
readonly static_tf_binary_sha256='9443966d0a2a5b6b43a2a0c3d03dd1de6e40905d88b7443e449d91a21718704f'
readonly cloud_frame_binary="$private_runtime_root/bin/point_cloud_xyzrgb"
readonly cloud_frame_binary_sha256='3dab14f288f27de23f0e2dcb83242c3a86fda4264d29fb49b6fd9f8ef9663488'
readonly rviz_binary='/opt/ros/humble/bin/rviz2'
readonly rviz_binary_sha256='358dd38dc96e2c957e2c5a799c37304c35514c99198e5982ab1bba599b5c7da5'
readonly exporter_binary="$private_runtime_root/bin/rtabmap-export"
readonly exporter_binary_sha256='5ba1979257e7f1b4c2e5c00c70b6aa75a22c31b6a8bddccd8b837bd46c200c8a'
run_exporter_binary=''
readonly sleep_binary='/usr/bin/sleep'
readonly sleep_binary_sha256='3a442bbbae8cf5d89c0a1b49a60f73855d6287bdf232786805d96f57ec4279da'
readonly camera_health_window_s=20
readonly camera_health_odom_min_hz=15
readonly camera_health_high_min_hz=0.45
readonly camera_health_coverage_min=0.90
readonly camera_health_integration_min_hz=0.20
readonly legacy_binary_sha256='3aa422fed28e14c34c6182b6277b1717cef6a4fee571333e795e37a615d66c27'

capture_camera_profile() {
    requested_camera_profile="${CALIB_AB_CAMERA_PROFILE-$default_camera_profile}"
    readonly requested_camera_profile
}

resolve_camera_profile() {
    case "$requested_camera_profile" in
        "$default_camera_profile")
            camera_profile="$default_camera_profile"
            camera_topology='one_sdk_owner_dual_rgbd'
            camera_grab_resolution='HD1200'
            camera_grab_fps=60
            camera_compute_cap_fps=30
            odom_width=384
            odom_height=240
            # The atomic RGB-D baseline schedules 20 Hz so the user's >=15 Hz
            # effective odometry acceptance is physically attainable.
            odom_publish_hz=20
            odom_effective_health_min_hz=15
            map_width=960
            map_height=600
            map_publish_hz=1.0
            odom_rgb_topic='/zed/rgb/image_rect_color'
            odom_depth_topic='/zed/depth/depth_registered'
            odom_display_depth_topic='/zed/depth/depth_display'
            odom_info_topic='/zed/rgb/camera_info'
            odom_rgbd_topic='/zed/rgbd_image'
            map_rgb_topic='/zed/mapping/rgb/image_rect_color'
            map_depth_topic='/zed/mapping/depth/depth_registered'
            map_info_topic='/zed/mapping/rgb/camera_info'
            slam_topic_queue_size=10
            slam_sync_queue_size=10
            camera_stamp_contract='low_triplet_published_before_high_retrieve;map_triplet_exact;map_stamp_is_low_triplet_stamp;map_publish_after_exact_valid_odom_ack;odom_propagates_low_stamp'
            ;;
        "$rollback_camera_profile")
            camera_profile="$rollback_camera_profile"
            camera_topology='one_sdk_owner_single_rgbd'
            camera_grab_resolution='HD1200'
            camera_grab_fps=30
            camera_compute_cap_fps=30
            odom_width=640
            odom_height=400
            odom_publish_hz=30
            odom_effective_health_min_hz=0
            map_width=640
            map_height=400
            map_publish_hz=30
            odom_rgb_topic='/zed/rgb/image_rect_color'
            odom_depth_topic='/zed/depth/depth_registered'
            odom_display_depth_topic='/zed/depth/depth_display'
            odom_info_topic='/zed/rgb/camera_info'
            map_rgb_topic="$odom_rgb_topic"
            map_depth_topic="$odom_depth_topic"
            map_info_topic="$odom_info_topic"
            slam_topic_queue_size=30
            slam_sync_queue_size=50
            camera_stamp_contract='single_rgb_depth_info_stamp;odom_propagates_image_stamp'
            ;;
        *)
            echo "CALIB_AB_CAMERA_PROFILE无效（空值也拒绝）: $requested_camera_profile" >&2
            return 1
            ;;
    esac
}

apply_camera_profile_environment() {
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        export ZED_GRAB_RESOLUTION="$camera_grab_resolution"
        export ZED_GRAB_FPS="$camera_grab_fps"
        export ZED_COMPUTE_CAP_FPS="$camera_compute_cap_fps"
        export ZED_ODOM_WIDTH="$odom_width"
        export ZED_ODOM_HEIGHT="$odom_height"
        export ZED_ODOM_FPS="$odom_publish_hz"
        export ZED_MAP_WIDTH="$map_width"
        export ZED_MAP_HEIGHT="$map_height"
        export ZED_MAP_FPS="$map_publish_hz"
        # Compatibility values are not the dual bridge ABI.  The frozen dual
        # bridge reads only the ZED_* dimensions/rates above.
        export RTAB_WIDTH="$odom_width"
        export RTAB_HEIGHT="$odom_height"
        export RTAB_FPS="$camera_grab_fps"
    else
        unset ZED_GRAB_RESOLUTION ZED_GRAB_FPS ZED_COMPUTE_CAP_FPS ZED_ODOM_WIDTH ZED_ODOM_HEIGHT \
            ZED_ODOM_FPS ZED_MAP_WIDTH ZED_MAP_HEIGHT ZED_MAP_FPS
        export RTAB_WIDTH="$odom_width"
        export RTAB_HEIGHT="$odom_height"
        export RTAB_FPS="$camera_grab_fps"
    fi
}

print_camera_profile() {
    printf 'CAMERA_PROFILE_SCHEMA_VERSION=5\n'
    printf 'CAMERA_PROFILE=%s\n' "$camera_profile"
    printf 'CAMERA_TOPOLOGY=%s\n' "$camera_topology"
    printf 'CAMERA_GRAB_RESOLUTION=%s\n' "$camera_grab_resolution"
    printf 'CAMERA_GRAB_FPS=%s\n' "$camera_grab_fps"
    printf 'CAMERA_COMPUTE_CAP_FPS=%s\n' "$camera_compute_cap_fps"
    printf 'ODOM_STREAM=%sx%s@%s\n' "$odom_width" "$odom_height" "$odom_publish_hz"
    printf 'ODOM_RGB_TOPIC=%s\n' "$odom_rgb_topic"
    printf 'ODOM_DEPTH_TOPIC=%s\n' "$odom_depth_topic"
    printf 'ODOM_DISPLAY_DEPTH_TOPIC=%s\n' "$odom_display_depth_topic"
    printf 'ODOM_CAMERA_INFO_TOPIC=%s\n' "$odom_info_topic"
    printf 'ODOM_EFFECTIVE_HEALTH_MIN_HZ=%s\n' "$odom_effective_health_min_hz"
    printf 'MAP_STREAM=%sx%s@%s\n' "$map_width" "$map_height" "$map_publish_hz"
    printf 'MAP_RGB_TOPIC=%s\n' "$map_rgb_topic"
    printf 'MAP_DEPTH_TOPIC=%s\n' "$map_depth_topic"
    printf 'MAP_CAMERA_INFO_TOPIC=%s\n' "$map_info_topic"
    printf 'CAMERA_STAMP_CONTRACT=%s\n' "$camera_stamp_contract"
    printf 'BRIDGE_BINARY=%s\n' "$test_binary"
    printf 'BRIDGE_BINARY_SHA256=%s\n' "$test_binary_expected_sha256"
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        printf 'CAMERA_HEALTH_OBSERVER_REQUIRED=true\n'
        printf 'CAMERA_HEALTH_OBSERVER=%s\n' "$camera_health_observer"
        printf 'CAMERA_HEALTH_OBSERVER_SHA256=%s\n' "$camera_health_observer_sha256"
        printf 'CAMERA_HEALTH_WINDOW_S=%s\n' "$camera_health_window_s"
        printf 'CAMERA_HEALTH_VALID_ODOM_MIN_HZ=%s\n' "$camera_health_odom_min_hz"
        printf 'CAMERA_HEALTH_HIGH_MIN_HZ=%s\n' "$camera_health_high_min_hz"
        printf 'CAMERA_HEALTH_EXACT_COVERAGE_MIN=%s\n' "$camera_health_coverage_min"
        printf 'CAMERA_HEALTH_INTEGRATION_MIN_HZ=%s\n' "$camera_health_integration_min_hz"
        printf 'CAMERA_HEALTH_ODOM_INFO_TOPIC=/odom_info_lite\n'
        printf 'CAMERA_HEALTH_RTABMAP_INFO_TOPIC=/info\n'
    else
        printf 'CAMERA_HEALTH_OBSERVER_REQUIRED=false\n'
    fi
    printf 'PROFILE_FALLBACK=none_fail_closed\n'
}

check_camera_profile_binary() {
    if [[ ! "$test_binary_expected_sha256" =~ ^[0-9a-f]{64}$ ]]; then
        echo "相机profile发布器哈希尚未冻结，拒绝启动: $camera_profile" >&2
        return 1
    fi
    if [[ -L "$test_binary" || ! -f "$test_binary" || ! -x "$test_binary" ]]; then
        echo "相机profile发布器必须是可执行普通文件且不能是符号链接: $test_binary" >&2
        return 1
    fi
    local owner mode links
    owner="$(stat -c %u -- "$test_binary")" || return 1
    mode="$(stat -c %a -- "$test_binary")" || return 1
    links="$(stat -c %h -- "$test_binary")" || return 1
    if [[ "$owner" != "$(id -u)" || "$links" != 1 || \
          "$mode" =~ [2367][0-7]$ || "$mode" =~ [2367]$ ]]; then
        echo "相机profile发布器属主/链接/写权限不安全: uid=$owner mode=$mode links=$links" >&2
        return 1
    fi
    local actual_sha256
    actual_sha256="$(sha256sum -- "$test_binary")" || return 1
    actual_sha256="${actual_sha256%% *}"
    if [[ "$actual_sha256" != "$test_binary_expected_sha256" ]]; then
        echo "相机profile发布器哈希不匹配: expected=$test_binary_expected_sha256 actual=$actual_sha256" >&2
        return 1
    fi
}

check_camera_health_file() {
    local path="$1" expected_sha256="$2" owner mode links actual_sha256
    if [[ ! "$expected_sha256" =~ ^[0-9a-f]{64}$ ]]; then
        echo "相机健康observer哈希尚未冻结: $path" >&2
        return 1
    fi
    if [[ -L "$path" || ! -f "$path" || ! -r "$path" ]]; then
        echo "相机健康observer依赖必须是可读普通文件且不能是符号链接: $path" >&2
        return 1
    fi
    owner="$(stat -c %u -- "$path")" || return 1
    mode="$(stat -c %a -- "$path")" || return 1
    links="$(stat -c %h -- "$path")" || return 1
    if [[ "$owner" != "$(id -u)" || "$links" != 1 || \
          "$mode" =~ [2367][0-7]$ || "$mode" =~ [2367]$ ]]; then
        echo "相机健康observer依赖属主/链接/写权限不安全: path=$path uid=$owner mode=$mode links=$links" >&2
        return 1
    fi
    actual_sha256="$(sha256sum -- "$path")" || return 1
    actual_sha256="${actual_sha256%% *}"
    if [[ "$actual_sha256" != "$expected_sha256" ]]; then
        echo "相机健康observer依赖哈希不匹配: path=$path expected=$expected_sha256 actual=$actual_sha256" >&2
        return 1
    fi
}

copy_verified_runtime_file() {
    local source="$1" expected="$2" destination="$3" actual
    check_camera_health_file "$source" "$expected" || return 1
    cp --reflink=never -- "$source" "$destination" || return 1
    chmod 400 -- "$destination" || return 1
    actual="$(sha256sum -- "$destination")" || return 1
    [[ "${actual%% *}" == "$expected" ]] || return 1
    check_camera_health_file "$source" "$expected" || return 1
    [[ ! -L "$destination" && -f "$destination" &&
       "$(stat -c %u -- "$destination")" == "$(id -u)" &&
       "$(stat -c %a -- "$destination")" == 400 &&
       "$(stat -c %h -- "$destination")" == 1 ]] || return 1
}

prepare_critical_script_runtime() {
    local detector_dir="$run_dir/runtime_scripts/apriltag_detector"
    local helper_dir="$run_dir/runtime_scripts/helpers"
    mkdir -m 700 -p -- "$detector_dir" "$helper_dir" || return 1
    run_apriltag_detector="$detector_dir/apriltag_landmark_node.py"
    run_apriltag_anchor_builder="$helper_dir/build_apriltag_multiview_anchors.py"
    run_apriltag_keyframe_binder="$helper_dir/apriltag_keyframe_binder.py"
    copy_verified_runtime_file "$apriltag_detector" "$apriltag_detector_sha256" "$run_apriltag_detector" || return 1
    copy_verified_runtime_file "$apriltag_pose_guard" "$apriltag_pose_guard_sha256" "$detector_dir/tag_pose_guard.py" || return 1
    copy_verified_runtime_file "$apriltag_pose_geometry" "$apriltag_pose_geometry_sha256" "$detector_dir/tag_pose_geometry.py" || return 1
    copy_verified_runtime_file "$apriltag_anchor_builder" "$apriltag_anchor_builder_sha256" "$run_apriltag_anchor_builder" || return 1
    copy_verified_runtime_file "$apriltag_keyframe_binder" "$apriltag_keyframe_binder_sha256" "$run_apriltag_keyframe_binder" || return 1
}

prepare_exporter_runtime() {
    local runtime_dir="$run_dir/runtime_bins" actual
    check_frozen_runtime_file "$exporter_binary" "$exporter_binary_sha256" true || return 1
    mkdir -m 700 -p -- "$runtime_dir" || return 1
    run_exporter_binary="$runtime_dir/rtabmap-export"
    cp --reflink=never -- "$exporter_binary" "$run_exporter_binary" || return 1
    chmod 500 -- "$run_exporter_binary" || return 1
    actual="$(sha256sum -- "$run_exporter_binary")" || return 1
    [[ "${actual%% *}" == "$exporter_binary_sha256" &&
       "$(stat -c %u -- "$run_exporter_binary")" == "$(id -u)" &&
       "$(stat -c %a -- "$run_exporter_binary")" == 500 &&
       "$(stat -c %h -- "$run_exporter_binary")" == 1 ]] || return 1
    check_frozen_runtime_file "$exporter_binary" "$exporter_binary_sha256" true || return 1
}

check_frozen_runtime_file() {
    local path="$1" expected_sha256="$2" require_executable="${3:-false}"
    local canonical owner mode links actual_sha256
    [[ "$expected_sha256" =~ ^[0-9a-f]{64}$ ]] || {
        echo "冻结运行文件哈希无效: $path" >&2
        return 1
    }
    [[ -f "$path" && ! -L "$path" && -r "$path" ]] || {
        echo "冻结运行文件不存在、不可读或是符号链接: $path" >&2
        return 1
    }
    [[ "$require_executable" != true || -x "$path" ]] || {
        echo "冻结运行文件不可执行: $path" >&2
        return 1
    }
    canonical="$(readlink -f -- "$path")" || return 1
    [[ "$canonical" == "$path" ]] || {
        echo "冻结运行文件路径不是精确规范路径: $path -> $canonical" >&2
        return 1
    }
    read -r owner links mode < <(stat -Lc '%u %h %a' -- "$path") || return 1
    if [[ "$owner" != 0 && "$owner" != "$(id -u)" ]] ||
       [[ "$links" != 1 ]] || (( (8#$mode & 0022) != 0 )); then
        echo "冻结运行文件身份/权限不安全: path=$path uid=$owner links=$links mode=$mode" >&2
        return 1
    fi
    actual_sha256="$(sha256sum -- "$path")" || return 1
    actual_sha256="${actual_sha256%% *}"
    [[ "$actual_sha256" == "$expected_sha256" ]] || {
        echo "冻结运行文件哈希不匹配: path=$path expected=$expected_sha256 actual=$actual_sha256" >&2
        return 1
    }
}

prepare_visualization_runtime() {
    local destination_root="$1"
    "$python3_runtime" - \
        "$destination_root" \
        "$graph_display_script" "$graph_display_script_sha256" \
        "$graph_display_module" "$graph_display_module_sha256" \
        "$test_rviz" "$test_rviz_sha256" \
        "$submap_display_module" "$submap_display_module_sha256" <<'VISUALIZATION_COPY_PY'
import hashlib
import os
import stat
import sys

run_dir = sys.argv[1]
sources = (
    (sys.argv[2], sys.argv[3], "rtabmap_graph_display.py"),
    (sys.argv[4], sys.argv[5], "graph_display_geometry.py"),
    (sys.argv[6], sys.argv[7], "rtabmap_dense_graph_optimized.rviz"),
    (sys.argv[8], sys.argv[9], "submap_display_geometry.py"),
)
directory_flags = (os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC |
                   getattr(os, "O_NOFOLLOW", 0))
run_fd = os.open(run_dir, directory_flags)
try:
    run_info = os.fstat(run_fd)
    if (not stat.S_ISDIR(run_info.st_mode) or run_info.st_uid != os.getuid() or
            stat.S_IMODE(run_info.st_mode) != 0o700):
        raise RuntimeError("unsafe per-run directory")
    os.mkdir("visualization_runtime", 0o700, dir_fd=run_fd)
    visual_fd = os.open("visualization_runtime", directory_flags, dir_fd=run_fd)
    try:
        for source_name, expected_sha, target_name in sources:
            source_fd = os.open(
                source_name,
                os.O_RDONLY | os.O_CLOEXEC | getattr(os, "O_NOFOLLOW", 0))
            try:
                source_info = os.fstat(source_fd)
                if (not stat.S_ISREG(source_info.st_mode) or
                        source_info.st_uid not in (0, os.getuid()) or
                        source_info.st_nlink != 1 or
                        stat.S_IMODE(source_info.st_mode) & 0o022):
                    raise RuntimeError("unsafe visualization source: " + source_name)
                digest = hashlib.sha256()
                while True:
                    block = os.read(source_fd, 1024 * 1024)
                    if not block:
                        break
                    digest.update(block)
                if digest.hexdigest() != expected_sha:
                    raise RuntimeError("visualization source digest changed: " + source_name)
                os.lseek(source_fd, 0, os.SEEK_SET)
                target_fd = os.open(
                    target_name,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC |
                    getattr(os, "O_NOFOLLOW", 0),
                    0o600,
                    dir_fd=visual_fd)
                try:
                    copied = hashlib.sha256()
                    while True:
                        block = os.read(source_fd, 1024 * 1024)
                        if not block:
                            break
                        copied.update(block)
                        offset = 0
                        while offset < len(block):
                            written = os.write(target_fd, block[offset:])
                            if written <= 0:
                                raise RuntimeError("short visualization write")
                            offset += written
                    os.fsync(target_fd)
                    target_info = os.fstat(target_fd)
                    if (not stat.S_ISREG(target_info.st_mode) or
                            target_info.st_uid != os.getuid() or
                            target_info.st_nlink != 1 or
                            stat.S_IMODE(target_info.st_mode) != 0o600 or
                            copied.hexdigest() != expected_sha):
                        raise RuntimeError("unsafe visualization copy: " + target_name)
                finally:
                    os.close(target_fd)
            finally:
                os.close(source_fd)
        os.fsync(visual_fd)
    finally:
        os.close(visual_fd)
    os.fsync(run_fd)
finally:
    os.close(run_fd)
VISUALIZATION_COPY_PY
}

check_camera_profile_dependencies() {
    local runtime_check_ld="$private_runtime_root/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/usr/local/zed/lib:/usr/local/cuda/lib64"
    check_frozen_runtime_file "$python3_runtime" "$python3_runtime_sha256" true || return 1
    check_frozen_runtime_file "$static_tf_binary" "$static_tf_binary_sha256" true || return 1
    check_frozen_runtime_file "$odom_binary" "$odom_binary_sha256" true || return 1
    check_frozen_runtime_file "$cloud_frame_binary" "$cloud_frame_binary_sha256" true || return 1
    check_frozen_runtime_file "$rviz_binary" "$rviz_binary_sha256" true || return 1
    check_frozen_runtime_file "$sleep_binary" "$sleep_binary_sha256" true || return 1
    check_frozen_runtime_file "$graph_display_script" "$graph_display_script_sha256" false || return 1
    check_frozen_runtime_file "$graph_display_module" "$graph_display_module_sha256" false || return 1
    check_frozen_runtime_file "$submap_display_module" "$submap_display_module_sha256" false || return 1
    check_frozen_runtime_file "$test_rviz" "$test_rviz_sha256" false || return 1
    check_frozen_runtime_file "$submap_display_script" "$submap_display_script_sha256" false || return 1
    local display_pythonpath="$private_runtime_root/local/lib/python3.10/dist-packages:$private_runtime_root/lib/python3.10/site-packages:/home/exocean/opt/rtabmap/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"
    PYTHONPATH="$display_pythonpath" LD_LIBRARY_PATH="$runtime_check_ld" \
        "$python3_runtime" -B "$submap_display_script" --check-only || return 1
    check_camera_profile_binary || return 1
    check_camera_health_file "$apriltag_anchor_builder" "$apriltag_anchor_builder_sha256" || return 1
    check_camera_health_file "$apriltag_detector" "$apriltag_detector_sha256" || return 1
    check_camera_health_file "$apriltag_keyframe_binder" "$apriltag_keyframe_binder_sha256" || return 1
    check_frozen_runtime_file "$safe_reloc_gid_frontend" "$safe_reloc_gid_frontend_sha256" true || return 1
    check_frozen_runtime_file "$safe_reloc_ipc_engine" "$safe_reloc_ipc_engine_sha256" false || return 1
    check_frozen_runtime_file "$safe_reloc_engine_core" "$safe_reloc_engine_core_sha256" false || return 1
    check_frozen_runtime_file "$safe_reloc_identity_writer" "$safe_reloc_identity_writer_sha256" false || return 1
    check_frozen_runtime_file "$corner_recovery_consumer" "$corner_recovery_consumer_sha256" false || return 1
    check_frozen_runtime_file "$corner_recovery_renderer" "$corner_recovery_renderer_sha256" false || return 1
    check_frozen_runtime_file "$corner_recovery_template" "$corner_recovery_template_sha256" false || return 1
    check_frozen_runtime_file "$corner_recovery_message_info" "$corner_recovery_message_info_sha256" false || return 1
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" \
        "$apriltag_anchor_builder" --check-only || return 1
    LD_LIBRARY_PATH="$runtime_check_ld" "$safe_reloc_gid_frontend" \
        --check-only \
        --session-token 0000000000000000000000000000000000000000000000000000000000000000 \
        || return 1
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -I -B \
        "$safe_reloc_ipc_engine" \
        --check-only \
        --core-module "$safe_reloc_engine_core" \
        --core-module-sha256 "$safe_reloc_engine_core_sha256" \
        --session-token 0000000000000000000000000000000000000000000000000000000000000000 \
        || return 1
    [[ "$camera_profile" == "$default_camera_profile" ]] || return 0
    check_frozen_runtime_file "$camera_health_observer" "$camera_health_observer_sha256" true || return 1
    LD_LIBRARY_PATH="$runtime_check_ld" "$camera_health_observer" \
        --check-only \
        --duration-s 0 \
        --window-s "$camera_health_window_s" \
        --odom-min-hz "$camera_health_odom_min_hz" \
        --high-min-hz "$camera_health_high_min_hz" \
        --coverage-min "$camera_health_coverage_min" \
        --integration-min-hz "$camera_health_integration_min_hz" \
        --expected-high-width "$map_width" \
        --expected-high-height "$map_height" \
        --odom-topic /odom \
        --odom-info-topic /odom_info_lite \
        --high-camera-info-topic "$map_info_topic" \
        --rtabmap-info-topic /info \
        --require-integration-proxy
}
# END PURE_CAMERA_PROFILE_CONFIG

# BEGIN RESET_QUARANTINE_RUNTIME_CONFIG
readonly reset_runtime_root="$private_runtime_root"
readonly reset_runtime_check="$reset_runtime_root/tools/check_private_runtime.py"
readonly reset_runtime_check_sha256='f8cc1365c76ca21e3e0cb060e0dc36d70cf15f9e0338ddbbbd309c730b4ba2e4'
readonly reset_runtime_manifest="$reset_runtime_root/runtime-manifest.json"
readonly reset_runtime_manifest_sha256='0b44f95e0df8fb4de1a78be3a25fe98c47f3742fdd8a24fbc5c24165399c498b'
readonly reset_runtime_node="$reset_runtime_root/bin/rtabmap"
readonly reset_runtime_node_sha256='63361f4627b50390e000f38c7a8e71b88ece6251dfe1eab5f3507ad099a1dc8c'
readonly odom_binary="$reset_runtime_root/bin/rgbd_odometry"
readonly odom_binary_sha256='5f28ef2802a695f105530f236f2dc6aece01b08f38b304209d20b9a87e3a3825'
readonly reset_runtime_library_directory="$reset_runtime_root/lib"

check_reset_quarantine_runtime() {
    wbrov_check_dds_profile || return 1
    local path expected actual owner links mode
    if [[ -n "${LD_PRELOAD:-}" || -n "${LD_AUDIT:-}" ]]; then
        echo '[reset quarantine runtime rejected] nonempty LD_PRELOAD/LD_AUDIT is forbidden' >&2
        return 1
    fi
    for path in "$reset_runtime_check" "$reset_runtime_manifest" "$reset_runtime_node" "$odom_binary"; do
        [[ -f "$path" && ! -L "$path" ]] || {
            echo "[reset quarantine runtime rejected] missing/unsafe file: $path" >&2
            return 1
        }
        owner="$(stat -c %u -- "$path")" || return 1
        links="$(stat -c %h -- "$path")" || return 1
        mode="$(stat -c %a -- "$path")" || return 1
        if [[ "$owner" != "$(id -u)" || "$links" != 1 ||
              "$mode" =~ [2367][0-7]$ || "$mode" =~ [2367]$ ]]; then
            echo "[reset quarantine runtime rejected] owner/link/mode mismatch: path=$path uid=$owner links=$links mode=$mode" >&2
            return 1
        fi
    done
    for path in "$reset_runtime_root" "$reset_runtime_library_directory" "${reset_runtime_node%/*}"; do
        [[ -d "$path" && ! -L "$path" ]] || {
            echo "[reset quarantine runtime rejected] missing/unsafe directory: $path" >&2
            return 1
        }
        owner="$(stat -c %u -- "$path")" || return 1
        mode="$(stat -c %a -- "$path")" || return 1
        if [[ "$owner" != "$(id -u)" ||
              "$mode" =~ [2367][0-7]$ || "$mode" =~ [2367]$ ]]; then
            echo "[reset quarantine runtime rejected] directory owner/mode mismatch: path=$path uid=$owner mode=$mode" >&2
            return 1
        fi
    done
    actual="$(sha256sum -- "$reset_runtime_check")" || return 1
    actual="${actual%% *}"
    [[ "$actual" == "$reset_runtime_check_sha256" ]] || {
        echo "[reset quarantine runtime rejected] checker hash mismatch: $actual" >&2
        return 1
    }
    actual="$(sha256sum -- "$reset_runtime_manifest")" || return 1
    actual="${actual%% *}"
    [[ "$actual" == "$reset_runtime_manifest_sha256" ]] || {
        echo "[reset quarantine runtime rejected] manifest hash mismatch: $actual" >&2
        return 1
    }
    [[ "$reset_runtime_node_sha256" =~ ^[0-9a-f]{64}$ ]] || {
        echo '[reset quarantine runtime rejected] final node hash is not frozen' >&2
        return 1
    }
    [[ "$odom_binary_sha256" =~ ^[0-9a-f]{64}$ ]] || {
        echo '[reset quarantine runtime rejected] final odometry node hash is not frozen' >&2
        return 1
    }
    actual="$(sha256sum -- "$reset_runtime_node")" || return 1
    actual="${actual%% *}"
    [[ "$actual" == "$reset_runtime_node_sha256" ]] || {
        echo "[reset quarantine runtime rejected] node hash mismatch: $actual" >&2
        return 1
    }
    actual="$(sha256sum -- "$odom_binary")" || return 1
    actual="${actual%% *}"
    [[ "$actual" == "$odom_binary_sha256" ]] || {
        echo "[reset quarantine runtime rejected] odometry node hash mismatch: $actual" >&2
        return 1
    }
    # Retain every original checker operation; only independent ldd -r jobs
    # run concurrently after ALL payload hash/ELF checks have completed.
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
        "$python3_runtime" -I - "$reset_runtime_check" "$reset_runtime_root" <<'PY_RUNTIME_TREE'
import concurrent.futures, importlib.util, sys, time
checker_path, runtime_root = sys.argv[1:]
spec = importlib.util.spec_from_file_location("verified_runtime_checker", checker_path)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)
verified_elf_paths = []
futures = {}
original_dynamic = checker.verify_dynamic_tags
original_ldd = checker.ldd_dependencies
def record_verified_elf(path):
    original_dynamic(path)
    verified_elf_paths.append(path)
checker.verify_dynamic_tags = record_verified_elf
started = time.monotonic()
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    def parallel_ldd(path, root):
        if not futures:
            futures.update((p, pool.submit(original_ldd, p, root)) for p in verified_elf_paths)
        return futures[path].result()
    checker.ldd_dependencies = parallel_ldd
    result = checker.main(["tree", "--runtime-root", runtime_root])
print("[运行库检查] %.2f秒；完整校验，未跳过文件或依赖。" % (time.monotonic() - started), file=sys.stderr)
raise SystemExit(result)
PY_RUNTIME_TREE
}

runtime_process_maps_valid() {
    local pid="$1" role="$2" maps="/proc/$1/maps"
    [[ "$pid" =~ ^[1-9][0-9]*$ && -r "$maps" ]] || return 1
    grep -Fq -- "$reset_runtime_library_directory/librtabmap_core" "$maps" || return 1
    if [[ "$role" == odom ]]; then
        grep -Fq -- "$reset_runtime_library_directory/librtabmap_odom" "$maps" || return 1
        grep -Fq -- "$reset_runtime_library_directory/librtabmap_odom_plugins" "$maps" || return 1
    elif [[ "$role" == slam ]]; then
        grep -Fq -- "$reset_runtime_library_directory/librtabmap_slam_plugins" "$maps" || return 1
    else
        return 1
    fi
    if grep -E '/home/exocean/opt/rtabmap/(opt|usr)/.*librtabmap' "$maps" >/dev/null; then
        return 1
    fi
}

wait_for_runtime_process_maps() {
    local pid="$1" role="$2" attempt
    for ((attempt=0; attempt<100; attempt++)); do
        owned_process_alive "$pid" || return 1
        if runtime_process_maps_valid "$pid" "$role"; then
            return 0
        fi
        "$sleep_binary" 0.1
    done
    return 1
}
# END RESET_QUARANTINE_RUNTIME_CONFIG

# BEGIN PURE_RANGE_CONFIG
# Capture once, before any sourced global file can change the caller's values.
# '-' (not ':-') deliberately preserves an explicit empty value for rejection.
capture_trial_ranges() {
    declare -gA requested_trial_ranges=(
        [RTAB_TRACKING_MIN_M]="${RTAB_TRACKING_MIN_M-0.20}"
        [RTAB_TRACKING_MAX_M]="${RTAB_TRACKING_MAX_M-1.50}"
        [DENSE_MIN_M]="${DENSE_MIN_M-0.20}"
        [DENSE_MAX_M]="${DENSE_MAX_M-1.00}"
        [APRILTAG_MIN_RANGE_M]="${APRILTAG_MIN_RANGE_M-0.30}"
        [APRILTAG_MAX_RANGE_M]="${APRILTAG_MAX_RANGE_M-0.75}"
    )
    readonly -A requested_trial_ranges
}

validate_trial_ranges() {
    "$python3_runtime" - \
        "${requested_trial_ranges[RTAB_TRACKING_MIN_M]}" \
        "${requested_trial_ranges[RTAB_TRACKING_MAX_M]}" \
        "${requested_trial_ranges[DENSE_MIN_M]}" \
        "${requested_trial_ranges[DENSE_MAX_M]}" \
        "${requested_trial_ranges[APRILTAG_MIN_RANGE_M]}" \
        "${requested_trial_ranges[APRILTAG_MAX_RANGE_M]}" <<'RANGE_PY'
import math
import re
import struct
import sys

names = ('RTAB_TRACKING_MIN_M', 'RTAB_TRACKING_MAX_M', 'DENSE_MIN_M',
         'DENSE_MAX_M', 'APRILTAG_MIN_RANGE_M', 'APRILTAG_MAX_RANGE_M')
decimal = re.compile(r'[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?')
values = []
try:
    for name, raw in zip(names, sys.argv[1:]):
        if not decimal.fullmatch(raw):
            raise ValueError(name + ' must be a finite decimal number (empty is not a default)')
        number = float(raw)
        if not math.isfinite(number) or number > 3.4028234663852886e38:
            raise ValueError(name + ' must be finite and not exceed float32 maximum')
        number32 = struct.unpack('f', struct.pack('f', number))[0]
        if not math.isfinite(number32) or number32 <= 0:
            raise ValueError(name + ' must be positive and representable in float32')
        values.append((number, number32))
    for index in (0, 2, 4):
        if not (values[index][0] < values[index + 1][0] and
                values[index][1] < values[index + 1][1]):
            raise ValueError(names[index] + '/' + names[index + 1] + ' require min < max in float32')
    if values[0][0] > 3.0:
        raise ValueError('RTAB_TRACKING_MIN_M exceeds the SDK limit of 3 m')
except (ValueError, OverflowError, struct.error) as error:
    print('[range config rejected] ' + str(error), file=sys.stderr)
    sys.exit(2)
RANGE_PY
}

restore_trial_ranges() {
    local key
    for key in RTAB_TRACKING_MIN_M RTAB_TRACKING_MAX_M DENSE_MIN_M DENSE_MAX_M \
        APRILTAG_MIN_RANGE_M APRILTAG_MAX_RANGE_M; do
        printf -v "$key" '%s' "${requested_trial_ranges[$key]}"
        export "${key?}"
    done
}

print_trial_ranges() {
    local key
    for key in RTAB_TRACKING_MIN_M RTAB_TRACKING_MAX_M DENSE_MIN_M DENSE_MAX_M \
        APRILTAG_MIN_RANGE_M APRILTAG_MAX_RANGE_M; do
        printf '%s=%s\n' "$key" "${requested_trial_ranges[$key]}"
    done
}

ros_double_literal() {
    # ROS distinguishes integer and double parameter literals. Keep a decimal
    # point even when a caller chooses e.g. DENSE_MAX_M=1 or 1e0.
    "$python3_runtime" -c 'import sys; print(format(float(sys.argv[1]), "#.17g"))' "$1"
}
# END PURE_RANGE_CONFIG

# BEGIN PURE_LANDMARK_TIME_GUARD
# Frozen deployment inputs, never inherited from an environment override.
# Frozen same-build conversion library from the content-addressed runtime.
readonly landmark_time_guard_sha256='6e1680cec5a8c825b79711f5e4b87a5ff962b9a267f74dfc14f39212e2806292'
readonly landmark_time_guard_so="$reset_runtime_library_directory/librtabmap_conversions.so"
readonly landmark_time_guard_directory="${landmark_time_guard_so%/*}"
readonly imu_candidate_sha256='72c291636e26785373cdbf901c7961be97eb59999066c094f407f388cae096bf'
readonly imu_candidate_binary='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/zed_rgbd_publisher_calib_test_imu_candidate'

capture_imu_trial_config() {
    enable_imu="$caller_enable_imu"
    if [[ -n "${CALIB_AB_TEST_BINARY:-}" ]]; then
        echo "CALIB_AB_TEST_BINARY覆盖已禁用；camera profile必须绑定冻结发布器。" >&2
        return 1
    fi
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        if [[ "$enable_imu" != 1 ]]; then
            echo "dual-stream profile要求RTAB_ENABLE_IMU=1；完整回退请改用$rollback_camera_profile。" >&2
            return 1
        fi
        test_binary="$dual_stream_binary"
        test_binary_expected_sha256="$dual_stream_binary_sha256"
        return 0
    fi

    case "$enable_imu" in
        0)
            test_binary="$HOME/.cache/rtabmap_dense/calib_ab_20260826/zed_rgbd_publisher_calib_test"
            test_binary_expected_sha256="$legacy_binary_sha256"
            ;;
        1)
            test_binary="$imu_candidate_binary"
            test_binary_expected_sha256="$imu_candidate_sha256"
            ;;
        *)
            echo "RTAB_ENABLE_IMU 只支持 0（回退基线）或 1（默认IMU单因素候选）: $enable_imu" >&2
            return 1
            ;;
    esac
}

print_imu_trial_config() {
    printf 'imu_trial_enabled=%s\n' "$enable_imu"
    printf 'odom_f2m_max_size=2000\n'
    printf 'odom_f2m_max_size_reason=single_factor_field_trial_after_loss_containment;previous_2000_trial_was_confounded\n'
    printf 'test_binary=%s\n' "$test_binary"
    printf 'test_binary_sha256=%s\n' "$test_binary_expected_sha256"
    printf 'imu_candidate_binary=%s\n' "$imu_candidate_binary"
    printf 'imu_candidate_sha256=%s\n' "$imu_candidate_sha256"
}

landmark_time_guard_python() {
    if [[ -n "${LD_PRELOAD:-}" || -n "${LD_AUDIT:-}" ]]; then
        echo '[landmark time guard rejected] nonempty LD_PRELOAD/LD_AUDIT is forbidden' >&2
        return 1
    fi
    if [[ ! "$landmark_time_guard_sha256" =~ ^[0-9a-f]{64}$ ]]; then
        echo '[landmark time guard rejected] private SO hash is PENDING/unfrozen' >&2
        return 1
    fi
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -I - \
        "$landmark_time_guard_so" "$landmark_time_guard_sha256" \
        "$reset_runtime_root" "$@" <<'LANDMARK_TIME_GUARD_PY'
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import struct
import sys

PROC_ROOT = Path('/proc')
EXPECTED_UID = 1000
LIBRARY_NAME = 'librtabmap_conversions.so'
MAX_LIBRARY_BYTES = 512 * 1024 * 1024


class GuardError(ValueError):
    pass


def require(condition, reason):
    if not condition:
        raise GuardError(reason)


def device_pair(device):
    return os.major(device), os.minor(device)


def identity(info):
    major, minor = device_pair(info.st_dev)
    return {'device': info.st_dev, 'device_major': major, 'device_minor': minor,
            'inode': info.st_ino, 'bytes': info.st_size, 'uid': info.st_uid,
            'mode': info.st_mode, 'links': info.st_nlink,
            'mtime_ns': info.st_mtime_ns, 'ctime_ns': info.st_ctime_ns}


def lstat(path):
    return path.lstat()


def regular_info(path):
    info = lstat(path)
    require(stat.S_ISREG(info.st_mode), 'private SO must be a regular non-symlink file')
    require(info.st_uid == EXPECTED_UID and info.st_nlink == 1,
            'private SO owner or hard-link count mismatch')
    require(not info.st_mode & 0o022, 'private SO is group/other writable')
    require(64 <= info.st_size <= MAX_LIBRARY_BYTES, 'private SO size is invalid')
    return info


def parent_chain(path, private_root):
    observed = []
    for parent in reversed(path.parents):
        info = lstat(parent)
        require(stat.S_ISDIR(info.st_mode), 'private SO ancestor is not a non-symlink directory')
        if parent == private_root or parent == path.parent:
            require(info.st_uid == EXPECTED_UID and not info.st_mode & 0o022,
                    'private SO directory owner/mode mismatch')
        observed.append((str(parent), info.st_dev, info.st_ino, info.st_uid, info.st_mode))
    return observed


def elf_header(header, size):
    require(len(header) == 64 and header[:7] == b'\x7fELF\x02\x01\x01',
            'private SO is not ELF64 little-endian version 1')
    fields = struct.unpack('<HHIQQQIHHHHHH', header[16:64])
    kind, machine, version, _, phoff, _, _, ehsize, phentsize, phnum, _, _, _ = fields
    require(kind == 3 and machine == 183 and version == 1 and ehsize == 64,
            'private SO must be an ELF64 aarch64 shared object')
    require(phoff >= 64 and phentsize == 56 and phnum > 0 and
            phoff + phentsize * phnum <= size, 'private SO program header table is invalid')
    return {'class': 64, 'endianness': 'little', 'machine': 'aarch64', 'type': 'ET_DYN'}


def check_library(raw_path, expected_hash, private_root, expected_receipt=None):
    require(not os.environ.get('LD_PRELOAD', '') and not os.environ.get('LD_AUDIT', ''),
            'nonempty LD_PRELOAD/LD_AUDIT is forbidden')
    require(isinstance(expected_hash, str) and re.fullmatch('[0-9a-f]{64}', expected_hash),
            'private SO hash is PENDING/unfrozen')
    path = Path(raw_path)
    require(private_root.is_absolute() and path.is_absolute() and
            str(path) == str(private_root / 'lib' / LIBRARY_NAME),
            'private SO path is not the exact frozen private-runtime path')
    parents = parent_chain(path, private_root)
    before = regular_info(path)
    fingerprint = identity(before)
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0)
    digest = hashlib.sha256()
    with os.fdopen(os.open(path, flags), 'rb') as source:
        require(identity(os.fstat(source.fileno())) == fingerprint, 'private SO identity changed on open')
        header = source.read(64)
        digest.update(header)
        layout = elf_header(header, before.st_size)
        total = len(header)
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            total += len(chunk)
            require(total <= before.st_size, 'private SO grew during hashing')
            digest.update(chunk)
        require(total == before.st_size, 'private SO truncated during hashing')
        require(identity(os.fstat(source.fileno())) == fingerprint, 'private SO changed during hashing')
    require(identity(regular_info(path)) == fingerprint and parent_chain(path, private_root) == parents,
            'private SO pathname/ancestor identity changed during hashing')
    require(digest.hexdigest() == expected_hash, 'private SO SHA256 mismatch')
    receipt = {'schema_version': 1, 'path': str(path), 'sha256': expected_hash,
               'identity': fingerprint, 'elf': layout, 'runtime_mapping_verified': False}
    if expected_receipt is not None:
        require(receipt == expected_receipt, 'private SO changed since the preflight identity receipt')
    return receipt


def process_identity(pid, expected_ticks):
    require(isinstance(pid, str) and re.fullmatch('[1-9][0-9]*', pid), 'invalid SLAM PID')
    require(isinstance(expected_ticks, str) and re.fullmatch('[1-9][0-9]*', expected_ticks),
            'missing/invalid recorded SLAM start ticks')
    directory = PROC_ROOT / pid
    info = lstat(directory)
    require(stat.S_ISDIR(info.st_mode) and info.st_uid == EXPECTED_UID,
            'SLAM process owner/instance is unverified')
    with (directory / 'stat').open('r', encoding='ascii') as stream:
        record = stream.read(65537)
    require(len(record) <= 65536 and record.startswith(pid + ' ('), 'malformed SLAM stat')
    closing = record.rfind(') ')
    require(closing > 0, 'malformed SLAM stat comm')
    fields = record[closing + 2:].split()
    require(len(fields) >= 20 and fields[0] not in ('Z', 'X', 'x') and
            fields[19] == expected_ticks, 'SLAM PID exited or recorded start ticks changed')
    return {'pid': int(pid), 'start_ticks': expected_ticks, 'uid': info.st_uid}


def verify_mapped(raw_path, expected_hash, private_root, pid, expected_ticks, receipt):
    before = process_identity(pid, expected_ticks)
    library = check_library(raw_path, expected_hash, private_root, receipt)
    with (PROC_ROOT / pid / 'maps').open('r', encoding='utf-8', errors='surrogateescape') as stream:
        text = stream.read(4 * 1024 * 1024 + 1)
    require(len(text) <= 4 * 1024 * 1024, 'SLAM maps exceeds audit limit')
    matched = []
    for line in text.splitlines():
        fields = line.split(None, 5)
        if len(fields) != 6:
            continue
        address, permissions, offset, device, inode, pathname = fields
        basename = Path(pathname.removesuffix(' (deleted)')).name
        if not basename.startswith(LIBRARY_NAME):
            continue
        require(pathname == library['path'], 'SLAM mapped a different/deleted conversion library path')
        require(re.fullmatch('[0-9a-fA-F]+-[0-9a-fA-F]+', address) and
                re.fullmatch('[r-][w-][x-][ps]', permissions) and
                re.fullmatch('[0-9a-fA-F]+', offset) and
                re.fullmatch('[0-9a-fA-F]+:[0-9a-fA-F]+', device) and
                re.fullmatch('[0-9]+', inode), 'malformed conversion library maps record')
        low, high = (int(value, 16) for value in address.split('-'))
        major, minor = (int(value, 16) for value in device.split(':'))
        expected = library['identity']
        require(low < high and int(inode) == expected['inode'] and
                (major, minor) == (expected['device_major'], expected['device_minor']),
                'SLAM conversion library inode/device mismatch')
        matched.append({'address': address, 'permissions': permissions, 'offset': offset,
                        'device': device, 'inode': int(inode), 'path': pathname})
    require(matched and any('x' in item['permissions'] for item in matched) and
            any(int(item['offset'], 16) == 0 for item in matched),
            'SLAM has no complete executable mapping of the expected private conversion library')
    require(process_identity(pid, expected_ticks) == before, 'SLAM instance changed during maps audit')
    check_library(raw_path, expected_hash, private_root, receipt)
    require(process_identity(pid, expected_ticks) == before, 'SLAM instance changed before audit completion')
    return dict(library, runtime_mapping_verified=True, process=before, mapped_segments=matched,
                scope='slam_process_only', tf_failure_policy='reject_detection',
                observation_ttl='unchanged_not_guarded', reset_epoch='unchanged_not_guarded',
                continuing_mapping_or_physical_accuracy_validated=False)


def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise GuardError('duplicate JSON key: ' + str(key))
        result[key] = value
    return result


def main(arguments):
    require(len(arguments) >= 4, 'usage: FROZEN_SO SHA256 PRIVATE_ROOT check|mapped ...')
    raw_path, expected_hash, private_root_raw, mode = arguments[:4]
    private_root = Path(private_root_raw)
    require(sys.platform.startswith('linux'), 'private SO audit requires Linux; no library is loaded by this check')
    if mode == 'check' and len(arguments) == 4:
        result = check_library(raw_path, expected_hash, private_root)
    elif mode == 'mapped' and len(arguments) == 7:
        result = verify_mapped(
            raw_path, expected_hash, private_root, arguments[4], arguments[5],
            json.loads(arguments[6], object_pairs_hook=reject_duplicate_pairs))
    else:
        raise GuardError('invalid private SO audit arguments')
    print(json.dumps(result, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except (GuardError, OSError, ValueError, TypeError, KeyError, struct.error) as error:
        print('[landmark time guard rejected] ' + str(error), file=sys.stderr)
        sys.exit(1)
LANDMARK_TIME_GUARD_PY
}

landmark_time_guard_check() {
    landmark_time_guard_python check
}

landmark_time_guard_mapped() {
    landmark_time_guard_python mapped "$1" "$2" "$3"
}
# END PURE_LANDMARK_TIME_GUARD

capture_camera_profile
resolve_camera_profile || exit 1
capture_trial_ranges
validate_trial_ranges
capture_imu_trial_config || exit 1
if (($#)); then
    if (($# == 1)) && [[ "$1" == --print-camera-profile ]]; then
        print_camera_profile
        exit 0
    fi
    if (($# == 1)) && [[ "$1" == --check-camera-profile ]]; then
        check_camera_profile_dependencies
        exit $?
    fi
    if (($# == 1)) && [[ "$1" == --print-range-config ]]; then
        print_trial_ranges
        exit 0
    fi
    if (($# == 1)) && [[ "$1" == --check-landmark-time-guard ]]; then
        landmark_time_guard_check
        exit $?
    fi
    if (($# == 1)) && [[ "$1" == --check-reset-quarantine-runtime ]]; then
        check_reset_quarantine_runtime
        exit $?
    fi
    if (($# == 1)) && [[ "$1" == --print-imu-config ]]; then
        print_imu_trial_config
        exit 0
    fi
    echo "用法: $0 [--print-camera-profile|--check-camera-profile|--print-range-config|--print-imu-config|--check-landmark-time-guard|--check-reset-quarantine-runtime]" >&2
    exit 2
fi
# The read-only modes above never load ROS/SDK/global config, create a state
# directory, takes a runtime lock, or starts a process which controls hardware.
camera_profile_preflight_receipt="$(check_camera_profile_dependencies)" || exit 1
readonly camera_profile_preflight_receipt
landmark_time_guard_receipt="$(landmark_time_guard_check)" || exit 1
readonly landmark_time_guard_receipt
reset_runtime_receipt="$(check_reset_quarantine_runtime)" || exit 1
readonly reset_runtime_receipt
source /opt/ros/humble/setup.bash
wbrov_enable_dds_profile || exit 1
set -u

bundle_dir="/home/exocean/Desktop/RTABMap_稠密建图"
profile="$caller_profile"
candidate="$caller_calibration_file"
light_mode="$caller_light_mode"
teleop_pid=""
case "$light_mode" in
    teleop|off) ;;
    *)
        echo "CALIB_AB_LIGHT_MODE 只支持 teleop 或 off: $light_mode"
        exit 1
        ;;
esac
if [[ -n "$caller_light_hook" ]]; then
    echo "CALIB_AB_LIGHT_HOOK 已因外设整帧覆盖风险禁用；请使用 teleop 或 off 模式。"
    exit 1
fi
case "$profile" in
    factory)
        unset ZED_OPENCV_CALIBRATION_FILE
        ;;
    custom)
        if [[ ! -r "$candidate" ]]; then
            echo "水下候选标定文件不存在或不可读: $candidate"
            exit 1
        fi
        export ZED_OPENCV_CALIBRATION_FILE="$candidate"
        ;;
    *)
        echo "CALIB_AB_PROFILE 只支持 factory 或 custom"
        exit 1
        ;;
esac
if [[ ! -r "$test_rviz" ]]; then
    echo "图优化点云 RViz 配置不存在或不可读: $test_rviz"
    exit 1
fi
for graph_display_file in "$graph_display_script" "$graph_display_module" "$submap_display_module"; do
    if [[ ! -f "$graph_display_file" || ! -r "$graph_display_file" ]]; then
        echo "图结构显示文件不存在或不可读: $graph_display_file" >&2
        exit 1
    fi
done
# Reproduce the small, reviewed environment setup inline.  Do not execute a
# mutable desktop helper or any legacy environment/configuration file.
rtab_root="$HOME/opt/rtabmap"
rtab_prefix="$rtab_root/opt/ros/humble"
[[ -d "$rtab_prefix" ]] || {
    echo "找不到用户目录 RTAB-Map: $rtab_prefix" >&2
    exit 1
}
readonly managed_runtime_path="$rtab_prefix/bin:$managed_system_path"
readonly managed_runtime_ament_prefix_path="$rtab_prefix:/opt/ros/humble"
readonly managed_runtime_cmake_prefix_path="$rtab_prefix:/opt/ros/humble"
readonly managed_runtime_pythonpath="$private_runtime_root/local/lib/python3.10/dist-packages:$private_runtime_root/lib/python3.10/site-packages:$rtab_prefix/local/lib/python3.10/dist-packages:$rtab_prefix/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages"
# Core and odometry must resolve one immutable RTAB-Map lineage.  The private
# runtime contains every reached RTAB-Map/user-prefix library, including the
# reviewed time-guard conversions build.  Production RTAB-Map prefixes are
# deliberately absent so a mixed ABI cannot be created by loader fallback.
readonly managed_runtime_ld_library_path="$reset_runtime_library_directory:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/usr/local/zed/lib:/usr/local/cuda/lib64"
export PATH="$managed_runtime_path"
export AMENT_PREFIX_PATH="$managed_runtime_ament_prefix_path"
export CMAKE_PREFIX_PATH="$managed_runtime_cmake_prefix_path"
export COLCON_PREFIX_PATH=''
export PYTHONPATH="$managed_runtime_pythonpath"
export LD_LIBRARY_PATH="$managed_runtime_ld_library_path"
if ! PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" \
    "$apriltag_detector" --self-test; then
    echo "AprilTag 检测器只读自测失败，尚未启动相机或建图节点。" >&2
    exit 1
fi
# Import/syntax/message-construction check only: no ROS context, DDS or SDK.
if ! PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" \
    "$graph_display_script" --check-only; then
    echo "图结构显示只读预检失败，尚未启动本次相机或建图节点。" >&2
    exit 1
fi

# Trial ranges were captured before any sourced environment and are restored
# below.  Runtime depth mode/rate are fixed later in this launcher.
declare -a odom_camera_profile_args=()
declare -a odom_transport_args=()
declare -a apriltag_camera_profile_args=()
if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    # Process atomic RGB-D inline; KEEP_LAST(1) retains only the newest waiting
    # sample. The asynchronous try-lock drops even fresh 33/67 ms burst inputs
    # while the previous frame is computing. Keep the DDS backlog bounded.
    # Acceptance is still the measured valid /odom rate (>=15 Hz), not input FPS.
    odom_camera_profile_args=(-p always_process_most_recent_frame:=false -p topic_queue_size:=1)
    # Carry RGB, depth and both CameraInfo objects in one DDS sample. Raw
    # topics stay available for RViz, depth display, AprilTag and diagnostics.
    odom_transport_args=(
        -r "rgbd_image:=$odom_rgbd_topic"
        -p subscribe_rgbd:=true
    )
    # The tag solver uses the sparse 960x600 map stream.  Absolute source names
    # in the existing Python node are remapped without changing that file.
    apriltag_camera_profile_args=(
        --ros-args
        -r "/zed/rgb/image_rect_color:=$map_rgb_topic"
        -r "/zed/rgb/camera_info:=$map_info_topic"
    )
else
    odom_transport_args=(
        -r "rgb/image:=$odom_rgb_topic"
        -r "depth/image:=$odom_depth_topic"
        -r "rgb/camera_info:=$odom_info_topic"
    )
fi
# Re-apply the caller contract after the root-owned ROS setup.  Runtime
# semantics are frozen below instead of executing the mutable desktop
# AprilTag参数.env as shell code.
resolve_camera_profile || exit 1
capture_imu_trial_config || exit 1
odom_camera_profile_args=()
odom_transport_args=()
apriltag_camera_profile_args=()
if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    odom_camera_profile_args=(-p always_process_most_recent_frame:=false -p topic_queue_size:=1)
    odom_transport_args=(
        -r "rgbd_image:=$odom_rgbd_topic"
        -p subscribe_rgbd:=true
    )
    apriltag_camera_profile_args=(
        --ros-args
        -r "/zed/rgb/image_rect_color:=$map_rgb_topic"
        -r "/zed/rgb/camera_info:=$map_info_topic"
    )
else
    odom_transport_args=(
        -r "rgb/image:=$odom_rgb_topic"
        -r "depth/image:=$odom_depth_topic"
        -r "rgb/camera_info:=$odom_info_topic"
    )
fi
profile="$caller_profile"
candidate="$caller_calibration_file"
light_mode="$caller_light_mode"
case "$profile" in
    factory) unset ZED_OPENCV_CALIBRATION_FILE ;;
    custom)
        [[ -r "$candidate" ]] || {
            echo "水下候选标定文件不存在或不可读: $candidate" >&2
            exit 1
        }
        export ZED_OPENCV_CALIBRATION_FILE="$candidate"
        ;;
    *) echo "CALIB_AB_PROFILE 只支持 factory 或 custom" >&2; exit 1 ;;
esac
case "$light_mode" in
    teleop|off) ;;
    *) echo "CALIB_AB_LIGHT_MODE 只支持 teleop 或 off: $light_mode" >&2; exit 1 ;;
esac
[[ -z "$caller_light_hook" ]] || {
    echo "CALIB_AB_LIGHT_HOOK 已禁用。" >&2
    exit 1
}
readonly profile candidate light_mode
readonly camera_profile camera_topology camera_grab_resolution camera_grab_fps
readonly camera_compute_cap_fps odom_width odom_height odom_publish_hz
readonly odom_effective_health_min_hz map_width map_height map_publish_hz
readonly odom_rgb_topic odom_depth_topic odom_display_depth_topic odom_info_topic
readonly map_rgb_topic map_depth_topic map_info_topic test_binary
readonly test_binary_expected_sha256 enable_imu
readonly -a odom_camera_profile_args odom_transport_args apriltag_camera_profile_args

# Reject loader injection after the final source, then replace the inherited
# search path with a fixed absolute allowlist.  The private runtime checker
# verifies the final node's dependency closure against this exact path.
if [[ -n "${LD_PRELOAD:-}" || -n "${LD_AUDIT:-}" ]]; then
    echo '[启动拒绝] sourced配置设置了LD_PRELOAD或LD_AUDIT。' >&2
    exit 1
fi
export PATH="$managed_runtime_path"
export AMENT_PREFIX_PATH="$managed_runtime_ament_prefix_path"
export CMAKE_PREFIX_PATH="$managed_runtime_cmake_prefix_path"
export COLCON_PREFIX_PATH=''
export PYTHONPATH="$managed_runtime_pythonpath"
export LD_LIBRARY_PATH="$managed_runtime_ld_library_path"
reset_runtime_receipt_after_sources="$(check_reset_quarantine_runtime)" || exit 1
readonly reset_runtime_receipt_after_sources
camera_profile_receipt_after_sources="$(check_camera_profile_dependencies)" || exit 1
readonly camera_profile_receipt_after_sources
landmark_time_guard_receipt_after_sources="$(landmark_time_guard_check)" || exit 1
readonly landmark_time_guard_receipt_after_sources
apply_camera_profile_environment
export RTAB_DEPTH_MODE=NEURAL
if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    # Keep the backend cap strictly above the 1 Hz input.  The high-resolution
    # branch is deliberately sparse so it cannot starve the 15+ Hz VO path.
    export RTAB_DETECTION_RATE=5.0
else
    export RTAB_DETECTION_RATE=5.0
fi
if ! awk -v rate="$RTAB_DETECTION_RATE" \
    'BEGIN {exit !(rate >= 1.0 && rate <= 15.0)}'; then
    echo "RTAB_DETECTION_RATE 必须在 1–15 Hz 之间"
    exit 1
fi
# Fixed only for this isolated launcher and its newly-created run/database.
export APRILTAG_LANDMARK_MODE=position_g2o
export APRILTAG_SIZE_M='0.080'
restore_trial_ranges
validate_trial_ranges
frame_min_depth="$(ros_double_literal "$DENSE_MIN_M")"
frame_max_depth="$(ros_double_literal "$DENSE_MAX_M")"
if ! awk -v size="$APRILTAG_SIZE_M" 'BEGIN {exit !(size > 0 && size <= 2)}'; then
    echo "AprilTag 边长无效: $APRILTAG_SIZE_M m"
    exit 1
fi

verify_production_teleop_light_owner() {
    local expected_pid="${1:-}"
    local current_uid
    local -a candidates=()
    local -a socket_lines=()
    local -a socket_owner_pids=()
    local candidate_pid
    local line

    if ! command -v ss >/dev/null 2>&1; then
        echo "teleop 灯光门控缺少 ss，拒绝启动相机。"
        return 1
    fi
    current_uid="$(id -u)"
    mapfile -t candidates < <(
        pgrep -u "$current_uid" -f '[t]eleop_keyboard\.py' || true
    )
    if ((${#candidates[@]} != 1)); then
        echo "teleop 模式要求恰好一个同用户生产遥控进程；当前数量=${#candidates[@]}。"
        return 1
    fi
    candidate_pid="${candidates[0]}"
    if [[ -n "$expected_pid" && "$candidate_pid" != "$expected_pid" ]]; then
        echo "生产遥控进程在建图启动前发生变化，拒绝启动相机。"
        return 1
    fi
    if [[ ! -r "/proc/$candidate_pid/environ" ]] || \
       ! tr '\0' '\n' < "/proc/$candidate_pid/environ" | grep -x 'DRY_RUN=0' >/dev/null; then
        echo "PID=$candidate_pid 不是可确认的 DRY_RUN=0 生产遥控。"
        return 1
    fi
    mapfile -t socket_lines < <(
        ss -H -u -a -n -p 'sport = :3333' 2>/dev/null || true
    )
    if ((${#socket_lines[@]} == 0)); then
        echo "PID=$candidate_pid 未持有 UDP 3333，不能依赖遥控进程维持灯光。"
        return 1
    fi
    mapfile -t socket_owner_pids < <(
        printf '%s\n' "${socket_lines[@]}" |
            grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u
    )
    if ((${#socket_owner_pids[@]} != 1)) || \
       [[ "${socket_owner_pids[0]:-}" != "$candidate_pid" ]]; then
        echo "UDP 3333 必须仅由生产遥控 PID=$candidate_pid 持有。"
        return 1
    fi
    for line in "${socket_lines[@]}"; do
        if [[ "$line" != *"pid=$candidate_pid,"* ]]; then
            echo "UDP 3333 存在非生产遥控 PID 的持有者，拒绝启动相机。"
            return 1
        fi
    done
    if ! kill -0 "$candidate_pid" 2>/dev/null; then
        echo "生产遥控 PID=$candidate_pid 已退出，拒绝启动相机。"
        return 1
    fi
    teleop_pid="$candidate_pid"
}

state_dir="$HOME/.cache/rtabmap_dense_calib_ab"
production_state_dir="$HOME/.cache/rtabmap_dense"
run_root="$HOME/maps/rtabmap_calibration_ab"

ensure_private_runtime_directory() {
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" - \
        "$HOME" "$1" <<'SECURE_RUNTIME_DIRECTORY_PY'
import os
from pathlib import Path
import stat
import sys

home = Path(sys.argv[1])
target = Path(sys.argv[2])
if any(character in str(target) for character in ("\n", "\r", "\0")):
    raise SystemExit("managed directory contains a forbidden control character")
home_info = home.lstat()
if (not stat.S_ISDIR(home_info.st_mode) or home_info.st_uid != os.getuid() or
        home.is_symlink()):
    raise SystemExit("HOME is not a canonical owned directory")
home_real = home.resolve(strict=True)
absolute = Path(os.path.abspath(str(target)))
try:
    relative = absolute.relative_to(home_real)
except ValueError:
    raise SystemExit("managed directory escapes HOME")
flags = os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW
descriptor = os.open(home_real, flags)
try:
    parts = relative.parts
    if not parts:
        raise SystemExit("managed directory cannot be HOME itself")
    for index, component in enumerate(parts):
        if component in ("", ".", ".."):
            raise SystemExit("managed directory has an unsafe component")
        try:
            os.mkdir(component, 0o700, dir_fd=descriptor)
        except FileExistsError:
            pass
        next_descriptor = os.open(component, flags, dir_fd=descriptor)
        os.close(descriptor)
        descriptor = next_descriptor
        info = os.fstat(descriptor)
        if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid():
            raise SystemExit("managed directory component is not an owned directory")
        if index + 1 == len(parts) and stat.S_IMODE(info.st_mode) != 0o700:
            raise SystemExit("managed state/run root must have mode 0700")
    if Path(os.path.realpath(absolute)) != absolute:
        raise SystemExit("managed directory contains a symlinked ancestor")
finally:
    os.close(descriptor)
SECURE_RUNTIME_DIRECTORY_PY
}

ensure_private_runtime_directory "$state_dir" || exit 1
ensure_private_runtime_directory "$run_root" || exit 1

atomic_state_line() {
    local destination="$1" value="$2" temporary
    [[ "${destination%/*}" == "$state_dir" &&
       "${destination##*/}" =~ ^[a-z_]+(\.pid)?$ ]] || return 1
    temporary="$(mktemp "$state_dir/.${destination##*/}.XXXXXX")" || return 1
    if ! printf '%s\n' "$value" >"$temporary" || ! chmod 600 -- "$temporary" ||
       ! sync -f -- "$temporary" || ! mv -fT -- "$temporary" "$destination" ||
       ! sync -f -- "$state_dir"; then
        rm -f -- "$temporary"
        return 1
    fi
}

read_secure_pid_file() {
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -I - "$1" <<'SECURE_PID_READ_PY'
import os
from pathlib import Path
import stat
import sys

path = Path(sys.argv[1])
if not os.path.lexists(path):
    raise SystemExit(2)
parent = path.parent
parent_info = parent.lstat()
if (not stat.S_ISDIR(parent_info.st_mode) or parent.is_symlink() or
        parent_info.st_uid != os.getuid() or stat.S_IMODE(parent_info.st_mode) & 0o022):
    raise SystemExit(1)
flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
try:
    parent_descriptor = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        descriptor = os.open(path.name, flags, dir_fd=parent_descriptor)
    finally:
        os.close(parent_descriptor)
except OSError:
    raise SystemExit(1)
try:
    info = os.fstat(descriptor)
    if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
            info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o600 or
            info.st_size <= 0 or info.st_size > 32):
        raise SystemExit(1)
    raw = os.read(descriptor, 64)
finally:
    os.close(descriptor)
try:
    text = raw.decode("ascii").strip()
except UnicodeDecodeError:
    raise SystemExit(1)
if not text.isdigit() or int(text) <= 0:
    raise SystemExit(1)
print(text)
SECURE_PID_READ_PY
}

early_process_start_ticks() {
    local record tail
    local -a fields=()
    [[ "$1" =~ ^[0-9]+$ && -r "/proc/$1/stat" ]] || return 1
    IFS= read -r record < "/proc/$1/stat" || return 1
    tail="${record##*) }"
    read -r -a fields <<< "$tail"
    [[ "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 1
    printf '%s\n' "${fields[19]}"
}

# A tiny broker opens the lock with O_NOFOLLOW, validates its inode, takes
# LOCK_EX|LOCK_NB, and holds it until this launcher's pipe closes.  Bash's own
# redirection cannot provide O_NOFOLLOW.
coproc WBROV_LAUNCHER_LOCK_BROKER {
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 exec "$python3_runtime" -c '
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
import tempfile

path = sys.argv[1]
run_root = Path(sys.argv[2])
flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW
descriptor = os.open(path, flags, 0o600)
info = os.fstat(descriptor)
if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
        info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o600):
    raise SystemExit("unsafe launcher lock inode")
try:
    fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit("launcher lock is already held")
print("locked", flush=True)
request = sys.stdin.readline().rstrip("\n")
parts = request.split("\t")
if len(parts) != 2 or parts[0] != "release":
    raise SystemExit("missing terminal lock-release request")
run = Path(parts[1])
root_info = run_root.lstat()
run_info = run.lstat()
if (not stat.S_ISDIR(root_info.st_mode) or run_root.is_symlink() or
        root_info.st_uid != os.getuid() or stat.S_IMODE(root_info.st_mode) & 0o022 or
        not stat.S_ISDIR(run_info.st_mode) or run.is_symlink() or
        run_info.st_uid != os.getuid() or stat.S_IMODE(run_info.st_mode) != 0o700 or
        run.resolve(strict=True).parent != run_root.resolve(strict=True)):
    raise SystemExit("unsafe terminal run directory")
lifecycle_path = run / "lifecycle.json"
lifecycle_flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW
lifecycle_fd = os.open(lifecycle_path, lifecycle_flags)
try:
    lifecycle_info = os.fstat(lifecycle_fd)
    if (not stat.S_ISREG(lifecycle_info.st_mode) or
            lifecycle_info.st_uid != os.getuid() or lifecycle_info.st_nlink != 1 or
            stat.S_IMODE(lifecycle_info.st_mode) != 0o600 or
            lifecycle_info.st_size <= 0 or lifecycle_info.st_size > 65536):
        raise SystemExit("unsafe stopped lifecycle inode")
    lifecycle_bytes = b""
    while True:
        block = os.read(lifecycle_fd, 65536)
        if not block:
            break
        lifecycle_bytes += block
        if len(lifecycle_bytes) > 65536:
            raise SystemExit("stopped lifecycle is too large")
finally:
    os.close(lifecycle_fd)
def reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)

def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + str(key))
        result[key] = value
    return result

lifecycle = json.loads(
    lifecycle_bytes.decode("utf-8"), parse_constant=reject_constant,
    object_pairs_hook=reject_duplicate_pairs)
parent_pid = os.getppid()
stat_record = Path(f"/proc/{parent_pid}/stat").read_text(encoding="ascii")
parent_ticks = int(stat_record.rsplit(") ", 1)[1].split()[19])
boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
if (not isinstance(lifecycle, dict) or
        set(lifecycle) != {"schema_version", "phase", "launcher_pid",
        "process_start_ticks", "boot_id", "nodes_stopped", "export_succeeded",
        "cleanup_succeeded", "exit_code", "requested_exit_code"} or
        type(lifecycle.get("schema_version")) is not int or
        lifecycle.get("schema_version") != 1 or lifecycle.get("phase") != "stopped" or
        type(lifecycle.get("launcher_pid")) is not int or
        lifecycle.get("launcher_pid") != parent_pid or
        type(lifecycle.get("process_start_ticks")) is not int or
        lifecycle.get("process_start_ticks") != parent_ticks or
        lifecycle.get("boot_id") != boot_id or
        lifecycle.get("nodes_stopped") is not True or
        lifecycle.get("export_succeeded") is not True or
        lifecycle.get("cleanup_succeeded") is not True or
        type(lifecycle.get("exit_code")) is not int or
        lifecycle.get("exit_code") != 0 or
        type(lifecycle.get("requested_exit_code")) is not int or
        lifecycle.get("requested_exit_code") not in (0, 129, 130, 143)):
    raise SystemExit("terminal lifecycle is not a successful exact parent instance")
lifecycle_sha = hashlib.sha256(lifecycle_bytes).hexdigest()
receipt = {
    "schema_version": 1,
    "producer": "wbrov_mapping_lock_broker",
    "launcher_pid": parent_pid,
    "process_start_ticks": parent_ticks,
    "boot_id": boot_id,
    "run_directory": run.name,
    "lifecycle_sha256": lifecycle_sha,
}
encoded = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
receipt_path = run / "lock_release.json"
if os.path.lexists(receipt_path):
    raise SystemExit("lock release receipt already exists")
temporary_fd, temporary_name = tempfile.mkstemp(prefix=".lock_release.", dir=str(run))
temporary_path = Path(temporary_name)
try:
    os.fchmod(temporary_fd, 0o600)
    with os.fdopen(temporary_fd, "wb", closefd=True) as stream:
        temporary_fd = -1
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_path, receipt_path)
    temporary_path = None
    run_fd = os.open(run, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        os.fsync(run_fd)
    finally:
        os.close(run_fd)
finally:
    if temporary_fd >= 0:
        os.close(temporary_fd)
    if temporary_path is not None:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
print("released " + hashlib.sha256(encoded).hexdigest(), flush=True)
' "$state_dir/launcher.lock" "$run_root"
}
launcher_lock_broker_pid="$WBROV_LAUNCHER_LOCK_BROKER_PID"
launcher_lock_broker_read_fd="${WBROV_LAUNCHER_LOCK_BROKER[0]}"
launcher_lock_broker_write_fd="${WBROV_LAUNCHER_LOCK_BROKER[1]}"
if ! IFS= read -r -t 3 launcher_lock_state <&"$launcher_lock_broker_read_fd" ||
   [[ "$launcher_lock_state" != locked ]]; then
    echo "隔离建图或上一次保存/清理尚未结束，安全锁获取失败。" >&2
    wait "$launcher_lock_broker_pid" 2>/dev/null || true
    exit 1
fi
# Keep both pipe ends for the terminal release transaction.  The broker will
# verify the durable run-local stopped lifecycle and ACK its atomic receipt
# while it still owns the singleton flock.
readonly launcher_lock_broker_pid
launcher_lock_broker_start_ticks="$(early_process_start_ticks "$launcher_lock_broker_pid")" || {
    echo '无法绑定建图锁broker进程身份。' >&2
    exit 1
}
readonly launcher_lock_broker_start_ticks
lock_broker_alive_early() {
    local actual
    kill -0 "$launcher_lock_broker_pid" 2>/dev/null || return 1
    actual="$(early_process_start_ticks "$launcher_lock_broker_pid")" || return 1
    [[ "$actual" == "$launcher_lock_broker_start_ticks" ]]
}
old_pid_status=0
old_pid="$(read_secure_pid_file "$state_dir/launcher.pid" 2>/dev/null)" || old_pid_status=$?
if ((old_pid_status == 1)); then
    echo '本建图入口的launcher.pid存在但安全校验失败；拒绝忽略。' >&2
    exit 1
elif ((old_pid_status == 0)); then
    if [[ -n "${old_pid:-}" ]] && kill -0 "$old_pid" 2>/dev/null; then
        echo "已有 RTAB-Map 建图在运行，PID=$old_pid"
        exit 1
    fi
fi
production_pid_status=0
production_pid="$(read_secure_pid_file "$production_state_dir/launcher.pid" 2>/dev/null)" || production_pid_status=$?
if ((production_pid_status == 0)); then
    if [[ -n "${production_pid:-}" ]] && kill -0 "$production_pid" 2>/dev/null; then
        echo "正式 RTAB-Map 正在运行，PID=$production_pid"
        exit 1
    fi
elif ((production_pid_status == 1)); then
    echo '正式入口PID文件未满足私有目录契约；继续执行精确进程扫描，不信任该文件。' >&2
fi
if pgrep -f "$HOME/ORB_Dense/MyExample/zed_orb_mapping" >/dev/null; then
    echo "ORB_Dense 正在占用 ZED X，请先停止 ORB 建图。"
    exit 1
fi
if pgrep -f '[c]apture_zedx_charuco|[z]ed_rgbd_publisher|[r]tabmap_slam/rtabmap|[r]tabmap_runtime_candidates/.*/bin/rtabmap|[r]tabmap-export' >/dev/null; then
    echo "相机或另一套建图进程正在运行："
    pgrep -af '[c]apture_zedx_charuco|[z]ed_rgbd_publisher|[r]tabmap_slam/rtabmap|[r]tabmap_runtime_candidates/.*/bin/rtabmap|[r]tabmap-export' || true
    exit 1
fi

readonly safe_relocalization_protocol_version='wbrov_safe_relocalization_three_bridge_tx_v1'
readonly mapping_semantics_version='wbrov_underwater_mapping_semantics_v3_imu_zup'
readonly factory_calibration_file='/usr/local/zed/settings/SN40408468.conf'
sha256_path_value() {
    local value
    value="$(sha256sum -- "$1")" || return 1
    value="${value%% *}"
    [[ "$value" =~ ^[0-9a-f]{64}$ ]] || return 1
    printf '%s\n' "$value"
}
if [[ "$profile" == factory ]]; then
    actual_calibration_path="$factory_calibration_file"
else
    actual_calibration_path="$candidate"
fi
actual_calibration_sha256="$(sha256_path_value "$actual_calibration_path")" || {
    echo "无法读取本次实际标定内容以建立续建契约: $actual_calibration_path" >&2
    exit 1
}
reset_runtime_node_actual_sha256="$(sha256_path_value "$reset_runtime_node")" || {
    echo "无法读取安全事务Core节点以建立续建契约: $reset_runtime_node" >&2
    exit 1
}
launcher_actual_sha256="$(sha256_path_value "${BASH_SOURCE[0]}")" || {
    echo "无法读取本次launcher内容以建立续建契约: ${BASH_SOURCE[0]}" >&2
    exit 1
}
readonly actual_calibration_path actual_calibration_sha256
readonly reset_runtime_node_actual_sha256 launcher_actual_sha256

continuation_contract_tool() {
    local mode="$1" source_run="$2"
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" - \
        "$mode" "$source_run" "$profile" "$actual_calibration_sha256" \
        "$camera_profile" "$RTAB_TRACKING_MIN_M" "$RTAB_TRACKING_MAX_M" \
        "$DENSE_MIN_M" "$DENSE_MAX_M" "$APRILTAG_MIN_RANGE_M" \
        "$APRILTAG_MAX_RANGE_M" "$safe_relocalization_protocol_version" \
        "$mapping_semantics_version" "$enable_imu" "$launcher_actual_sha256" \
        "$test_binary_expected_sha256" "$odom_binary_sha256" \
        "$apriltag_anchor_builder_sha256" "$apriltag_detector_sha256" \
        "$apriltag_keyframe_binder_sha256" "$apriltag_safe_relocalizer_sha256" \
        "$reset_runtime_manifest_sha256" "$reset_runtime_node_actual_sha256" \
        "$landmark_time_guard_sha256" <<'CONTINUATION_CONTRACT_PY'
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
import time

if len(sys.argv) != 25:
    raise SystemExit("invalid continuation contract arguments")
mode = sys.argv[1]
run = Path(sys.argv[2])
profile = sys.argv[3]
calibration_sha256 = sys.argv[4]
camera_profile = sys.argv[5]
range_names = (
    "RTAB_TRACKING_MIN_M", "RTAB_TRACKING_MAX_M", "DENSE_MIN_M",
    "DENSE_MAX_M", "APRILTAG_MIN_RANGE_M", "APRILTAG_MAX_RANGE_M")
try:
    range_values = tuple(float(value) for value in sys.argv[6:12])
except ValueError as error:
    raise SystemExit("invalid continuation range: " + str(error))
if (not all(math.isfinite(value) and value > 0.0 for value in range_values) or
        not all(range_values[index] < range_values[index + 1]
                for index in (0, 2, 4))):
    raise SystemExit("invalid continuation range values")
protocol = sys.argv[12]
mapping_semantics_version = sys.argv[13]
if mapping_semantics_version != "wbrov_underwater_mapping_semantics_v3_imu_zup":
    raise SystemExit("invalid mapping semantics version")
if sys.argv[14] not in ("0", "1"):
    raise SystemExit("invalid IMU semantics value")
enable_imu = int(sys.argv[14])
hash_names = (
    "launcher_sha256", "bridge_binary_sha256", "odom_binary_sha256",
    "apriltag_anchor_builder_sha256", "apriltag_detector_sha256",
    "apriltag_keyframe_binder_sha256", "apriltag_safe_relocalizer_sha256",
    "reset_runtime_manifest_sha256", "reset_runtime_node_sha256",
    "landmark_time_guard_sha256")
hash_values = sys.argv[15:25]
if (re.fullmatch(r"[0-9a-f]{64}", calibration_sha256) is None or
        any(re.fullmatch(r"[0-9a-f]{64}", value) is None
            for value in hash_values)):
    raise SystemExit("invalid continuation contract SHA-256")

expected = {
    "schema_version": 2,
    "profile": profile,
    "calibration_sha256": calibration_sha256,
    "camera_profile": camera_profile,
    "ranges_m": dict(zip(range_names, range_values)),
    "base_T_camera": {
        "translation": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
    },
    "safe_relocalization_protocol_version": protocol,
    "mapping_semantics_version": mapping_semantics_version,
    "enable_imu": enable_imu,
}
expected.update(dict(zip(hash_names, hash_values)))
contract_path = run / "continuation_contract.json"


def require_safe_run_directory():
    info = run.lstat()
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or
            (stat.S_IMODE(info.st_mode) & 0o022)):
        raise SystemExit("unsafe continuation contract run directory")


def strict_equal(actual, wanted, location="contract"):
    if type(actual) is not type(wanted):
        raise SystemExit(location + " has the wrong JSON type")
    if isinstance(wanted, dict):
        if set(actual) != set(wanted):
            raise SystemExit(location + " has the wrong field set")
        for key in wanted:
            strict_equal(actual[key], wanted[key], location + "." + key)
    elif isinstance(wanted, list):
        if len(actual) != len(wanted):
            raise SystemExit(location + " has the wrong list length")
        for index, (item, wanted_item) in enumerate(zip(actual, wanted)):
            strict_equal(item, wanted_item, location + "[" + str(index) + "]")
    elif actual != wanted:
        raise SystemExit(location + " does not match the current invocation")


def reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)


def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + str(key))
        result[key] = value
    return result


def read_contract():
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(contract_path, flags)
    try:
        info = os.fstat(descriptor)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
                info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o600 or
                info.st_size <= 0 or info.st_size > 64 * 1024):
            raise SystemExit("unsafe continuation contract inode")
        with os.fdopen(descriptor, "r", encoding="utf-8", closefd=False) as stream:
            payload = json.load(
                stream, parse_constant=reject_constant,
                object_pairs_hook=reject_duplicate_pairs)
    finally:
        os.close(descriptor)
    if not isinstance(payload, dict):
        raise SystemExit("continuation contract must be a JSON object")
    if set(payload) != set(expected) | {"created_unix_ns"}:
        raise SystemExit("continuation contract has the wrong field set")
    created_unix_ns = payload.pop("created_unix_ns")
    if (type(created_unix_ns) is not int or created_unix_ns <= 0 or
            created_unix_ns > 9223372036854775807):
        raise SystemExit("invalid continuation contract created_unix_ns")
    strict_equal(payload, expected)
    return created_unix_ns


if mode == "validate":
    require_safe_run_directory()
    print(read_contract())
elif mode == "publish":
    require_safe_run_directory()
    if os.path.lexists(contract_path):
        raise SystemExit("continuation contract already exists")
    created_unix_ns = time.time_ns()
    payload = dict(expected)
    payload["created_unix_ns"] = created_unix_ns
    encoded = (json.dumps(payload, sort_keys=True, separators=(",", ":"),
                          allow_nan=False) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".continuation_contract.", dir=str(run))
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            descriptor = -1
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, contract_path)
        temporary_path = None
        directory_descriptor = os.open(
            run, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
            getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0))
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        if read_contract() != created_unix_ns:
            raise SystemExit("published continuation contract changed")
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
    print(created_unix_ns)
else:
    raise SystemExit("continuation contract mode must be publish or validate")
CONTINUATION_CONTRACT_PY
}

continuation_request="$caller_continuation_request"
continuation_source_run=""
continuation_source_database=""
continuation_source_anchors=""
continuation_database_sha256=""
continuation_anchors_sha256=""
continuation_source_created_unix_ns=""

validate_continuation_candidate() {
    local source_run="$1"
    local contract_created_unix_ns artifact_receipt
    contract_created_unix_ns="$(continuation_contract_tool validate "$source_run")" || return 1
    artifact_receipt="$(PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" - \
        "$run_root" "$source_run" <<'CONTINUATION_VALIDATE_PY'
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys

def reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)

def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + str(key))
        result[key] = value
    return result

raw_root = Path(sys.argv[1])
raw_run = Path(sys.argv[2])
if any(character in str(raw_run) for character in ("\n", "\r", "\0")):
    raise SystemExit("continuation run path contains a forbidden control character")
for path, label in ((raw_root, "managed run root"), (raw_run, "continuation run")):
    info = path.lstat()
    if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or
            (stat.S_IMODE(info.st_mode) & 0o022)):
        raise SystemExit(label + " is a symlink, is not an owned directory, or is writable by another user")
root = raw_root.resolve(strict=True)
run = raw_run.resolve(strict=True)
if (Path(os.path.abspath(str(raw_root))) != root or
        Path(os.path.abspath(str(raw_run))) != run):
    raise SystemExit("continuation path contains a symlinked ancestor or non-canonical component")
if run.parent != root:
    raise SystemExit("continuation run must be one direct child of the managed run root")
database = run / "rtabmap.db"
anchors = run / "apriltag_multiview_anchors.json"
lifecycle_path = run / "lifecycle.json"
lock_release_path = run / "lock_release.json"
for path in (database, anchors, lifecycle_path, lock_release_path):
    info = path.lstat()
    if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
            info.st_nlink != 1 or info.st_size <= 0 or
            (stat.S_IMODE(info.st_mode) & 0o022)):
        raise SystemExit("unsafe continuation artifact: " + str(path))
info = lock_release_path.lstat()
if stat.S_IMODE(info.st_mode) != 0o600:
    raise SystemExit("lock release receipt must have mode 0600")
for suffix in ("-wal", "-shm", "-journal"):
    if os.path.lexists(str(database) + suffix):
        raise SystemExit("continuation database has a live/recovery sidecar: " + suffix)
digest = hashlib.sha256()
with database.open("rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(block)
database_sha = digest.hexdigest()
anchor_digest = hashlib.sha256()
with anchors.open("rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        anchor_digest.update(block)
anchors_sha = anchor_digest.hexdigest()
lifecycle_bytes = lifecycle_path.read_bytes()
lifecycle = json.loads(
    lifecycle_bytes.decode("utf-8"),
    parse_constant=reject_constant, object_pairs_hook=reject_duplicate_pairs)
if (type(lifecycle.get("schema_version")) is not int or
        lifecycle.get("schema_version") != 1 or
        type(lifecycle.get("launcher_pid")) is not int or
        lifecycle.get("launcher_pid") <= 0 or
        type(lifecycle.get("process_start_ticks")) is not int or
        lifecycle.get("process_start_ticks") <= 0 or
        not isinstance(lifecycle.get("boot_id"), str) or
        re.fullmatch(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}",
                     lifecycle["boot_id"]) is None or
        lifecycle.get("phase") != "stopped" or
        lifecycle.get("nodes_stopped") is not True or
        lifecycle.get("export_succeeded") is not True or
        lifecycle.get("cleanup_succeeded") is not True or
        type(lifecycle.get("exit_code")) is not int or
        lifecycle.get("exit_code") != 0 or
        type(lifecycle.get("requested_exit_code")) is not int or
        lifecycle.get("requested_exit_code") not in (0, 129, 130, 143)):
    raise SystemExit("continuation source does not have a complete stopped lifecycle")
release = json.loads(
    lock_release_path.read_text(encoding="utf-8"),
    parse_constant=reject_constant, object_pairs_hook=reject_duplicate_pairs)
if (set(release) != {"schema_version", "producer", "launcher_pid",
        "process_start_ticks", "boot_id", "run_directory",
        "lifecycle_sha256"} or
        release.get("schema_version") != 1 or
        release.get("producer") != "wbrov_mapping_lock_broker" or
        release.get("launcher_pid") != lifecycle["launcher_pid"] or
        release.get("process_start_ticks") != lifecycle["process_start_ticks"] or
        release.get("boot_id") != lifecycle["boot_id"] or
        release.get("run_directory") != run.name or
        release.get("lifecycle_sha256") != hashlib.sha256(lifecycle_bytes).hexdigest()):
    raise SystemExit("continuation source lacks an exact lock-held terminal receipt")
payload = json.loads(
    anchors.read_text(encoding="utf-8"),
    parse_constant=reject_constant, object_pairs_hook=reject_duplicate_pairs)
schema = payload.get("schema_version")
policy = payload.get("policy")
kind = payload.get("candidate_source_kind")
valid_kind = (
    (schema == 3 and kind == "runtime_exact_keyframe_bindings_jsonl") or
    (schema == 4 and kind in (
        "runtime_exact_keyframe_bindings_jsonl",
        "previous_exact_anchors_only",
        "previous_exact_anchors_plus_runtime_exact_keyframe_bindings_jsonl"))
)
if (schema not in (3, 4) or not valid_kind or
        payload.get("producer") != "wbrov_apriltag_multiview_anchor_builder" or
        payload.get("source_database_sha256") != database_sha or
        payload.get("map_frame") != "map" or
        payload.get("base_frame") != "base_link" or
        payload.get("camera_frame") != "zed_left_camera_optical_frame" or
        not isinstance(policy, dict) or
        policy.get("runtime_exact_keyframe_binding_required") is not True or
        policy.get("required_distinct_current_frames") != 3 or
        not isinstance(payload.get("tags"), dict) or not payload["tags"]):
    raise SystemExit("anchor library is empty or is not bound to this exact stopped database")
if schema == 4:
    lineage = payload.get("lineage")
    if not isinstance(lineage, dict):
        raise SystemExit("schema 4 anchor lineage is missing")
    if kind == "runtime_exact_keyframe_bindings_jsonl":
        if (lineage.get("mode") != "new" or
                lineage.get("parent_schema_version") is not None or
                lineage.get("parent_database_sha256") is not None or
                lineage.get("parent_anchor_sha256") is not None or
                lineage.get("all_parent_nodes_present") is not None or
                lineage.get("parent_anchor_mutated") is not None):
            raise SystemExit("new schema 4 anchor lineage is not empty")
    elif (lineage.get("mode") != "continued" or
            type(lineage.get("parent_schema_version")) is not int or
            lineage.get("parent_schema_version") not in (3, 4) or
            not isinstance(lineage.get("parent_database_sha256"), str) or
            re.fullmatch(r"[0-9a-f]{64}", lineage["parent_database_sha256"]) is None or
            not isinstance(lineage.get("parent_anchor_sha256"), str) or
            re.fullmatch(r"[0-9a-f]{64}", lineage["parent_anchor_sha256"]) is None or
            lineage.get("all_parent_nodes_present") is not True or
            lineage.get("parent_anchor_mutated") is not False):
        raise SystemExit("continued schema 4 anchor lineage is unsafe")
print(str(run))
print(str(database))
print(str(anchors))
print(database_sha)
print(anchors_sha)
CONTINUATION_VALIDATE_PY
    )" || return 1
    printf '%s\n%s\n' "$artifact_receipt" "$contract_created_unix_ns"
}

select_continuation_source() {
    local continuation_candidate_dir="" receipt="" candidate_receipt=""
    local candidate_created_unix_ns="" latest_created_unix_ns=-1
    local -a rows=() candidate_rows=()
    case "$continuation_request" in
        new)
            return 0
            ;;
        latest)
            while IFS= read -r -d '' continuation_candidate_dir; do
                [[ -n "$continuation_candidate_dir" ]] || continue
                if candidate_receipt="$(validate_continuation_candidate "$continuation_candidate_dir" 2>/dev/null)"; then
                    mapfile -t candidate_rows <<<"$candidate_receipt"
                    ((${#candidate_rows[@]} == 6)) || continue
                    candidate_created_unix_ns="${candidate_rows[5]}"
                    [[ "$candidate_created_unix_ns" =~ ^[1-9][0-9]*$ ]] || continue
                    if ((candidate_created_unix_ns > latest_created_unix_ns)); then
                        latest_created_unix_ns="$candidate_created_unix_ns"
                        receipt="$candidate_receipt"
                    fi
                fi
                candidate_receipt=""
                candidate_rows=()
            done < <(find "$run_root" -mindepth 1 -maxdepth 1 -type d -print0)
            [[ -n "$receipt" ]] || {
                echo "没有找到契约兼容、含精确AprilTag多视角锚点且已停止的旧地图；请先完成并正常退出一次新建图。" >&2
                return 1
            }
            ;;
        /*)
            receipt="$(validate_continuation_candidate "$continuation_request")" || return 1
            ;;
        *)
            echo "CALIB_AB_CONTINUE_RUN_DIR只接受 new、latest 或受管run目录的绝对路径。" >&2
            return 1
            ;;
    esac
    mapfile -t rows <<<"$receipt"
    ((${#rows[@]} == 6)) || return 1
    continuation_source_run="${rows[0]}"
    continuation_source_database="${rows[1]}"
    continuation_source_anchors="${rows[2]}"
    continuation_database_sha256="${rows[3]}"
    continuation_anchors_sha256="${rows[4]}"
    continuation_source_created_unix_ns="${rows[5]}"
}

select_continuation_source || exit 1
if [[ "$light_mode" == teleop ]]; then
    verify_production_teleop_light_owner
fi

odom_imu_args=()
slam_imu_args=()
slam_gravity_args=(-p "Mem/UseOdomGravity:='false'")
if [[ "$enable_imu" == 1 ]]; then
    odom_imu_args=(
        -r imu:=/zed/imu/data
		-p managed_expected_imu_node:=zed_rgbd_dual_stream_publisher
		-p managed_expected_imu_namespace:=/
		-p managed_expected_imu_frame:=zed_left_camera_optical_frame
		-p "managed_expected_imu_to_base:='0 0 0 -0.5 0.5 -0.5 0.5'"
        -p wait_imu_to_init:=true
        -p always_check_imu_tf:=false
        -p imu_queue_size:=200
        -p qos_imu:=2
        -p "OdomF2M/BundleAdjustment:='1'"
        -p "OdomF2M/BundleAdjustmentMaxFrames:='3'"
        -p "Optimizer/Iterations:='5'"
        -p "Optimizer/GravitySigma:='0.3'"
    )
    slam_imu_args=(
		-r imu:=/zed/imu/data
		-p reset_quarantine_expected_imu_node:=zed_rgbd_dual_stream_publisher
		-p reset_quarantine_expected_imu_namespace:=/
		-p reset_quarantine_expected_imu_frame:=zed_left_camera_optical_frame
	)
    slam_gravity_args=(
        -p "Mem/UseOdomGravity:='false'"
        -p "Optimizer/GravitySigma:='0.3'"
    )
fi

printf -v run_stamp '%(%Y%m%d_%H%M%S)T' -1
run_dir="$run_root/${profile}_$run_stamp"
if ! mkdir -m 700 -- "$run_dir"; then
    echo "[启动失败] 运行目录已存在或不可安全创建: $run_dir" >&2
    exit 1
fi
# This frozen Core contains optimizer trace instrumentation. It requires an
# existing writable directory before processing the first graph, not just at
# process startup. Keep each Core user separate to avoid trace-name collisions.
readonly optimizer_trace_root="$run_dir/optimizer_trace"
if ! mkdir -m 700 -- "$optimizer_trace_root" \
    "$optimizer_trace_root/odom" "$optimizer_trace_root/slam" "$optimizer_trace_root/export"; then
    echo '[启动失败] 无法创建本次优化器诊断目录；尚未启动设备节点。' >&2
    exit 1
fi
prepare_critical_script_runtime || {
    echo '[启动失败] 无法建立本次只读关键Python运行副本；尚未启动设备节点。' >&2
    exit 1
}
prepare_exporter_runtime || {
    echo '[启动失败] 无法建立本次冻结导出器副本；尚未启动设备节点。' >&2
    exit 1
}
prepare_visualization_runtime "$run_dir" || {
    echo '[启动失败] 无法建立本次只读显示资源副本；尚未启动设备节点。' >&2
    exit 1
}
run_graph_display_script="$run_dir/visualization_runtime/rtabmap_graph_display.py"
run_graph_display_module="$run_dir/visualization_runtime/graph_display_geometry.py"
run_rviz_config="$run_dir/visualization_runtime/rtabmap_dense_graph_optimized.rviz"
run_submap_display_module="$run_dir/visualization_runtime/submap_display_geometry.py"
readonly run_graph_display_script run_graph_display_module run_submap_display_module run_rviz_config
# Validate the actual copied import layout before starting any device node.
if ! PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -B \
    "$run_graph_display_script" --check-only; then
    echo "[启动失败] 本次显示运行副本依赖检查失败；尚未启动设备节点。" >&2
    exit 1
fi
database="$run_dir/rtabmap.db"
continuation_enabled=false
[[ -n "$continuation_source_run" ]] && continuation_enabled=true
readonly safe_relocalization_new_database_sha256='0000000000000000000000000000000000000000000000000000000000000000'
safe_relocalization_database_sha256="$safe_relocalization_new_database_sha256"
safe_relocalization_start_new_map_on_first_update=false
if [[ "$continuation_enabled" == true ]]; then
    safe_relocalization_database_sha256="$continuation_database_sha256"
    safe_relocalization_start_new_map_on_first_update=true
fi
readonly safe_relocalization_database_sha256
readonly safe_relocalization_start_new_map_on_first_update
safe_relocalization_session_token="$(PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
    "$python3_runtime" -c 'import secrets; print(secrets.token_hex(32))')"
if [[ ! "$safe_relocalization_session_token" =~ ^[0-9a-f]{64}$ ]]; then
    echo '[启动失败] 无法生成安全重定位会话标识。' >&2
    exit 1
fi
safe_relocalization_session_token_sha256="$(printf '%s' "$safe_relocalization_session_token" | sha256sum | awk '{print $1}')"
atomic_state_line "$state_dir/launcher.pid" "$$" || exit 1
atomic_state_line "$state_dir/run_dir" "$run_dir" || exit 1
{
    printf 'profile=%s\n' "$profile"
    printf 'optimizer_trace_root=%s\noptimizer_trace_scope=per_run_per_process\n' "$optimizer_trace_root"
    printf 'continuation_requested=%s\n' "$continuation_request"
    printf 'continuation_enabled=%s\n' "$continuation_enabled"
    printf 'continuation_source_run=%s\n' "$continuation_source_run"
    printf 'continuation_source_database_sha256=%s\n' "$continuation_database_sha256"
    printf 'continuation_source_anchors_sha256=%s\n' "$continuation_anchors_sha256"
    printf 'safe_relocalization_session_token_sha256=%s\n' "$safe_relocalization_session_token_sha256"
    printf 'safe_relocalization_database_sha256=%s\n' "$safe_relocalization_database_sha256"
    printf 'safe_relocalization_start_new_map_on_first_update=%s\n' "$safe_relocalization_start_new_map_on_first_update"
    printf 'calibration_file=%s\n' "${ZED_OPENCV_CALIBRATION_FILE:-factory}"
    print_camera_profile
    printf 'camera_profile_field_effect_validated=false\n'
    printf 'camera_profile_rate_evidence=bridge_CAMERA_RATE_schema1_is_publisher_only;managed_observer_enforced_for_dual\n'
    printf 'camera_profile_odom_health_acceptance_hz=>=%s\n' "$odom_effective_health_min_hz"
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        printf 'camera_health_preflight=passed_before_ROS_source_or_runtime_writes\n'
        printf 'camera_health_runtime_mode=duration_0_rolling_nonblocking_runtime_terminal_success_gate\n'
        printf 'camera_health_startup_gate=background_diagnostic_nonblocking_RViz\n'
        printf 'camera_health_failure_policy=do_not_close_RViz_midrun;save_artifacts_but_forbid_continuation_and_success\n'
        printf 'camera_health_valid_odom_contract=finite_near_unit_odom_and_same_stamp_odom_info_lite_lost_false\n'
        printf 'camera_health_high_integration_proxy=/info_exact_stamp_not_database_persistence\n'
        printf 'camera_health_nearest_or_restamp=forbidden\n'
    fi
    printf 'camera_profile_map_high_display_topic=unused_not_subscribed;publisher_candidate_expected_to_remove\n'
    printf 'apriltag_camera_stream=%s\n' "$([[ "$camera_profile" == "$default_camera_profile" ]] && printf mapping_high || printf legacy_single)"
    printf 'frame_cloud_camera_stream=odom_low\n'
    print_trial_ranges
    printf 'tracking_depth_z_m=%s-%s\n' "$RTAB_TRACKING_MIN_M" "$RTAB_TRACKING_MAX_M"
    printf 'vis_feature_radial_max_m=%s\n' "$RTAB_TRACKING_MAX_M"
    printf 'frame_cloud_radial_m=%s-%s\n' "$DENSE_MIN_M" "$DENSE_MAX_M"
    printf 'grid_export_depth_z_m=%s-%s\n' "$DENSE_MIN_M" "$DENSE_MAX_M"
    printf 'apriltag_radial_m=%s-%s\n' "$APRILTAG_MIN_RANGE_M" "$APRILTAG_MAX_RANGE_M"
    printf 'range_accuracy_status=trial_not_independently_validated\n'
    printf 'odom_cor_guess_match_to_projection=true\n'
    printf 'imu_orientation_source=image_synchronized_acceleration_roll_pitch\n'
    printf 'imu_yaw_policy=fixed_by_optical_to_base_convention;accelerometer_does_not_observe_yaw\n'
    printf 'imu_acceleration_gate_m_s2=8.0-12.0;outliers_not_published_as_gravity\n'
    printf 'imu_gravity_replay_validated=true;field_validated=false\n'
    printf 'minimum_range_trial=0.20m_requested;new_capture_effect_not_validated_by_old_0.30m_clipped_recordings\n'
    printf 'odom_feature_profile=gftt_min_distance_3_grid_2x3_depth_masked\n'
    printf 'odom_feature_profile_evidence=recorded_corner_replay_not_field_validation\n'
    printf 'close_object_policy=retain_depth_and_inlier_gates;do_not_fill_missing_depth_or_force_pose\n'
    printf 'depth_mode=%s\n' "$RTAB_DEPTH_MODE"
    printf 'imu_trial_enabled=%s\n' "$enable_imu"
    if [[ "$enable_imu" == 1 ]]; then
        printf 'imu_topic=/zed/imu/data\n'
        printf 'imu_time_reference=IMAGE\n'
        printf 'imu_frame=zed_left_camera_optical_frame\n'
        printf 'imu_odom_wait_to_init=true\n'
		printf 'imu_odom_local_bundle_adjustment=1\n'
		printf 'imu_odom_local_bundle_adjustment_max_frames=3\n'
        printf 'imu_odom_local_optimizer_iterations=5\n'
		printf 'imu_odom_local_bundle_adjustment_reason=bounded_local_BA_restores_IMU_gravity_constraints_field_trial\n'
        printf 'imu_graph_use_odom_gravity=true\n'
        printf 'imu_gravity_sigma=0.3\n'
        printf 'imu_scope=roll_pitch_gravity_assistance_not_translation_or_absolute_yaw\n'
    else
        printf 'imu_graph_use_odom_gravity=false\n'
    fi
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        printf 'slam_rgbd_odom_transport=atomic_rtabmap_msgs_RGBDImage\n'
        printf 'slam_rgbd_odom_sync_contract=one_DDS_sample_contains_RGB_depth_and_both_CameraInfo_objects\n'
        printf 'slam_rgbd_odom_transport_hypothesis=avoid_partial_raw_triplet_delivery;field_cause_not_proven\n'
    else
        printf 'slam_rgbd_odom_sync=exact\n'
        printf 'slam_rgbd_odom_sync_contract=identical_RGB_depth_camera_info_odom_header_stamps;unmatched_samples_not_integrated\n'
        printf 'slam_rgbd_odom_sync_purpose=prevent_cross_timestamp_geometry_pairing;drift_root_cause_not_proven\n'
    fi
    printf 'neighbor_link_refining=false\n'
    printf 'neighbor_edge_policy=skip_backend_neighbor_reregistration_keep_frontend_odometry_measurement\n'
    printf 'neighbor_override_evidence=recorded_edge_vs_raw_vo_not_ground_truth\n'
    printf 'field_effect_validated=false\n'
    printf 'reset_quarantine_runtime_id=reset_quarantine_shadow_v6_27a51dec_a4628a77_7a7c0e23\n'
    printf 'reset_quarantine_enabled=true\n'
    printf 'reset_quarantine_lost_count=8\n'
    printf 'reset_quarantine_odom_info_max_age_s=0.5\n'
    printf 'reset_quarantine_unannounced_jump_gate=0.5m;30deg;reject_without_epoch_until_trusted_recovery\n'
    printf 'reset_quarantine_sparse_frame_continuity=source_bound_observed_steps;exact_endpoints;same_chain;pause_logged_not_pose_reset;fixed_step_gate_0.5m_30deg;rejected_jump_does_not_advance_anchor\n'
    printf 'odom_external_reset_services=disabled_in_managed_safe_runtime\n'
    printf 'legacy_cross_map_pending_gate=false\n'
    printf 'reset_quarantine_field_effect_validated=false\n'
    printf 'joint_change_attribution=not_a_single_factor_causal_test\n'
    printf 'odom_reset_countdown=8\n'
    printf 'odom_reset_semantics=eight_consecutive_failures_then_reset_f2m_at_last_pose;shorter_loss_bursts_pause_without_clearing_local_map\n'
    printf 'odom_publish_null_when_lost=false\n'
    printf 'odom_lost_notification=authenticated_odom_info_only_no_invalid_pose\n'
    printf 'odom_lost_pose_policy=invalid_pose_not_integrated;high_map_frame_requires_finite_near_unit_quaternion\n'
    printf 'slam_start_new_map_on_loop_closure=false\n'
    printf 'slam_new_map_policy=after_odom_reset_continue_in_isolated_map_id;never_add_neighbor_edge_across_reset;merge_only_after_cross_map_pending_gate\n'
    printf 'post_loss_mapping_continues=true\n'
    printf 'odom_visual_grid=default_1x1\n'
    printf 'odom_min_inliers_distribution=default_0_disabled\n'
    printf 'odom_rotation_guess_window_px=default_40\n'
    printf 'rotation_frontend_rollback_reason=rotation_robust_trial_failed_to_build_a_usable_map;four_unvalidated_overrides_removed\n'
    printf 'odom_always_process_most_recent_frame=%s\n' "$([[ "$camera_profile" == "$default_camera_profile" ]] && printf true || printf default_true)"
    printf 'slam_topic_queue_size=%s\n' "$slam_topic_queue_size"
    printf 'slam_sync_queue_size=%s\n' "$slam_sync_queue_size"
    printf 'slam_queue_reason=%s\n' "$([[ "$camera_profile" == "$default_camera_profile" ]] && printf 1Hz_960x600_reliable_exact_valid_odom_ack || printf legacy_single_stream_rollback)"
    printf 'rtab_detection_rate_cap_hz=%s\n' "$RTAB_DETECTION_RATE"
    printf 'rtab_detection_rate_policy=%s\n' "$([[ "$camera_profile" == "$default_camera_profile" ]] && printf cap_5Hz_above_1Hz_input_preserves_backend_headroom || printf legacy_5Hz_cap)"
    printf 'keyframe_motion_gate=linear_0.08m;angular_0.0872665rad\n'
    printf 'graph_display_source=/mapGraph\n'
    printf 'graph_display_topic=/mapping/graph_display\n'
    printf 'graph_display_failure_policy=stop_and_save_without_restart\n'
    printf 'apriltag_landmark_mode=position_g2o\n'
    printf 'apriltag_optimizer_ignored=true\n'
    printf 'apriltag_optimizer_ignore_reason=cross_map_position_only_landmarks_are_not_full_se3_observable\n'
    printf 'apriltag_multiview_anchor_save=post_shutdown_read_only\n'
    printf 'apriltag_multiview_min_distinct_saved_nodes=3\n'
    printf 'apriltag_multiview_image_grid=3x3\n'
    printf 'apriltag_multiview_output=apriltag_multiview_anchors.json\n'
    printf 'apriltag_multiview_field_validated=false\n'
    printf 'apriltag_safe_relocalization=three_exact_views_single_json_transaction_core_owned\n'
    printf 'apriltag_safe_relocalization_continuation=%s\n' "$continuation_enabled"
    printf 'apriltag_safe_relocalization_three_bridge_commit=true\n'
    printf 'apriltag_safe_relocalization_leave_one_out_shadow_optimization=true\n'
    printf 'apriltag_safe_relocalization_old_component_drift_limit=0.005m_0.5deg\n'
    printf 'apriltag_safe_relocalization_new_component_rigidity_limit=0.03m_3deg_with_z0.02m_tilt2deg\n'
    printf 'apriltag_safe_relocalization_runtime_field_validated=false\n'
    printf 'odom_f2m_max_size=2000\n'
    printf 'odom_f2m_max_size_reason=single_factor_field_trial_after_loss_containment;previous_2000_trial_was_confounded\n'
    printf 'rgbd_proximity_by_space=false\n'
    printf 'landmark_time_guard_so=%s\n' "$landmark_time_guard_so"
    printf 'landmark_time_guard_sha256=%s\n' "$landmark_time_guard_sha256"
    printf 'landmark_time_guard_scope=slam_process_only\n'
    printf 'landmark_time_guard_tf_failure_policy=reject_detection\n'
    printf 'landmark_time_guard_observation_ttl=unchanged_not_guarded\n'
    printf 'landmark_time_guard_reset_epoch=unchanged_not_guarded\n'
    printf 'landmark_time_guard_field_effect_validated=false\n'
    printf 'landmark_time_guard_preflight_receipt=%s\n' "$landmark_time_guard_receipt"
    printf 'optimizer_strategy=1\n'
    printf 'optimizer_variance_ignored=false\n'
    printf 'export_approach=2_saved_optimized_poses_only\n'
    printf 'export_optimizer_parameter_strategy=1\n'
    printf 'export_optimizer_variance_ignored=false\n'
    printf 'export_fallback=forbidden\n'
    printf 'export_source_access=stopped_all_owned_children_then_slam_wait_zero_then_binary_copy_only\n'
    printf 'export_sql_scope=private_copy_only_never_source_database\n'
    printf 'export_acceptance=saved_pose_ids_and_transforms_match_and_valid_ply_and_database_metric_coverage\n'
    printf 'export_partial_database_policy=retain_diagnostics_and_previous_ply_report_failure\n'
    printf 'robot_pose_dof=6\n'
    printf 'single_landmark_full_pose_observable=false\n'
    printf 'live_map_source=/cloud_map (graph optimized)\n'
    printf 'cloud_map_qos=reliable,transient_local\n'
    printf 'light_mode=%s\n' "$light_mode"
    printf 'legacy_light_hook=disabled\n'
    if [[ "$light_mode" == teleop ]]; then
        printf 'light_control=production_teleop\n'
        printf 'teleop_pid=%s\n' "$teleop_pid"
        printf 'udp3333_owner_pid=%s\n' "$teleop_pid"
    else
        printf 'light_control=none\n'
    fi
    sha256sum /usr/local/zed/settings/SN40408468.conf
    sha256sum "$test_binary"
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        sha256sum "$camera_health_observer"
    fi
    sha256sum "$test_rviz" "$run_rviz_config"
    sha256sum "$graph_display_script" "$graph_display_module" "$submap_display_module" \
        "$run_graph_display_script" "$run_graph_display_module" "$run_submap_display_module"
    sha256sum "$apriltag_anchor_builder" "$apriltag_detector" "$apriltag_keyframe_binder" \
        "$safe_reloc_gid_frontend" "$safe_reloc_ipc_engine" \
        "$safe_reloc_engine_core" "$safe_reloc_identity_writer"
    if [[ "$profile" == custom ]]; then
        sha256sum "$candidate"
    fi
} > "$run_dir/test_manifest.txt"

bridge_pid=""
tf_pid=""
apriltag_pid=""
apriltag_keyframe_binder_pid=""
safe_reloc_gid_frontend_pid=""
safe_reloc_ipc_engine_pid=""
safe_reloc_ipc_dir=""
safe_reloc_socket=""
safe_reloc_engine_identity=""
odom_pid=""
slam_pid=""
camera_health_pid=""
cloud_frame_pid=""
graph_display_pid="" submap_display_pid=""
rviz_pid=""
timer_pid=""
cleaned=0
cleanup_nodes_stopped=false
cleanup_export_succeeded=false
cleanup_no_tag_available=false
cleanup_apriltag_anchor_succeeded=false
cleanup_continuation_contract_succeeded=false
critical_failure_reason=""
camera_health_ready_passed=false
runtime_admitted=false
declare -A owned_start_ticks=()
declare -A owned_exe_paths=()
declare -A owned_exe_devices=()
declare -A owned_exe_inodes=()
declare -A owned_cmd_tokens=()

process_start_ticks() {
    local record tail
    local -a fields=()
    [[ "$1" =~ ^[0-9]+$ && -r "/proc/$1/stat" ]] || return 1
    IFS= read -r record < "/proc/$1/stat" || return 1
    tail="${record##*) }"
    read -r -a fields <<< "$tail"
    [[ "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 1
    printf '%s\n' "${fields[19]}"
}

process_cmdline_has_token() {
    local pid="$1" expected="$2" argument
    [[ -r "/proc/$pid/cmdline" ]] || return 1
    while IFS= read -r -d '' argument; do
        [[ "$argument" != "$expected" ]] || return 0
    done < "/proc/$pid/cmdline"
    return 1
}

record_child() {
    local name="$1" pid="$2" expected_exe="${3:-}" expected_sha="${4:-}"
    local expected_cmd_token="${5:-}" ticks receipt expected_device expected_inode
    local actual_ticks actual_owner actual_device actual_inode admitted=false attempt
    # Preserve evidence even if /proc identity cannot be established. An
    # unverified process is never signalled by this launcher.
    ticks="$(process_start_ticks "$pid")" || {
        echo "无法记录建图子进程 $name 的身份，停止本次启动。" >&2
        return 1
    }
    owned_start_ticks[$pid]="$ticks"
    if [[ -n "$expected_exe" ]]; then
        check_frozen_runtime_file "$expected_exe" "$expected_sha" true || return 1
        read -r expected_device expected_inode < <(stat -Lc '%d %i' -- "$expected_exe") || return 1
        for ((attempt=0; attempt<20; attempt++)); do
            actual_ticks="$(process_start_ticks "$pid" 2>/dev/null || true)"
            actual_owner="$(stat -Lc %u -- "/proc/$pid" 2>/dev/null || true)"
            read -r actual_device actual_inode < <(
                stat -Lc '%d %i' -- "/proc/$pid/exe" 2>/dev/null || true) || true
            if [[ "$actual_ticks" == "$ticks" && "$actual_owner" == "$(id -u)" &&
                  "$actual_device" == "$expected_device" &&
                  "$actual_inode" == "$expected_inode" ]] &&
               { [[ -z "$expected_cmd_token" ]] ||
                 process_cmdline_has_token "$pid" "$expected_cmd_token"; }; then
                admitted=true
                break
            fi
            sleep 0.05
        done
        if [[ "$admitted" != true ]]; then
            echo "建图子进程 $name 未执行冻结二进制或命令参数身份不匹配，停止本次启动。" >&2
            return 1
        fi
        owned_exe_paths[$pid]="$expected_exe"
        owned_exe_devices[$pid]="$expected_device"
        owned_exe_inodes[$pid]="$expected_inode"
        owned_cmd_tokens[$pid]="$expected_cmd_token"
    fi
    atomic_state_line "$state_dir/$name.pid" "$pid" || return 1
}

record_child lock_broker "$launcher_lock_broker_pid" \
    "$python3_runtime" "$python3_runtime_sha256" "$state_dir/launcher.lock" || exit 1
if [[ "${owned_start_ticks[$launcher_lock_broker_pid]:-}" != \
      "$launcher_lock_broker_start_ticks" ]]; then
    echo '建图锁broker在启动准备期间身份发生变化。' >&2
    exit 1
fi

owned_process_alive() {
    local pid="${1:-}" actual owner expected_path expected_device expected_inode
    local path_device path_inode process_device process_inode expected_cmd_token
    [[ -n "$pid" && -n "${owned_start_ticks[$pid]:-}" ]] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    actual="$(process_start_ticks "$pid")" || return 1
    [[ "$actual" == "${owned_start_ticks[$pid]}" ]] || return 1
    owner="$(stat -Lc %u -- "/proc/$pid" 2>/dev/null)" || return 1
    [[ "$owner" == "$(id -u)" ]] || return 1
    expected_path="${owned_exe_paths[$pid]:-}"
    if [[ -n "$expected_path" ]]; then
        expected_device="${owned_exe_devices[$pid]:-}"
        expected_inode="${owned_exe_inodes[$pid]:-}"
        [[ -f "$expected_path" && ! -L "$expected_path" ]] || return 1
        read -r path_device path_inode < <(stat -Lc '%d %i' -- "$expected_path") || return 1
        read -r process_device process_inode < <(stat -Lc '%d %i' -- "/proc/$pid/exe") || return 1
        [[ "$path_device" == "$expected_device" && "$path_inode" == "$expected_inode" &&
           "$process_device" == "$expected_device" && "$process_inode" == "$expected_inode" ]] || return 1
        expected_cmd_token="${owned_cmd_tokens[$pid]:-}"
        [[ -z "$expected_cmd_token" ]] ||
            process_cmdline_has_token "$pid" "$expected_cmd_token" || return 1
    fi
    return 0
}

pid_may_be_running() {
    local pid="${1:-}"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    # Unknown/reused identity is not permission to signal it, nor evidence
    # that cleanup succeeded. Retain its PID evidence conservatively.
    kill -0 "$pid" 2>/dev/null || [[ -d "/proc/$pid" ]]
}

camera_health_state_status() {
    local expected_phase="${1:-live}"
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" - \
        "$run_dir/header_observer_state.json" "$map_info_topic" "$expected_phase" \
        "$camera_health_odom_min_hz" "$camera_health_high_min_hz" \
        "$camera_health_coverage_min" "$camera_health_integration_min_hz" <<'CAMERA_HEALTH_STATE_PY'
import json
import math
import os
import stat
import sys
import time

def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + str(key))
        result[key] = value
    return result

path, high_topic, expected_phase = sys.argv[1:4]
limits = tuple(float(value) for value in sys.argv[4:8])
if expected_phase not in ("live", "terminal"):
    raise SystemExit(1)
if not os.path.lexists(path):
    raise SystemExit(2)
try:
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        info = os.fstat(descriptor)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
                info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o600 or
                info.st_size <= 0 or info.st_size > 1024 * 1024):
            raise ValueError("unsafe observer state file")
        with os.fdopen(descriptor, "r", encoding="utf-8", closefd=False) as stream:
            event = json.load(stream, object_pairs_hook=reject_duplicate_pairs)
    finally:
        os.close(descriptor)
except (OSError, ValueError, TypeError, json.JSONDecodeError):
    raise SystemExit(1)

if event.get("schema_version") != 2:
    raise SystemExit(1)
event_monotonic_ns = event.get("monotonic_ns")
event_sequence = event.get("sequence")
now_monotonic_ns = time.monotonic_ns()
if (type(event_monotonic_ns) is not int or
        type(event_sequence) is not int or event_sequence < 0 or
        event_monotonic_ns < 0 or
        now_monotonic_ns < event_monotonic_ns or
        now_monotonic_ns - event_monotonic_ns > 3_000_000_000):
    raise SystemExit(1)
snapshot = event.get("snapshot")
if not isinstance(snapshot, dict) or snapshot.get("schema_version") != 2:
    raise SystemExit(1)
if snapshot.get("ready") is not True:
    raise SystemExit(2 if event.get("kind") == "warming" else 1)
if expected_phase == "live":
    if event.get("kind") not in ("ready", "rolling"):
        raise SystemExit(1)
else:
    if (event.get("kind") != "final" or event.get("exit_reason") != "SIGINT" or
            type(event.get("exit_code")) is not int or event.get("exit_code") != 130):
        raise SystemExit(1)
if event.get("topics") != {
    "odom": "/odom",
    "odom_info_lite": "/odom_info_lite",
    "high_camera_info": high_topic,
    "rtabmap_info": "/info",
}:
    raise SystemExit(1)
publisher_sources = event.get("publisher_sources")
expected_sources = {
    "odom": ("/odom", "rtabmap_rgbd_odometry", "/"),
    "odom_info_lite": ("/odom_info_lite", "rtabmap_rgbd_odometry", "/"),
    "high_camera_info": (high_topic, "zed_rgbd_dual_stream_publisher", "/"),
    "rtabmap_info": ("/info", "rtabmap", "/"),
}
if (not isinstance(publisher_sources, dict) or
        set(publisher_sources) != {"all_authenticated", "failure", "gid_contract_bytes", "sources"} or
        publisher_sources.get("all_authenticated") is not True or
        publisher_sources.get("failure") is not None or
        publisher_sources.get("gid_contract_bytes") != 24 or
        not isinstance(publisher_sources.get("sources"), dict) or
        set(publisher_sources["sources"]) != set(expected_sources)):
    raise SystemExit(1)
for role, (topic, node_name, node_namespace) in expected_sources.items():
    source = publisher_sources["sources"].get(role)
    if (not isinstance(source, dict) or
            set(source) != {"topic", "node_name", "node_namespace", "publisher_gid", "publisher_gid_bytes", "authenticated"} or
            source.get("topic") != topic or source.get("node_name") != node_name or
            source.get("node_namespace") != node_namespace or
            source.get("publisher_gid_bytes") != 24 or source.get("authenticated") is not True or
            not isinstance(source.get("publisher_gid"), str) or
            len(source["publisher_gid"]) != 48 or
            any(char not in "0123456789abcdef" for char in source["publisher_gid"]) or
            set(source["publisher_gid"]) == {"0"}):
        raise SystemExit(1)
try:
    metrics = snapshot["metrics"]
    gates = snapshot["gates"]
    values = (
        float(metrics["odom"]["valid_unique_rate_hz"]),
        float(metrics["high_camera_info"]["valid_unique_rate_hz"]),
        float(metrics["high_to_valid_odom_exact"]["coverage_ratio"]),
        float(metrics["high_integrated_proxy"]["rate_hz"]),
    )
    passed = (
        all(math.isfinite(value) for value in values) and
        all(value >= limit for value, limit in zip(values, limits)) and
        metrics["high_to_valid_odom_exact"]["nearest_stamp_or_restamp_used"] is False and
        metrics["high_integrated_proxy"]["observable"] is True and
        gates["integration_proxy_required"] is True and
        gates["hard_pass"] is True and
        gates["integration_proxy_pass"] is True and
        gates["candidate_pass"] is True
    )
except (KeyError, TypeError, ValueError):
    passed = False
raise SystemExit(0 if passed else 1)
CAMERA_HEALTH_STATE_PY
}

create_safe_reloc_ipc_directory() {
    local ipc_candidate_dir canonical owner mode
    ipc_candidate_dir="$(mktemp -d "/tmp/wbrov_sr.$$.XXXXXXXX")" || return 1
    canonical="$(readlink -f -- "$ipc_candidate_dir")" || return 1
    owner="$(stat -c %u -- "$ipc_candidate_dir")" || return 1
    mode="$(stat -c %a -- "$ipc_candidate_dir")" || return 1
    if [[ "$canonical" != "$ipc_candidate_dir" || -L "$ipc_candidate_dir" || ! -d "$ipc_candidate_dir" ||
          "$owner" != "$(id -u)" || "$mode" != 700 ||
          ! "$ipc_candidate_dir" =~ ^/tmp/wbrov_sr\.$$\.[A-Za-z0-9]{8}$ ]]; then
        echo '[启动拒绝] 安全重定位短IPC目录身份不安全。' >&2
        return 1
    fi
    safe_reloc_ipc_dir="$ipc_candidate_dir"
    safe_reloc_socket="$ipc_candidate_dir/i.sock"
    safe_reloc_engine_identity="$ipc_candidate_dir/e.id"
    if ((${#safe_reloc_socket} >= 108)); then
        echo '[启动拒绝] 安全重定位AF_UNIX socket路径超过107字节。' >&2
        return 1
    fi
}

cleanup_safe_reloc_ipc_directory() {
    local canonical owner mode
    [[ -n "$safe_reloc_ipc_dir" ]] || return 0
    canonical="$(readlink -f -- "$safe_reloc_ipc_dir" 2>/dev/null || true)"
    owner="$(stat -c %u -- "$safe_reloc_ipc_dir" 2>/dev/null || true)"
    mode="$(stat -c %a -- "$safe_reloc_ipc_dir" 2>/dev/null || true)"
    if [[ "$canonical" != "$safe_reloc_ipc_dir" || -L "$safe_reloc_ipc_dir" ||
          ! -d "$safe_reloc_ipc_dir" || "$owner" != "$(id -u)" || "$mode" != 700 ||
          ! "$safe_reloc_ipc_dir" =~ ^/tmp/wbrov_sr\.$$\.[A-Za-z0-9]{8}$ ]]; then
        echo '[清理拒绝] 安全重定位短IPC目录身份变化；保留现场证据。' >&2
        return 1
    fi
    if pid_may_be_running "$safe_reloc_gid_frontend_pid" ||
       pid_may_be_running "$safe_reloc_ipc_engine_pid"; then
        echo '[清理拒绝] 安全重定位子进程仍在，保留IPC目录。' >&2
        return 1
    fi
    if [[ -e "$safe_reloc_socket" || -L "$safe_reloc_socket" ]]; then
        if [[ ! -S "$safe_reloc_socket" || -L "$safe_reloc_socket" ||
              "$(stat -c %u -- "$safe_reloc_socket" 2>/dev/null || true)" != "$(id -u)" ||
              "$(stat -c %a -- "$safe_reloc_socket" 2>/dev/null || true)" != 600 ]]; then
            echo '[清理拒绝] IPC socket类型、属主或权限异常；保留证据。' >&2
            return 1
        fi
        rm -- "$safe_reloc_socket" || return 1
    fi
    if [[ -e "$safe_reloc_engine_identity" || -L "$safe_reloc_engine_identity" ]]; then
        if [[ -L "$safe_reloc_engine_identity" || ! -f "$safe_reloc_engine_identity" ||
              "$(stat -c %u -- "$safe_reloc_engine_identity" 2>/dev/null || true)" != "$(id -u)" ||
              "$(stat -c %a -- "$safe_reloc_engine_identity" 2>/dev/null || true)" != 600 ||
              "$(stat -c %h -- "$safe_reloc_engine_identity" 2>/dev/null || true)" != 1 ]]; then
            echo '[清理拒绝] engine identity文件身份异常；保留证据。' >&2
            return 1
        fi
        rm -- "$safe_reloc_engine_identity" || return 1
    fi
    rmdir -- "$safe_reloc_ipc_dir"
}

safe_reloc_state_status() {
    local expected_phase="$1"
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -I - \
        "$run_dir/corner_recovery/ready.json" \
        "$run_dir/corner_recovery/events.jsonl" \
        "$run_dir/corner_recovery/graph_states.jsonl" \
        "$run_dir/corner_recovery/RESULT.json" \
        "$run_dir/corner_recovery/cleanup.json" \
        "$run_dir/corner_recovery_config.json" \
        "$expected_phase" \
        "$safe_reloc_gid_frontend_pid" \
        "${owned_start_ticks[${safe_reloc_gid_frontend_pid:-0}]:-0}" \
        "$safe_relocalization_session_token_sha256" \
        "$safe_relocalization_database_sha256" <<'CORNER_RECOVERY_STATE_PY'
import hashlib
import json
import os
import stat
import sys

def unique(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError('duplicate JSON key')
        out[key] = value
    return out

def secure_bytes(path, *, required=True, limit=128 * 1024 * 1024):
    if not os.path.lexists(path):
        if not required:
            return None
        raise FileNotFoundError(path)
    fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
                info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o600 or
                info.st_size < 0 or info.st_size > limit):
            raise ValueError('unsafe recovery evidence inode')
        chunks = []
        remaining = info.st_size
        while remaining:
            block = os.read(fd, min(1024 * 1024, remaining))
            if not block:
                raise ValueError('short recovery evidence read')
            chunks.append(block)
            remaining -= len(block)
        return b''.join(chunks)
    finally:
        os.close(fd)

def load_json(path, *, required=True):
    raw = secure_bytes(path, required=required, limit=4 * 1024 * 1024)
    if raw is None:
        return None
    return json.loads(raw.decode('utf-8'), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))

def load_jsonl(path):
    raw = secure_bytes(path)
    if raw and not raw.endswith(b'\n'):
        raise BlockingIOError('writer has not completed the last JSONL record')
    records = []
    for line in raw.splitlines():
        if line:
            records.append(json.loads(line.decode('utf-8'), object_pairs_hook=unique,
                                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value))))
    return records

ready_path, events_path, states_path, result_path, cleanup_path, config_path = sys.argv[1:7]
phase = sys.argv[7]
pid = int(sys.argv[8])
start_ticks = int(sys.argv[9])
token_sha = sys.argv[10]
database_sha = sys.argv[11]
if phase not in ('admitted', 'live', 'terminal'):
    raise SystemExit(1)
try:
    ready = load_json(ready_path)
    config = load_json(config_path)
except FileNotFoundError:
    raise SystemExit(2 if phase in ('admitted', 'live') else 1)
except (OSError, UnicodeError, ValueError, TypeError, json.JSONDecodeError):
    raise SystemExit(1)

expected_nodes = {
    'detector': 'verified_apriltag_pixel_observer',
    'consumer': 'authenticated_core_pixel_consumer',
    'proposer': 'safe_reloc_gid_frontend',
}
expected_sources = {
    'core': {'topic': '/wbrov/accepted_image_input', 'node': 'rtabmap', 'namespace': '/'},
    'pixels': {'topic': '/apriltag/pixel_observations', 'node': 'verified_apriltag_pixel_observer', 'namespace': '/'},
    'rgb': {'topic': '/zed/mapping/rgb/image_rect_color', 'node': 'zed_rgbd_dual_stream_publisher', 'namespace': '/'},
    'camera_info': {'topic': '/zed/mapping/rgb/camera_info', 'node': 'zed_rgbd_dual_stream_publisher', 'namespace': '/'},
    'state': {'topic': '/wbrov/safe_relocalization/state', 'node': 'rtabmap', 'namespace': '/'},
    'status': {'topic': '/wbrov/safe_relocalization/status', 'node': 'rtabmap', 'namespace': '/'},
}
valid_config = (
    set(config) == {'fixture_mode', 'session_token', 'session_token_sha256',
                    'database_sha256', 'body_from_camera', 'camera_frame',
                    'pending_max_age_s', 'image_buffer_capacity', 'tag_side_m', 'min_tag_side_px',
                    'frame_ready_topic', 'nodes', 'sources'} and
    config.get('fixture_mode') is False and
    config.get('session_token_sha256') == token_sha and
    hashlib.sha256(config.get('session_token', '').encode('ascii')).hexdigest() == token_sha and
    config.get('database_sha256') == database_sha and
    config.get('pending_max_age_s') == 30 and
    config.get('image_buffer_capacity') == 128 and
    config.get('tag_side_m') == 0.08 and config.get('min_tag_side_px') == 8 and
    config.get('camera_frame') == 'zed_left_camera_optical_frame' and
    config.get('body_from_camera') == [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0] and
    config.get('frame_ready_topic') is None and
    config.get('nodes') == expected_nodes and
    config.get('sources') == expected_sources
)
valid_ready = (ready == {'pid': pid, 'start_ticks': str(start_ticks)} or
               ready == {'pid': pid, 'start_ticks': start_ticks})
if not valid_config or not valid_ready:
    raise SystemExit(1)

if phase in ('admitted', 'live'):
    try:
        events = load_jsonl(events_path)
        states = load_jsonl(states_path)
    except (FileNotFoundError, BlockingIOError):
        raise SystemExit(2)
    except (OSError, UnicodeError, ValueError, TypeError, json.JSONDecodeError):
        raise SystemExit(1)
    bad_event = any(record.get('event') in ('rejected', 'sources_invalidated') for record in events)
    frame_ready = [record for record in events if record.get('event') == 'frame_ready']
    valid_state = any(
        type(record.get('reset_epoch')) is int and record['reset_epoch'] >= 0 and
        type(record.get('graph_revision')) is int and record['graph_revision'] >= 0 and
        type(record.get('pose_count')) is int and record['pose_count'] > 0 and
        isinstance(record.get('actual_message_gid'), str) and len(record['actual_message_gid']) >= 32
        for record in states)
    if bad_event:
        raise SystemExit(1)
    # A stationary startup may retain only the first Core keyframe. Its RGB
    # predates this consumer's volatile subscription, so a full pixel join is
    # not a startup prerequisite. ready.json is written only after source
    # admission; a checked graph message establishes a working Core observer.
    # This status never authorizes a recovery proposal or a graph update.
    if not valid_state or (phase == 'live' and not frame_ready):
        raise SystemExit(2)
    raise SystemExit(0)

try:
    result = load_json(result_path)
    cleanup = load_json(cleanup_path)
except (FileNotFoundError, OSError, UnicodeError, ValueError, TypeError, json.JSONDecodeError):
    raise SystemExit(1)
counts = result.get('counts')
terminals = result.get('bridge_terminals')
graph_epochs = result.get('graph_epochs')
committed_epochs = result.get('committed_epochs')
valid_epochs = (
    isinstance(graph_epochs, list) and graph_epochs and
    all(type(value) is int and value >= 0 for value in graph_epochs) and
    graph_epochs == sorted(set(graph_epochs)) and
    isinstance(committed_epochs, list) and committed_epochs == graph_epochs[1:]
)
valid_terminal = (
    result.get('state') == 'closed' and
    result.get('fixture_mode') is False and
    result.get('image_buffer_capacity') == 128 and
    isinstance(result.get('buffer_dropped'), dict) and
    set(result['buffer_dropped']) == {'rgb', 'camera_info'} and
    all(type(value) is int and value >= 0 for value in result['buffer_dropped'].values()) and
    type(result.get('ready_frames')) is int and result['ready_frames'] >= 0 and
    type(result.get('graph_states')) is int and result['graph_states'] > 0 and
    result.get('source_compromised') is False and
    result.get('graph_state_rejections') == 0 and
    isinstance(counts, dict) and counts.get('rejected') == 0 and
    valid_epochs and isinstance(terminals, dict) and
    result.get('bridge_proposals') == len(terminals) == len(committed_epochs) and
    all(value.get('result') == 'committed' for value in terminals.values()) and
    cleanup == {'primary_exception': None, 'cleanup_errors': [], 'bridge_complete': True}
)
raise SystemExit(0 if valid_terminal else 1)
CORNER_RECOVERY_STATE_PY
}
# BEGIN GRAPH_DISPLAY_SESSION_MONITOR
verify_critical_children_alive() {
    local context="$1" critical_name critical_pid
    local -a critical_names=(lock_broker slam odom bridge tf apriltag apriltag_keyframe_binder safe_reloc_gid_frontend cloud_frame graph_display submap_display)
    [[ -z "$camera_health_pid" ]] || critical_names+=(camera_health)
    for critical_name in "${critical_names[@]}"; do
        case "$critical_name" in
            lock_broker) critical_pid="$launcher_lock_broker_pid" ;;
            slam) critical_pid="$slam_pid" ;;
            odom) critical_pid="$odom_pid" ;;
            bridge) critical_pid="$bridge_pid" ;;
            tf) critical_pid="$tf_pid" ;;
            apriltag) critical_pid="$apriltag_pid" ;;
            apriltag_keyframe_binder) critical_pid="$apriltag_keyframe_binder_pid" ;;
            safe_reloc_gid_frontend) critical_pid="$safe_reloc_gid_frontend_pid" ;;
            cloud_frame) critical_pid="$cloud_frame_pid" ;;
            graph_display) critical_pid="$graph_display_pid" ;;
            submap_display) critical_pid="$submap_display_pid" ;;
            camera_health) critical_pid="$camera_health_pid" ;;
        esac
        if ! owned_process_alive "$critical_pid"; then
            critical_failure_reason="$critical_name PID=$critical_pid unexpectedly exited or lost its recorded identity ($context)"
            return 1
        fi
    done
    return 0
}

wait_for_display_session() {
    local session_pid="$1" session_role="$2" result=0
    local stable_polls=0 session_admitted=false
    # Poll in this foreground shell: no detached watcher and no inherited
    # SIGINT trap. RViz close/timer completion still enters the same cleanup.
    while :; do
        # Do not reap here: cleanup validates SLAM's real wait status only
        # after every owned child is absent. Never signal an unknown PID.
        verify_critical_children_alive "display-session poll" || return 1
        if ! pid_may_be_running "$session_pid"; then
            # Reap only a PID already absent, never an unknown/reused process.
            wait "$session_pid" || result=$?
            if [[ "$session_role" == rviz ]]; then
                if [[ "$session_admitted" != true || "$result" != 0 ]]; then
                    critical_failure_reason="RViz exited before stable admission or returned status=$result"
                    echo "[RViz启动/运行失败] status=$result；不能把闪退当成用户正常关闭。日志请查看桌面终端。" >&2
                    return 1
                fi
                verify_critical_children_alive "RViz terminal recheck" || return 1
                return 0
            fi
            verify_critical_children_alive "timer terminal recheck" || return 1
            return "$result"
        fi
        if ! owned_process_alive "$session_pid"; then
            # It may have exited between kill -0 and the /proc identity read.
            # Reap only after absence is confirmed; never wait on an unknown PID.
            if ! pid_may_be_running "$session_pid"; then
                continue
            fi
            echo "[显示会话失效] $session_role PID=$session_pid 身份无法确认，停止并保存；不对未知进程发信号。" >&2
            return 1
        fi
        if ! owned_process_alive "$graph_display_pid"; then
            echo "[图结构显示失效] graph_display PID=$graph_display_pid 已退出或身份改变，停止并保存；不自动重启。日志: $run_dir/graph_display.log" >&2
            return 1
        fi
        if [[ "$session_role" == rviz && "$session_admitted" != true ]]; then
            ((stable_polls+=1))
            if ((stable_polls >= 10)); then
                session_admitted=true
            fi
        fi
        sleep 0.1
    done
}
# END GRAPH_DISPLAY_SESSION_MONITOR

stop_pid() {
    local pid="${1:-}"
    if owned_process_alive "$pid"; then
        kill -INT "$pid" 2>/dev/null || true
    fi
}

wait_children() {
    local count pid active
    # One shared approximately 40-second budget, not one per child.
    for ((count=0; count<400; count++)); do
        active=0
        for pid in "$@"; do
            if pid_may_be_running "$pid"; then
                active=1
            fi
        done
        ((active)) || return 0
        sleep 0.1
    done
    return 1
}

remove_matching_pid_file() {
    local name="$1" expected="${2:-}" recorded
    [[ -n "$expected" ]] || return 0
    if ! recorded="$(read_secure_pid_file "$state_dir/$name.pid" 2>/dev/null)"; then
        [[ ! -e "$state_dir/$name.pid" && ! -L "$state_dir/$name.pid" ]] && return 0
        return 1
    fi
    if [[ "$recorded" == "$expected" ]]; then
        rm -f -- "$state_dir/$name.pid" && sync -f -- "$state_dir"
    fi
}

safe_export_guard() {
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" - "$@" <<'SAFE_EXPORT_PY'
import contextlib
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import stat
import struct
import sys
import zlib

MAX_POSES = 200000
MAX_LINKS = 2000000
ROTATION_EPS = 0.001  # Serialization sanity, not a tracking/registration gate.
POSE_TEXT_EPS = 0.0001  # Export format 11 prints six decimal places.


class GuardError(ValueError):
    pass


def reject_constant(value):
    raise GuardError("non-finite JSON constant: " + value)


def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise GuardError("duplicate JSON key: " + str(key))
        result[key] = value
    return result


def require(condition, message):
    if not condition:
        raise GuardError(message)


def fingerprint(info):
    return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns]


def regular(path):
    info = path.lstat()
    require(stat.S_ISREG(info.st_mode), "not a regular non-symlink file: " + str(path))
    return info


def sidecars_absent(database):
    for suffix in ("-wal", "-shm", "-journal"):
        require(not os.path.lexists(str(database) + suffix),
                "source sidecar present; no implicit checkpoint/recovery: " + suffix)


def source_observations(database, source_info):
    """Sample visible FDs and kernel locks; visibility gaps are not a guarantee."""
    require(sys.platform.startswith("linux"), "source holder audit requires Linux /proc")
    target_names = {str(database) + suffix for suffix in ("", "-wal", "-shm", "-journal")}
    holders, gaps = [], []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            descriptors = list((process / "fd").iterdir())
        except FileNotFoundError:
            continue
        except PermissionError:
            gaps.append(int(process.name))
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
                info = descriptor.stat()
            except FileNotFoundError:
                continue
            except PermissionError:
                gaps.append(int(process.name))
                continue
            name = target.removesuffix(" (deleted)")
            if name in target_names or (info.st_dev, info.st_ino) == (source_info.st_dev, source_info.st_ino):
                holders.append([int(process.name), descriptor.name, target])
    lock_key = (os.major(source_info.st_dev), os.minor(source_info.st_dev), source_info.st_ino)
    locks = []
    for line in Path("/proc/locks").read_text(encoding="ascii").splitlines():
        for field in line.split():
            match = re.fullmatch(r"([0-9a-fA-F]+):([0-9a-fA-F]+):([0-9]+)", field)
            if match and (int(match[1], 16), int(match[2], 16), int(match[3])) == lock_key:
                locks.append(line)
    require(not holders and not locks, "visible source database holder or kernel lock: " + repr([holders, locks]))
    return {"visible_holders": holders, "kernel_locks": locks,
            "visibility_gap_pids": sorted(set(gaps)), "absolute_all_uid_exclusion_proven": False}


def copy_source_snapshot(database, output_dir):
    database = Path(os.path.abspath(database))
    before = regular(database)
    require(before.st_size > 0, "empty source database")
    sidecars_absent(database)
    audit_before = source_observations(database, before)
    destination = output_dir / "source.db"
    digest = hashlib.sha256()
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    with os.fdopen(os.open(database, flags), "rb") as source:
        require(fingerprint(os.fstat(source.fileno())) == fingerprint(before), "source identity changed before copy")
        fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "wb") as target:
            while True:
                part = source.read(1024 * 1024)
                if not part:
                    break
                digest.update(part)
                target.write(part)
            target.flush()
            os.fsync(target.fileno())
        require(fingerprint(os.fstat(source.fileno())) == fingerprint(before), "source changed during copy")
    sidecars_absent(database)
    require(fingerprint(regular(database)) == fingerprint(before), "source pathname changed during copy")
    audit_after = source_observations(database, before)
    sidecars_absent(database)
    require(fingerprint(regular(database)) == fingerprint(before), "source changed during final copy audit")
    return {"path": str(database), "stat": fingerprint(before), "sha256": digest.hexdigest(),
            "holder_audit_before": audit_before, "holder_audit_after": audit_after,
            "source_sqlite_opened": False, "sidecar_policy": "all_three_absent_no_checkpoint"}


def decode_mat(blob, type_number, columns=None):
    require(isinstance(blob, bytes) and 12 < len(blob) <= MAX_POSES * 48 + 65536,
            "missing/empty/oversized compressed matrix")
    rows, cols, kind = struct.unpack("<iii", blob[-12:])
    require(rows == 1 and kind == type_number and 0 < cols <= MAX_POSES * 12,
            "unsupported compressed matrix shape/type")
    if columns is not None:
        require(cols == columns, "compressed matrix column count mismatch")
    size = cols * 4
    decoder = zlib.decompressobj()
    raw = decoder.decompress(blob[:-12], size + 1)
    require(len(raw) == size and decoder.eof and not decoder.unused_data and not decoder.unconsumed_tail,
            "truncated/trailing/oversized zlib matrix payload")
    return cols, raw


def valid_pose(values):
    require(len(values) == 12 and all(math.isfinite(v) for v in values), "nonfinite/malformed pose")
    rotation = [values[0:3], values[4:7], values[8:11]]
    for row in range(3):
        for other in range(3):
            dot = sum(rotation[row][i] * rotation[other][i] for i in range(3))
            require(abs(dot - (1.0 if row == other else 0.0)) <= ROTATION_EPS,
                    "degenerate/non-orthonormal rotation")
    a, b, c = rotation
    determinant = a[0] * (b[1]*c[2]-b[2]*c[1]) - a[1] * (b[0]*c[2]-b[2]*c[0]) + a[2] * (b[0]*c[1]-b[1]*c[0])
    require(abs(determinant - 1.0) <= ROTATION_EPS, "improper rotation")


def pose_blob(blob):
    require(isinstance(blob, bytes) and len(blob) == 48, "invalid raw transform encoding")
    values = list(struct.unpack("<12f", blob))
    valid_pose(values)
    return values


def components(ids, edges, saved):
    parent = {node: node for node in ids}
    def find(node):
        parent.setdefault(node, node)
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node
    for first, second in edges:
        parent[find(first)] = find(second)
    groups = {}
    for node in ids:
        groups.setdefault(find(node), []).append(node)
    return [{"positive_count": len(nodes), "saved_positive_count": len(set(nodes) & saved),
             "missing_positive_ids": sorted(set(nodes) - saved)}
            for nodes in sorted(groups.values(), key=lambda items: (-len(items), min(items)))]


def load_saved_graph(database):
    require(sys.byteorder == "little", "only pinned little-endian RTAB-Map encoding is supported")
    regular(database)
    uri = database.resolve().as_uri() + "?mode=ro&immutable=1"
    with contextlib.closing(sqlite3.connect(uri, uri=True, timeout=0.0)) as connection:
        connection.execute("PRAGMA query_only=ON")
        admin = connection.execute("SELECT version,opt_ids,opt_poses FROM Admin LIMIT 2").fetchall()
        require(len(admin) == 1 and admin[0][0] == "0.23.7", "unsupported/missing Admin version or row count")
        count, id_bytes = decode_mat(admin[0][1], 4)
        require(count <= MAX_POSES, "too many saved poses")
        _, pose_bytes = decode_mat(admin[0][2], 5, count * 12)
        ids = list(struct.unpack("<" + "i" * count, id_bytes))
        require(0 not in ids and len(set(ids)) == count, "zero or duplicate saved pose IDs")
        positive = {node for node in ids if node > 0}
        require(positive, "saved graph contains no positive node")
        matrices = [list(row) for row in struct.iter_unpack("<12f", pose_bytes)]
        for matrix in matrices:
            valid_pose(matrix)
        raw_nodes = connection.execute("SELECT id,weight,pose FROM Node WHERE id>0 LIMIT ?", (MAX_POSES + 1,)).fetchall()
        require(len(raw_nodes) <= MAX_POSES, "too many database nodes")
        all_nodes, eligible, excluded, invalid = set(), set(), set(), []
        for node, weight, blob in raw_nodes:
            require(type(node) is int and node > 0 and node not in all_nodes and type(weight) is int,
                    "invalid/duplicate Node id or weight")
            all_nodes.add(node)
            if weight <= -9 or weight == -1:
                excluded.add(node)
                continue
            try:
                pose_blob(blob)
            except GuardError:
                invalid.append(node)
                continue
            eligible.add(node)
        require(positive <= eligible, "saved positive IDs lack eligible finite Node poses")
        data = {row[0]: row[1:] for row in connection.execute("SELECT id,length(image),length(depth) FROM Data")}
        require(all(node in data and data[node][0] and data[node][1] for node in positive),
                "saved positive ID lacks nonempty RGB/depth Data blobs")
        participants, neighbor_edges, metric_edges, tag_edges = set(), [], [], []
        dangling, null_links, virtual_links, link_count = [], 0, 0, 0
        excluded_unary_links = []
        for first, second, kind, blob in connection.execute("SELECT from_id,to_id,type,transform FROM Link"):
            link_count += 1
            require(link_count <= MAX_LINKS, "too many database links")
            require(type(first) is int and type(second) is int and type(kind) is int and 0 <= kind <= 9,
                    "unknown/malformed link type or endpoint")
            if kind == 5:
                virtual_links += 1
                continue
            if blob is None or (isinstance(blob, bytes) and (not blob or not any(blob))):
                null_links += 1
                continue
            pose_blob(blob)
            # Memory::moveToTrash keeps self references when marking a removed
            # signature weight=-9. Intermediate/removed nodes are excluded
            # above; their unary priors/gravity do not connect metric nodes.
            if kind in (7, 9) and first == second and first in excluded:
                excluded_unary_links.append([first, second, kind])
                continue
            endpoints = {node for node in (first, second) if node > 0}
            participants.update(endpoints & eligible)
            if not endpoints or not endpoints <= eligible or first == 0 or second == 0:
                dangling.append([first, second, kind])
                continue
            if kind in (0, 1, 2, 3, 4, 6):
                if first <= 0 or second <= 0:
                    dangling.append([first, second, kind])
                    continue
                metric_edges.append((first, second))
                if kind in (0, 6):
                    neighbor_edges.append((first, second))
            elif kind == 8:
                if not (min(first, second) < 0 < max(first, second)):
                    dangling.append([first, second, kind])
                    continue
                tag_edges.append((first, second))
            # PosePrior(7)/Gravity(9) count participation, not SE3 connectivity.
        missing = sorted(participants - positive)
        coverage = {"database_positive_count": len(all_nodes), "eligible_positive_count": len(eligible),
                    "metric_participating_positive_count": len(participants),
                    "saved_positive_count": len(positive), "saved_negative_count": count - len(positive),
                    "missing_metric_positive_ids": missing,
                    "unlinked_or_nonmetric_positive_ids": sorted(eligible - participants),
                    "weight_excluded_positive_ids": sorted(excluded), "invalid_node_pose_ids": sorted(invalid),
                    "dangling_metric_links": dangling, "null_links_ignored": null_links,
                    "virtual_links_ignored": virtual_links,
                    "excluded_unary_links_ignored": excluded_unary_links,
                    "neighbor_components": components(participants, neighbor_edges, positive),
                    "metric_se3_components_including_user_links": components(participants, metric_edges, positive),
                    "tag_structural_components_not_full_pose_observability": components(participants, metric_edges + tag_edges, positive),
                    "metric_coverage_complete": not missing and not dangling,
                    "rgb_depth_blob_presence_checked_not_decoded": True,
                    "per_node_cloud_contribution_verified": False}
        return {"schema_version": 1, "saved_ids": ids, "saved_poses": matrices,
                "saved_admin_payload_sha256": hashlib.sha256(admin[0][1] + admin[0][2]).hexdigest(),
                "coverage": coverage}


def write_json(path, value):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(value, stream, allow_nan=False, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def prepare(database, output_dir):
    output_dir = Path(output_dir)
    require(output_dir.is_dir() and not output_dir.is_symlink(), "invalid private export directory")
    source = copy_source_snapshot(Path(database), output_dir)
    write_json(output_dir / "source_snapshot.json", source)
    expected = load_saved_graph(output_dir / "source.db")
    expected["source_snapshot"] = source
    write_json(output_dir / "saved_poses.expected.json", expected)
    print("SAFE_EXPORT_PREPARED " + json.dumps(expected["coverage"], sort_keys=True))


def pose_file(path, expected):
    regular(path)
    expected_by_id = dict(zip(expected["saved_ids"], expected["saved_poses"]))
    found = set()
    with path.open("r", encoding="ascii") as stream:
        for line in stream:
            if line.startswith("#") or not line.strip():
                continue
            fields = line.split()
            require(len(fields) == 9 and re.fullmatch(r"-?[0-9]+", fields[8]), "bad format11 pose row")
            node = int(fields[8])
            require(node != 0 and node not in found and node in expected_by_id, "unexpected/duplicate exported pose ID")
            values = [float(value) for value in fields[:8]]
            require(all(math.isfinite(value) for value in values), "nonfinite exported pose")
            _, x, y, z, qx, qy, qz, qw = values
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            require(abs(norm - 1.0) <= POSE_TEXT_EPS, "invalid exported quaternion")
            qx, qy, qz, qw = [q/norm for q in (qx, qy, qz, qw)]
            matrix = [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw), x,
                      2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw), y,
                      2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy), z]
            require(max(abs(a-b) for a, b in zip(matrix, expected_by_id[node])) <= POSE_TEXT_EPS,
                    "exported transform differs from saved optimized pose")
            found.add(node)
    require(found == set(expected_by_id), "exported pose coverage differs from saved IDs (positive or landmark)")
    return {"positive_count": sum(node > 0 for node in found), "negative_count": sum(node < 0 for node in found)}


def ply_file(path):
    regular(path)
    integer_types = {"char": (-128,127), "int8": (-128,127), "uchar": (0,255), "uint8": (0,255),
                     "short": (-32768,32767), "int16": (-32768,32767), "ushort": (0,65535), "uint16": (0,65535),
                     "int": (-2147483648,2147483647), "int32": (-2147483648,2147483647),
                     "uint": (0,4294967295), "uint32": (0,4294967295)}
    float_types = {"float", "float32", "double", "float64"}
    def scalar(token, kind):
        if kind in integer_types:
            require(re.fullmatch(r"[+-]?[0-9]+", token), "invalid PLY integer")
            value = int(token)
            low, high = integer_types[kind]
            require(low <= value <= high, "PLY integer out of range")
            return value
        require(kind in float_types, "unsupported PLY scalar type")
        return float(token)
    elements, header_bytes = [], 0
    with path.open("r", encoding="ascii") as stream:
        require(stream.readline().strip() == "ply" and stream.readline().strip() == "format ascii 1.0",
                "not an ASCII PLY 1.0")
        for _ in range(2048):
            line = stream.readline()
            header_bytes += len(line)
            require(line and header_bytes <= 131072, "truncated/oversized PLY header")
            fields = line.split()
            require(fields, "empty PLY header line")
            if fields[0] in ("comment", "obj_info"):
                continue
            if fields == ["end_header"]:
                break
            if fields[0] == "element":
                require(len(fields) == 3 and fields[2].isdigit(), "bad PLY element")
                count = int(fields[2])
                require(count <= 100000000 and fields[1] not in [item[0] for item in elements], "bad PLY count/name")
                elements.append([fields[1], count, []])
            elif fields[0] == "property":
                require(elements and (len(fields) == 3 or (len(fields) == 5 and fields[1] == "list")), "bad PLY property")
                property_spec = fields[1:]
                if len(property_spec) == 2:
                    require(property_spec[0] in integer_types or property_spec[0] in float_types, "unknown PLY property type")
                else:
                    require(property_spec[1] in integer_types and
                            (property_spec[2] in integer_types or property_spec[2] in float_types), "unknown PLY list type")
                require(property_spec[-1] not in [item[-1] for item in elements[-1][2]], "duplicate PLY property")
                elements[-1][2].append(property_spec)
            else:
                raise GuardError("unknown PLY header entry")
        else:
            raise GuardError("PLY header has no end_header")
        require(elements and elements[0][0] == "vertex" and elements[0][1] > 0, "PLY has no vertices")
        vertex_props = elements[0][2]
        require(all(len(spec) == 2 for spec in vertex_props), "list-valued PLY vertex unsupported")
        require(all(any(spec[-1] == axis and spec[0] in float_types for spec in vertex_props) for axis in ("x","y","z")),
                "PLY lacks float XYZ")
        minimum, maximum = [math.inf]*3, [-math.inf]*3
        for name, count, properties in elements:
            require(properties or count == 0, "PLY element lacks properties")
            for _ in range(count):
                tokens = stream.readline().split()
                require(tokens, "truncated PLY data")
                cursor, coordinates = 0, {}
                for spec in properties:
                    require(cursor < len(tokens), "truncated PLY row")
                    if len(spec) == 2:
                        value = scalar(tokens[cursor], spec[0])
                        cursor += 1
                        if name == "vertex" and spec[-1] in ("x","y","z"):
                            require(math.isfinite(value), "nonfinite PLY XYZ")
                            coordinates[spec[-1]] = value
                    else:
                        require(spec[1] in integer_types, "PLY list count must be integer")
                        length = scalar(tokens[cursor], spec[1])
                        cursor += 1
                        require(0 <= length <= 100000 and cursor + length <= len(tokens), "bad PLY list length")
                        for token in tokens[cursor:cursor+length]:
                            scalar(token, spec[2])
                        cursor += length
                require(cursor == len(tokens), "extra PLY row fields")
                if name == "vertex":
                    for axis, key in enumerate(("x","y","z")):
                        minimum[axis] = min(minimum[axis], coordinates[key])
                        maximum[axis] = max(maximum[axis], coordinates[key])
        require(not stream.read().strip(), "extra data after declared PLY elements")
    return {"vertex_count": elements[0][1], "bbox_min_m": minimum, "bbox_max_m": maximum,
            "per_node_cloud_contribution_verified": False}


def verify_snapshot(expected, output_dir):
    snapshot = expected["source_snapshot"]
    source = Path(snapshot["path"])
    before = regular(source)
    require(fingerprint(before) == snapshot["stat"], "source database changed after preparation")
    sidecars_absent(source)
    audit = source_observations(source, before)
    copy_path = output_dir / "source.db"
    sidecars_absent(copy_path)
    copy_before = regular(copy_path)
    digest = hashlib.sha256()
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
    with os.fdopen(os.open(copy_path, flags), "rb") as stream:
        require(fingerprint(os.fstat(stream.fileno())) == fingerprint(copy_before), "working copy identity changed")
        for part in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(part)
        require(fingerprint(os.fstat(stream.fileno())) == fingerprint(copy_before), "working copy changed during verification")
    require(fingerprint(regular(copy_path)) == fingerprint(copy_before), "working copy pathname changed")
    require(digest.hexdigest() == snapshot["sha256"], "working copy bytes changed after preparation/export")
    sidecars_absent(copy_path)
    sidecars_absent(source)
    require(fingerprint(regular(source)) == snapshot["stat"], "source database changed during verification")
    return audit


def verify(output_dir):
    output_dir = Path(output_dir)
    expected_path = output_dir / "saved_poses.expected.json"
    regular(expected_path)
    expected = json.loads(
        expected_path.read_text(encoding="utf-8"),
        parse_constant=reject_constant, object_pairs_hook=reject_duplicate_pairs)
    audit = verify_snapshot(expected, output_dir)
    poses = pose_file(output_dir / "rtabmap_dense_poses.txt", expected)
    cloud = ply_file(output_dir / "rtabmap_dense_cloud.ply")
    complete = expected["coverage"]["metric_coverage_complete"]
    summary = {"schema_version": 1,
               "status": "complete_saved_graph_and_metric_coverage" if complete else "complete_saved_graph_but_partial_database",
               "publish_allowed": complete, "poses": poses, "cloud": cloud,
               "coverage": expected["coverage"], "source_holder_audit_before_publish": audit,
               "field_accuracy_validated": False}
    write_json(output_dir / "export_validation.json", summary)
    print("SAFE_EXPORT_VALIDATION " + json.dumps(summary, sort_keys=True))
    require(complete, "saved graph exported, but database metric nodes/links remain uncovered; old PLY retained")


def main(arguments):
    if len(arguments) == 3 and arguments[0] == "prepare":
        prepare(arguments[1], arguments[2])
    elif len(arguments) == 2 and arguments[0] == "verify":
        verify(arguments[1])
    else:
        raise GuardError("usage: prepare DB PRIVATE_DIR | verify PRIVATE_DIR")


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except (GuardError, OSError, sqlite3.Error, ValueError, TypeError, KeyError, struct.error, zlib.error) as error:
        print("SAFE_EXPORT_REJECTED: " + str(error), file=sys.stderr)
        sys.exit(1)
SAFE_EXPORT_PY
}

continuation_anchor_copy=""
prepare_continuation_database() {
    [[ "$continuation_enabled" == true ]] || return 0
    local current_receipt temp_dir post_receipt
    current_receipt="$(validate_continuation_candidate "$continuation_source_run")" || return 1
    if [[ "$current_receipt" != "$(printf '%s\n%s\n%s\n%s\n%s\n%s' \
        "$continuation_source_run" "$continuation_source_database" \
        "$continuation_source_anchors" "$continuation_database_sha256" \
        "$continuation_anchors_sha256" "$continuation_source_created_unix_ns")" ]]; then
        echo '[续建拒绝] 源地图在选择后发生变化。' >&2
        return 1
    fi
    temp_dir="$(mktemp -d "$run_dir/.continuation_copy.XXXXXX")" || return 1
    printf 'private_continuation_directory=%s\n' "$temp_dir" \
        > "$run_dir/continuation_prepare.log" || return 1
    if ! safe_export_guard prepare "$continuation_source_database" "$temp_dir" \
        >> "$run_dir/continuation_prepare.log" 2>&1; then
        echo "[续建拒绝] 旧数据库安全副本验证失败；证据保留在 $temp_dir" >&2
        return 1
    fi
    continuation_anchor_copy="$run_dir/continued_apriltag_anchors.json"
    if ! PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" - \
        "$temp_dir" "$continuation_database_sha256" \
        "$continuation_source_anchors" "$continuation_anchors_sha256" \
        "$database" "$continuation_anchor_copy" "$run_dir" <<'CONTINUATION_COPY_PY'
import hashlib
import json
import os
from pathlib import Path
import stat
import sys

temp, expected_db, source_anchor, expected_anchor, database, anchor_copy, run_dir = (
    Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]), sys.argv[4],
    Path(sys.argv[5]), Path(sys.argv[6]), Path(sys.argv[7]))

def require(value, message):
    if not value:
        raise SystemExit(message)

def reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)

def reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + str(key))
        result[key] = value
    return result

def identity(path):
    info = path.lstat()
    require(stat.S_ISREG(info.st_mode) and not stat.S_ISLNK(info.st_mode) and
            info.st_uid == os.getuid() and info.st_nlink == 1 and
            not (stat.S_IMODE(info.st_mode) & 0o022),
            "unsafe continuation source: " + str(path))
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns,
            info.st_ctime_ns)

def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()

snapshot_path = temp / "source_snapshot.json"
saved_path = temp / "saved_poses.expected.json"
source_db_copy = temp / "source.db"
snapshot = json.loads(
    snapshot_path.read_text(encoding="utf-8"),
    parse_constant=reject_constant, object_pairs_hook=reject_duplicate_pairs)
require(snapshot.get("sha256") == expected_db and
        digest(source_db_copy) == expected_db,
        "private database copy digest mismatch")
anchor_before = identity(source_anchor)
require(digest(source_anchor) == expected_anchor,
        "source anchor digest changed before copy")
flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
source_fd = os.open(source_anchor, flags)
target_fd = os.open(anchor_copy,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                    getattr(os, "O_NOFOLLOW", 0), 0o600)
try:
    require(identity(source_anchor) == anchor_before,
            "source anchor identity changed before copy")
    while True:
        block = os.read(source_fd, 1024 * 1024)
        if not block:
            break
        offset = 0
        while offset < len(block):
            written = os.write(target_fd, block[offset:])
            require(written > 0, "short anchor write")
            offset += written
    os.fsync(target_fd)
    os.fchmod(target_fd, 0o600)
finally:
    os.close(target_fd)
    os.close(source_fd)
require(identity(source_anchor) == anchor_before and
        digest(source_anchor) == expected_anchor and
        digest(anchor_copy) == expected_anchor,
        "source or copied anchors changed")
require(not database.exists(), "new run database target already exists")
os.replace(source_db_copy, database)
os.chmod(database, 0o600)
os.replace(snapshot_path, run_dir / "continuation_source_snapshot.json")
os.replace(saved_path, run_dir / "continuation_saved_poses.expected.json")
directory_fd = os.open(
    run_dir, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC |
    getattr(os, "O_NOFOLLOW", 0))
try:
    os.fsync(directory_fd)
finally:
    os.close(directory_fd)
receipt = {
    "schema_version": 1,
    "source_database_sha256": expected_db,
    "source_anchors_sha256": expected_anchor,
    "database_private_copy": str(database),
    "anchors_private_copy": str(anchor_copy),
    "source_database_sqlite_opened": False,
    "private_database_quick_check": True,
}
receipt_path = run_dir / "continuation_copy_receipt.json"
fd = os.open(receipt_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
with os.fdopen(fd, "w", encoding="utf-8") as stream:
    json.dump(receipt, stream, sort_keys=True)
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
directory_fd = os.open(
    run_dir, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC |
    getattr(os, "O_NOFOLLOW", 0))
try:
    os.fsync(directory_fd)
finally:
    os.close(directory_fd)
CONTINUATION_COPY_PY
    then
        echo "[续建拒绝] 旧地图或锚点原子复制失败；证据保留在 $temp_dir" >&2
        return 1
    fi
    if ! PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -I -B - \
        "$safe_reloc_engine_core" "$safe_reloc_engine_core_sha256" \
        "$continuation_anchor_copy" "$continuation_database_sha256" \
        "$safe_relocalization_session_token" \
        >> "$run_dir/continuation_prepare.log" 2>&1 <<'SAFE_RELOC_ANCHOR_CHECK_PY'
import hashlib
import importlib.util
import os
from pathlib import Path
import stat
import sys

module_path, expected_module_sha, anchors_path, database_sha, token = sys.argv[1:]
module_path = Path(module_path)
anchors_path = Path(anchors_path)
for path in (module_path, anchors_path):
    info = path.lstat()
    if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
            info.st_nlink != 1 or path.resolve(strict=True) != path):
        raise SystemExit("unsafe pure-engine or anchor path")
if hashlib.sha256(module_path.read_bytes()).hexdigest() != expected_module_sha:
    raise SystemExit("pure-engine hash mismatch")
spec = importlib.util.spec_from_file_location("wbrov_safe_reloc_engine_core", module_path)
if spec is None or spec.loader is None:
    raise SystemExit("cannot load pure engine")
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)
if getattr(core, "PURE_ENGINE_ROS_NETWORK_DISABLED", False) is not True:
    raise SystemExit("pure engine provenance mismatch")
data = core.strict_json_loads(anchors_path.read_text(encoding="utf-8"))
engine = core.SafeRelocalizationEngine(
    core.pose((0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5)),
    session_token=token)
engine.set_authoritative_epoch(0, 0)
engine.load_anchor_library(data, database_sha)
if engine.snapshot().get("persistent_tag_count", 0) <= 0:
    raise SystemExit("continuation anchor library has no persistent tags")
print("SAFE_RELOC_ANCHOR_CHECK_OK")
SAFE_RELOC_ANCHOR_CHECK_PY
    then
        echo '[续建拒绝] 私有锚点库未通过在线重定位器的严格结构校验。' >&2
        return 1
    fi
    post_receipt="$(validate_continuation_candidate "$continuation_source_run")" || return 1
    if [[ "$post_receipt" != "$current_receipt" ]]; then
        echo '[续建拒绝] 源地图在复制期间发生变化。' >&2
        return 1
    fi
    rmdir -- "$temp_dir" || return 1
    printf 'continuation_private_database_sha256=%s\ncontinuation_private_anchors_sha256=%s\n' \
        "$(sha256sum "$database" | awk '{print $1}')" \
        "$(sha256sum "$continuation_anchor_copy" | awk '{print $1}')" \
        >> "$run_dir/test_manifest.txt"
    return 0
}

verify_run_database_binding() {
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -I - \
        "$database" "$safe_relocalization_database_sha256" <<'RUN_DATABASE_BINDING_PY'
import hashlib
import os
from pathlib import Path
import stat
import sys

path = Path(sys.argv[1])
expected = sys.argv[2]
if len(expected) != 64 or any(char not in "0123456789abcdef" for char in expected):
    raise SystemExit(1)
for suffix in ("-wal", "-shm", "-journal"):
    if os.path.lexists(str(path) + suffix):
        raise SystemExit(1)
if expected == "0" * 64:
    if not os.path.lexists(path):
        raise SystemExit(0)
    info = path.lstat()
    valid = (stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid() and
             info.st_nlink == 1 and not (stat.S_IMODE(info.st_mode) & 0o022) and
             info.st_size == 0)
    raise SystemExit(0 if valid else 1)
if not os.path.lexists(path):
    raise SystemExit(1)
before = path.lstat()
if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid() or
        before.st_nlink != 1 or stat.S_IMODE(before.st_mode) & 0o022 or
        before.st_size <= 0):
    raise SystemExit(1)
flags = os.O_RDONLY | os.O_CLOEXEC | getattr(os, "O_NOFOLLOW", 0)
digest = hashlib.sha256()
descriptor = os.open(path, flags)
try:
    opened = os.fstat(descriptor)
    fields = lambda value: (value.st_dev, value.st_ino, value.st_mode, value.st_uid,
                            value.st_nlink, value.st_size, value.st_mtime_ns,
                            value.st_ctime_ns)
    if fields(opened) != fields(before):
        raise SystemExit(1)
    while True:
        block = os.read(descriptor, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    if fields(os.fstat(descriptor)) != fields(opened) or fields(path.lstat()) != fields(opened):
        raise SystemExit(1)
finally:
    os.close(descriptor)
raise SystemExit(0 if digest.hexdigest() == expected else 1)
RUN_DATABASE_BINDING_PY
}

safe_export_saved_poses() {
    local export_dir
    export_dir="$(mktemp -d "$run_dir/.safe_export.XXXXXX")" || return 1
    printf 'private_export_directory=%s\n' "$export_dir" > "$run_dir/export.log" || return 1
    if ! safe_export_guard prepare "$database" "$export_dir" >>"$run_dir/export.log" 2>&1; then
        echo "[保存未完成] 已停止源数据库的副本/已保存位姿检查失败；不重优化，保留旧PLY。诊断: $run_dir/export.log；临时目录: $export_dir" >&2
        return 1
    fi
    if ! WBROV_PRIVATE_OPTIMIZER_TRACE="$optimizer_trace_root/export" \
        "$run_exporter_binary" \
        --cloud --ascii --poses --poses_format 11 --opt 2 \
        --Optimizer/Strategy 1 --Optimizer/VarianceIgnored false \
        --output rtabmap_dense --output_dir "$export_dir" \
        --min_range "$DENSE_MIN_M" --max_range "$DENSE_MAX_M" \
        --decimation 2 --voxel 0.02 --noise_radius 0.05 --noise_k 3 \
        "$export_dir/source.db" >>"$run_dir/export.log" 2>&1; then
        echo "[导出失败] CLI非零，保留旧PLY、源数据库及临时证据: $export_dir" >&2
        return 1
    fi
    if ! safe_export_guard verify "$export_dir" >>"$run_dir/export.log" 2>&1; then
        echo "[保存未完成] 导出位姿/点云/数据库约束覆盖验证未通过；不把局部导出当整轮成功，保留旧PLY。诊断: $run_dir/export.log；临时目录: $export_dir" >&2
        return 1
    fi
    if [[ -L "$run_dir/rtabmap_dense.ply" ]] || \
       { [[ -e "$run_dir/rtabmap_dense.ply" ]] && [[ ! -f "$run_dir/rtabmap_dense.ply" ]]; }; then
        echo "[导出失败] 目标PLY不是可替换的普通文件，保留临时证据: $export_dir" >&2
        return 1
    fi
    if ! mv -T -- "$export_dir/rtabmap_dense_cloud.ply" "$run_dir/rtabmap_dense.ply"; then
        echo "[导出失败] 已验证PLY发布失败，保留临时证据: $export_dir" >&2
        return 1
    fi
    printf '已验证保存图/metric节点覆盖及非空点云；每节点点云贡献和物理精度未验收。\n点云: %s\n诊断: %s\n' \
        "$run_dir/rtabmap_dense.ply" "$export_dir/export_validation.json" || true
    return 0
}

no_exact_tag_observations() {
    # Called only after all owned children have stopped and export passed.
    # This exception permits a saved new map, never a continuation contract.
    [[ "$continuation_enabled" == false ]] || return 1
    "$python3_runtime" - "$run_dir" "$apriltag_keyframe_binder_pid" <<'PY_NO_TAG'
import json, pathlib, sys
root = pathlib.Path(sys.argv[1])
state_path = root / 'apriltag_keyframe_binding_state.json'
bindings = root / 'apriltag_keyframe_bindings.jsonl'
try:
    assert state_path.is_file() and not state_path.is_symlink()
    assert bindings.is_file() and not bindings.is_symlink()
    state = json.loads(state_path.read_text())
    assert state['schema_version'] == 1 and state['pid'] == int(sys.argv[2])
    assert state['observer_only'] is True and state['graph_edge_added'] is False
    assert state['output'] == str(bindings)
    for name in ('accepted_bindings', 'pending_info_stamps', 'pending_tag_stamps'):
        assert type(state[name]) is int and state[name] == 0
    rows = 0
    for line in bindings.read_text().splitlines():
        if not line.strip(): continue
        event = json.loads(line)
        assert event['accepted'] is False
        assert event['reason'] in ('info_without_exact_tag', 'observer_shutdown')
        assert type(event['keyframe_id']) is int and event['keyframe_id'] > 0
        assert type(event['image_stamp_ns']) is int and event['image_stamp_ns'] > 0
        assert not any('tag' in key or 'pose' in key for key in event)
        rows += 1
    assert type(state['rejected_events']) is int and state['rejected_events'] == rows
except (AssertionError, KeyError, ValueError, TypeError, OSError):
    sys.exit(1)
PY_NO_TAG
}

build_apriltag_multiview_sidecar() {
    local bindings_jsonl="$run_dir/apriltag_keyframe_bindings.jsonl"
    local output="$run_dir/apriltag_multiview_anchors.json"
    if [[ "$continuation_enabled" == true && ! -e "$bindings_jsonl" ]]; then
        # A continued run may legitimately see no new Tag.  Preserve the
        # previous exact anchors through the v4 builder instead of inventing
        # observations or dropping the library.
        (umask 077; : > "$bindings_jsonl") || return 1
    fi
    if [[ ! -f "$bindings_jsonl" || -L "$bindings_jsonl" ]]; then
        echo "[Tag锚点未生成] 本轮没有精确Tag关键帧绑定日志: $bindings_jsonl" \
            > "$run_dir/apriltag_anchor_build.log"
        return 1
    fi
    local -a previous_anchor_args=()
    if [[ "$continuation_enabled" == true ]]; then
        previous_anchor_args=(
            --previous-anchors "$continuation_anchor_copy"
            --previous-database-sha256 "$continuation_database_sha256"
            --previous-anchors-sha256 "$continuation_anchors_sha256"
        )
    fi
    if ! check_camera_health_file "$apriltag_anchor_builder" \
            "$apriltag_anchor_builder_sha256"; then
        echo '[Tag锚点未生成] anchor builder在退出期执行前身份已变化。' >&2
        return 1
    fi
    if ! PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" \
        "$run_apriltag_anchor_builder" \
        --database "$database" \
        --bindings-jsonl "$bindings_jsonl" \
        --output "$output" \
        "${previous_anchor_args[@]}" \
        >"$run_dir/apriltag_anchor_build.log" 2>&1; then
        echo "[Tag锚点未生成] 地图数据库仍保留且点云保存结果不回退；诊断: $run_dir/apriltag_anchor_build.log" >&2
        return 1
    fi
    if ! check_camera_health_file "$apriltag_anchor_builder" \
            "$apriltag_anchor_builder_sha256"; then
        echo '[Tag锚点未生成] anchor builder执行期间路径身份发生变化；拒绝续建成功状态。' >&2
        return 1
    fi
    echo "同一Tag的多视角锚点库: $output"
    return 0
}

publish_continuation_contract() {
    local current_calibration_sha256 current_runtime_node_sha256
    if ! owned_process_alive "$launcher_lock_broker_pid"; then
        echo '[续建契约未发布] 建图锁broker在发布前已丢失，不能与第二入口并发。' >&2
        return 1
    fi
    current_calibration_sha256="$(sha256_path_value "$actual_calibration_path")" || return 1
    current_runtime_node_sha256="$(sha256_path_value "$reset_runtime_node")" || return 1
    if [[ "$current_calibration_sha256" != "$actual_calibration_sha256" ||
          "$current_runtime_node_sha256" != "$reset_runtime_node_actual_sha256" ]]; then
        echo '[续建契约未发布] 实际标定或安全事务Core节点在本轮期间发生变化。' >&2
        return 1
    fi
    check_camera_health_file "$apriltag_anchor_builder" \
        "$apriltag_anchor_builder_sha256" || return 1
    check_camera_health_file "$apriltag_detector" \
        "$apriltag_detector_sha256" || return 1
    check_camera_health_file "$apriltag_keyframe_binder" \
        "$apriltag_keyframe_binder_sha256" || return 1
    check_frozen_runtime_file "$safe_reloc_gid_frontend" \
        "$safe_reloc_gid_frontend_sha256" true || return 1
    check_frozen_runtime_file "$safe_reloc_ipc_engine" \
        "$safe_reloc_ipc_engine_sha256" false || return 1
    check_frozen_runtime_file "$safe_reloc_engine_core" \
        "$safe_reloc_engine_core_sha256" false || return 1
    check_frozen_runtime_file "$safe_reloc_identity_writer" \
        "$safe_reloc_identity_writer_sha256" false || return 1
    check_reset_quarantine_runtime >/dev/null || return 1
    landmark_time_guard_check >/dev/null || return 1
    continuation_contract_tool publish "$run_dir" >/dev/null
}

cleanup() {
    local failed=0 pid name slam_status=127 slam_wait_attempted=false
    local camera_health_status=127 camera_health_wait_attempted=false
    local safe_reloc_frontend_status=127 safe_reloc_frontend_wait_attempted=false
    local -a names=(safe_reloc_gid_frontend rviz bridge camera_health cloud_frame graph_display submap_display apriltag apriltag_keyframe_binder odom tf slam timer)
    local -a pids=("$safe_reloc_gid_frontend_pid" "$rviz_pid" "$bridge_pid" "$camera_health_pid" "$cloud_frame_pid" "$graph_display_pid" "$submap_display_pid" "$apriltag_pid" "$apriltag_keyframe_binder_pid" "$odom_pid" "$tf_pid" "$slam_pid" "$timer_pid")
    if ((cleaned)); then
        return 0
    fi
    cleaned=1
	if [[ "$runtime_admitted" == true && -z "$critical_failure_reason" ]]; then
		if ! verify_critical_children_alive "cleanup entry"; then
			critical_failure_reason='critical child identity was lost at cleanup entry'
			failed=1
		fi
	fi
    # A background shell sleep may inherit SIGINT ignored; it owns no data.
    if owned_process_alive "$timer_pid"; then
        kill -TERM "$timer_pid" 2>/dev/null || true
    fi
    # Stop the single authenticated corner-recovery process while Core and all
    # sensor publishers are still alive. Its close path waits for every pending
    # proposal terminal result before writing immutable terminal evidence.
    stop_pid "$safe_reloc_gid_frontend_pid"
    if ! wait_children "$safe_reloc_gid_frontend_pid"; then
        echo '[清理未完成] 四角断图恢复进程未先退出；不继续宣称正常保存。' >&2
        failed=1
    elif [[ -n "$safe_reloc_gid_frontend_pid" &&
            -n "${owned_start_ticks[${safe_reloc_gid_frontend_pid:-0}]:-}" ]]; then
        safe_reloc_frontend_status=0
        safe_reloc_frontend_wait_attempted=true
        wait "$safe_reloc_gid_frontend_pid" || safe_reloc_frontend_status=$?
        if ((safe_reloc_frontend_status != 0)); then
            printf '[四角断图恢复退出异常] wait=%s，预期受管信号后正常退出0。\n' \
                "$safe_reloc_frontend_status" >&2
            failed=1
        elif ! safe_reloc_state_status terminal; then
            echo '[四角断图恢复退出异常] 来源、事务或清理终态证明不成立。' >&2
            failed=1
        fi
    fi
    # Take the pre-stop health snapshot only after the proposal producer is
    # gone, immediately before stopping acquisition.  A much earlier cached
    # pass could otherwise hide a final interval of odometry overload.
    if [[ "$camera_profile" == "$default_camera_profile" ]] &&
       ! camera_health_state_status live; then
        echo '[相机健康诊断未完成] 采集停机紧前的认证rolling state未通过。' >&2
        failed=1
    fi
    # Stop and reap the observer while all four authenticated publishers are
    # still alive.  Its SIGINT path performs one final endpoint/GID/identity
    # verification before publishing the terminal state.  Stopping publishers
    # first would make that proof impossible (or, worse, turn absence into a
    # false normal terminal result).
    if [[ "$camera_profile" == "$default_camera_profile" ]]; then
        if [[ -z "$camera_health_pid" || -z "${owned_start_ticks[${camera_health_pid:-0}]:-}" ]]; then
            echo '[相机健康诊断未完成] 缺少本轮observer PID实例；地图文件保留，但本轮不能成功或续建。' >&2
            failed=1
        else
            stop_pid "$camera_health_pid"
        fi
        if [[ -n "$camera_health_pid" &&
              -n "${owned_start_ticks[${camera_health_pid:-0}]:-}" ]] &&
           ! wait_children "$camera_health_pid"; then
            echo '[相机健康诊断未完成] observer在停机后未于共享期限内退出。' >&2
            failed=1
        elif [[ -n "$camera_health_pid" &&
                -n "${owned_start_ticks[${camera_health_pid:-0}]:-}" ]] &&
             pid_may_be_running "$camera_health_pid"; then
            echo '[相机健康诊断未完成] observer身份仍在运行或未知。' >&2
            failed=1
        elif [[ -n "$camera_health_pid" &&
                -n "${owned_start_ticks[${camera_health_pid:-0}]:-}" ]]; then
            camera_health_status=0
            camera_health_wait_attempted=true
            wait "$camera_health_pid" || camera_health_status=$?
            if ((camera_health_status != 130)); then
                printf '[相机健康诊断异常] observer wait=%s，预期受管SIGINT退出130；保留文件但本轮不能成功或续建。\n' \
                    "$camera_health_status" >&2
                failed=1
            elif camera_health_state_status terminal; then
                camera_health_ready_passed=true
            else
                echo '[相机健康诊断未完成] observer退出后的最终认证rolling state未通过；禁止续建成功。' >&2
                failed=1
            fi
        fi
    fi
    # Only after the observer has committed and the launcher has validated its
    # terminal proof may acquisition, visualization and SLAM be stopped.
    for pid in "$rviz_pid" "$bridge_pid" "$cloud_frame_pid" \
               "$graph_display_pid" "$submap_display_pid" "$apriltag_pid" \
               "$apriltag_keyframe_binder_pid" "$odom_pid" "$tf_pid" \
               "$slam_pid" "$timer_pid"; do
        stop_pid "$pid"
    done
    # Safety signals precede output or any export/diagnostic work.
    echo "正在停止 RTAB-Map 并验证数据库保存…"
    if [[ -n "${critical_failure_reason:-}" ]]; then
        printf '[关键进程异常] %s；不自动重启，不能报告正常结束。\n' "$critical_failure_reason" >&2
        failed=1
    fi
    if ! wait_children "${pids[@]}"; then
        echo "[清理未完成] 等待约40秒后仍有未确认退出的子进程，保留PID文件；不强杀、不宣称全部停止。" >&2
        failed=1
    else
        cleanup_nodes_stopped=true
    fi
    if [[ "$cleanup_nodes_stopped" != true ]]; then
        echo "[保存未完成] 尚有本轮子进程未确认消失；禁止读取/复制数据库或导出。" >&2
        failed=1
    elif [[ -z "$slam_pid" || -z "${owned_start_ticks[${slam_pid:-0}]:-}" ]]; then
        echo "[保存未完成] 没有可确认的本轮SLAM子进程身份；不能用数据库非空代替成功退出。" >&2
        failed=1
    elif pid_may_be_running "$slam_pid"; then
        echo "[保存未完成] SLAM PID重新出现或仍未退出；不等待/不向未知进程发信号，不读取数据库。" >&2
        failed=1
    else
        # Absence is necessary, not sufficient. Bash retains a child's actual
        # status; use || rather than 'if ! wait' which would lose that status.
        slam_status=0
        slam_wait_attempted=true
        wait "$slam_pid" || slam_status=$?
        if ((slam_status != 0)); then
            printf '[保存未完成] SLAM wait=%s（含abort/未知127均拒绝）；保留数据库，不导出。\n' "$slam_status" >&2
            failed=1
        elif [[ -s "$database" ]]; then
            if safe_export_saved_poses; then
                cleanup_export_succeeded=true
                if no_exact_tag_observations; then
                    cleanup_no_tag_available=true
                    echo '[地图已保存] 本轮没有精确Tag观测；未生成Tag续建锚点，不列入2号续建候选。'
                elif build_apriltag_multiview_sidecar; then
                    cleanup_apriltag_anchor_succeeded=true
                    if [[ "$camera_profile" == "$default_camera_profile" &&
                          "$camera_health_ready_passed" != true ]]; then
                        echo '[续建契约未发布] 有效里程计/高清ACK健康门未通过；保留本轮文件但禁止作为可信续建地图。' >&2
                        failed=1
                    elif publish_continuation_contract; then
                        cleanup_continuation_contract_succeeded=true
                    else
                        echo '[续建契约未发布] 不能把本轮标记为可续建成功。' >&2
                        failed=1
                    fi
                else
                    # A database/PLY without its database-bound multi-view Tag
                    # sidecar cannot be selected for safe continuation.  Keep
                    # all artifacts, but do not publish a successful lifecycle.
                    failed=1
                fi
            else
                failed=1
            fi
        else
            echo "[保存未完成] 正常退出但没有非空数据库，不导出。" >&2
            failed=1
        fi
    fi
    if ! printf 'slam_pid=%s\nslam_wait_attempted=%s\nslam_wait_status=%s\n' \
        "$slam_pid" "$slam_wait_attempted" "$slam_status" > "$run_dir/slam_exit_status.txt"; then
        failed=1
    fi
    if ! printf 'camera_health_pid=%s\ncamera_health_ready_passed=%s\ncamera_health_wait_attempted=%s\ncamera_health_wait_status=%s\n' \
        "$camera_health_pid" "$camera_health_ready_passed" "$camera_health_wait_attempted" \
        "$camera_health_status" > "$run_dir/camera_health_exit_status.txt"; then
        failed=1
    fi
    if ! printf 'corner_recovery_pid=%s\ncorner_recovery_wait_attempted=%s\ncorner_recovery_wait_status=%s\ncorner_recovery_config=%s\ncorner_recovery_result=%s\ncorner_recovery_cleanup=%s\n' \
        "$safe_reloc_gid_frontend_pid" "$safe_reloc_frontend_wait_attempted" \
        "$safe_reloc_frontend_status" "$run_dir/corner_recovery_config.json" \
        "$run_dir/corner_recovery/RESULT.json" "$run_dir/corner_recovery/cleanup.json" \
        > "$run_dir/safe_reloc_exit_status.txt"; then
        failed=1
    fi
    if ! printf 'anchor_builder=%s\nanchor_builder_sha256=%s\nanchor_build_succeeded=%s\nanchor_file=%s\n' \
        "$apriltag_anchor_builder" "$apriltag_anchor_builder_sha256" \
        "$cleanup_apriltag_anchor_succeeded" "$run_dir/apriltag_multiview_anchors.json" \
        > "$run_dir/apriltag_anchor_exit_status.txt"; then
        failed=1
    fi
    for name in "${!names[@]}"; do
        pid="${pids[$name]}"
        if ! pid_may_be_running "$pid"; then
            remove_matching_pid_file "${names[$name]}" "$pid" || failed=1
        else
            printf '[仍在退出] %s PID=%s\n' "${names[$name]}" "$pid" >&2
            failed=1
        fi
    done
    # Keep both PID records until the run-local stopped lifecycle is durable.
    # The broker must still hold the singleton lock throughout export, anchor
    # construction and continuation publication.
    if ! owned_process_alive "$launcher_lock_broker_pid"; then
        echo '[清理未完成] 建图锁broker在最终lifecycle之前退出；禁止报告成功。' >&2
        failed=1
    fi
    printf '本次目录: %s\n数据库路径: %s\nAprilTag地图路径: %s\n多视角Tag锚点路径: %s\n' \
        "$run_dir" "$database" "$run_dir/apriltag_map.json" \
        "$run_dir/apriltag_multiview_anchors.json"
    return "$failed"
}

publish_lifecycle() {
    local phase="$1" exit_code="$2" requested_exit_code="$3" succeeded="$4"
    local ticks boot state_temporary run_temporary
    ticks="$(process_start_ticks "$$")" || return 1
    IFS= read -r boot < /proc/sys/kernel/random/boot_id || return 1
    state_temporary="$(mktemp "$state_dir/.lifecycle.XXXXXX")" || return 1
    run_temporary="$(mktemp "$run_dir/.lifecycle.XXXXXX")" || {
        rm -f -- "$state_temporary"
        return 1
    }
    if ! printf '{"schema_version":1,"phase":"%s","launcher_pid":%s,"process_start_ticks":%s,"boot_id":"%s","nodes_stopped":%s,"export_succeeded":%s,"cleanup_succeeded":%s,"exit_code":%s,"requested_exit_code":%s}\n' \
        "$phase" "$$" "$ticks" "$boot" "$cleanup_nodes_stopped" \
        "$cleanup_export_succeeded" "$succeeded" "$exit_code" "$requested_exit_code" > "$state_temporary" ||
       ! cp -p -- "$state_temporary" "$run_temporary"; then
        rm -f -- "$state_temporary" "$run_temporary"
        return 1
    fi
    sync -f -- "$state_temporary" || {
        rm -f -- "$state_temporary" "$run_temporary"
        return 1
    }
    sync -f -- "$run_temporary" || {
        rm -f -- "$state_temporary" "$run_temporary"
        return 1
    }
    # Both files are complete 0600 temporaries. Publish global state first;
    # continuation accepts only the subsequent run-local atomic rename.
    if ! mv -fT -- "$state_temporary" "$state_dir/lifecycle.json"; then
        rm -f -- "$state_temporary" "$run_temporary"
        return 1
    fi
    sync -f -- "$state_dir" || {
        rm -f -- "$run_temporary"
        return 1
    }
    if ! mv -fT -- "$run_temporary" "$run_dir/lifecycle.json"; then
        rm -f -- "$run_temporary"
        return 1
    fi
    sync -f -- "$run_dir" || return 1
}

release_lock_broker() {
    local ack_kind='' ack_sha='' actual_sha='' broker_status=127 failed=0
    if ! owned_process_alive "$launcher_lock_broker_pid"; then
        return 1
    fi
    if ! printf 'release\t%s\n' "$run_dir" >&"$launcher_lock_broker_write_fd" ||
       ! IFS=' ' read -r -t 5 ack_kind ack_sha <&"$launcher_lock_broker_read_fd" ||
       [[ "$ack_kind" != released || ! "$ack_sha" =~ ^[0-9a-f]{64}$ ]]; then
        failed=1
    fi
    if [[ "${launcher_lock_broker_write_fd:-}" =~ ^[0-9]+$ ]]; then
        exec {launcher_lock_broker_write_fd}>&- || failed=1
    else
        failed=1
    fi
    if [[ "${launcher_lock_broker_read_fd:-}" =~ ^[0-9]+$ ]]; then
        exec {launcher_lock_broker_read_fd}<&- || failed=1
    else
        failed=1
    fi
    broker_status=0
    wait "$launcher_lock_broker_pid" || broker_status=$?
    ((broker_status == 0)) || failed=1
    if ((failed == 0)); then
        actual_sha="$(sha256_path_value "$run_dir/lock_release.json")" || failed=1
        [[ "$actual_sha" == "$ack_sha" ]] || failed=1
    fi
    return "$failed"
}

on_exit() {
    local requested_status=$? status cleanup_status=0 lifecycle_failed=0 succeeded=false
    status="$requested_status"
    # A second close/TERM must not interrupt the database save or exporter.
    trap - EXIT
    trap '' INT TERM HUP
    publish_lifecycle stopping "$status" "$requested_status" false || lifecycle_failed=1
    cleanup || cleanup_status=$?
    if ((cleanup_status == 0 && lifecycle_failed == 0)) && [[ -z "$critical_failure_reason" ]]; then
        if [[ "$cleanup_no_tag_available" == true && "$continuation_enabled" == false &&
              "$cleanup_nodes_stopped" == true && "$cleanup_export_succeeded" == true ]]; then
            # No Tag is a valid new-map outcome. Health/export checks already
            # passed in cleanup; do not publish a usable continuation receipt.
            succeeded=true
            case "$requested_status" in
                0|129|130|143) status=0 ;;
            esac
        elif [[ "$cleanup_continuation_contract_succeeded" != true ]] ||
             ! continuation_contract_tool validate "$run_dir" >/dev/null; then
            echo '[保存状态缺失] 最终续建契约不存在或不再匹配本次调用；拒绝发布成功lifecycle。' >&2
            status=1
        else
            succeeded=true
            case "$requested_status" in
                0|129|130|143) status=0 ;;
            esac
        fi
    elif ((status == 0 || status == 129 || status == 130 || status == 143)); then
        status=1
    fi
    # A second launcher must remain excluded until immediately after this
    # run-local terminal record is atomically published.
    if ! owned_process_alive "$launcher_lock_broker_pid"; then
        echo '[保存状态缺失] 最终lifecycle发布前建图锁broker身份丢失。' >&2
        status=1
        succeeded=false
    fi
    if ! publish_lifecycle stopped "$status" "$requested_status" "$succeeded"; then
        echo "[保存状态缺失] 无法写入本次最终结果，不能将退出等同于保存完成。" >&2
        ((status != 0)) || status=1
    fi
    # The broker verifies the just-published terminal lifecycle and atomically
    # signs lock_release.json before releasing flock.  Continuation rejects a
    # run lacking this receipt, closing the last point-check TOCTOU window.
    if release_lock_broker; then
        remove_matching_pid_file lock_broker "$launcher_lock_broker_pid" || status=1
        remove_matching_pid_file launcher "$$" || status=1
    else
        printf '[锁释放异常] broker未签发并确认原子release receipt；保留PID记录。\n' >&2
        status=1
    fi
    exit "$status"
}
trap on_exit EXIT
trap 'trap "" INT TERM HUP; exit 130' INT
trap 'trap "" INT TERM HUP; exit 143' TERM
trap 'trap "" INT TERM HUP; exit 129' HUP

if ! prepare_continuation_database; then
    critical_failure_reason='continuation source copy or anchor validation failed before device startup'
    exit 1
fi

echo "本次 RTAB-Map 目录: $run_dir"
echo "标定 A/B profile: $profile"
echo "标定文件: ${ZED_OPENCV_CALIBRATION_FILE:-factory}"
echo "定位输入前向深度Z: $RTAB_TRACKING_MIN_M–$RTAB_TRACKING_MAX_M m；Vis特征另受径向上限 $RTAB_TRACKING_MAX_M m 限制"
echo "单帧显示径向: $DENSE_MIN_M–$DENSE_MAX_M m；累计地图/导出前向深度Z: $DENSE_MIN_M–$DENSE_MAX_M m"
echo "AprilTag独立径向范围: $APRILTAG_MIN_RANGE_M–$APRILTAG_MAX_RANGE_M m（保留原尺寸和重投影门限）"
echo "以上为试运行筛选范围，不代表NEURAL实距精度已验收。"
echo "相机profile: $camera_profile；单一SDK owner，${camera_grab_resolution}@${camera_grab_fps} grab，SDK计算上限 ${camera_compute_cap_fps} Hz"
echo "里程计流: ${odom_width}x${odom_height}@${odom_publish_hz} Hz；建图流: ${map_width}x${map_height}@${map_publish_hz} Hz；深度模式 $RTAB_DEPTH_MODE"
if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    echo "HD1200@60是native grab/短曝光配置，不是里程计处理速率；[CAMERA_RATE] schema_version=1只证明发布侧。"
    echo "低分辨率RGB-D先发布，随后才取全分辨率帧；高清分支不能再阻塞当前里程计帧。"
    echo "本轮达标只看受管observer：实际有效/odom >=${odom_effective_health_min_hz} Hz且高清三元组与有效odom同stamp覆盖 >=90%。"
fi
if [[ "$enable_imu" == 1 ]]; then
    echo "IMU单因素候选：图像同步IMU已开启；F2M局部BA与后端使用重力方向，仅辅助横滚/俯仰，不提供平移或可信绝对航向。"
else
    echo "IMU重力候选：关闭（默认安全参数基线，不混入IMU变量）。"
fi
echo "RTAB-Map 图更新上限: $RTAB_DETECTION_RATE Hz"
if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    echo "dual建图输入为960x600@1 Hz，并只在同时间戳里程计确认后发布；5 Hz只是后端上限。"
fi
echo "视觉里程计：连续8个失败帧后在最后有效位姿重建F2M局部跟踪器；失败帧不入图，20秒健康窗只在后台记录警告。"
echo "已撤回未验收的2x3网格、0.01内点分布门和80px搜索窗，恢复RTAB-Map默认1x1/0/40。"
echo "失锁恢复模式：新段先隔离建图；同一Tag需3个精确关键帧视角，并由核心做多桥影子优化、留出与刚体保平面门后才允许合图。"
if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    echo "dual profile使用单槽RGB-D输入队列以限制延迟；若有效/odom低于15Hz，本轮不算通过。"
fi
echo "运动速度平移外推已关闭，避免失锁后预测位移无限增长；仍保留同步IMU的帧间旋转预测。"
echo "恢复未获得核心同token提交回执前不能冒充已定位；普通跨epoch回环不能绕过安全事务入口。"
echo "安全事务不降低里程计15内点门，也不把单帧、单Tag视角或裸/add_link当成定位成功。"
echo "独立图结构显示: /mapGraph → /mapping/graph_display；显示节点失效也会停止并保存，不自动重启。"
echo "建图更新 2026-10-15：主图隔离、临时子图、AprilTag位置标注；无Tag自动接回和实时性仍待验收。"
echo "未对齐子图保存在独立坐标；对应RViz配置将在 $run_dir/submap_views 生成。"

echo "标签不作为原生6DoF Landmark边；只通过受token/epoch约束的三视角用户闭环事务，并在正常停止后保存/继承精确多视角锚点。"
echo "视觉里程计局部地图容量: 2000 words；空间邻近约束关闭，仅保留经视觉验证的同段回环候选。"
echo "建图后端为g2o=1；导出approach=2只读已保存优化位姿（不是后端编号），不允许回退重优化。"
echo "只有全部本轮子进程退出、SLAM wait=0且副本位姿/点云/数据库约束覆盖核对通过，才报告保存成功；失败保留旧PLY及诊断。"
echo "AprilTag 36h11 黑色方框边长: $APRILTAG_SIZE_M m"
echo "RViz2 显示点云、标签识别画面和标签地图；关闭 RViz2 即停止并保存。"

if [[ "$light_mode" == teleop ]]; then
    verify_production_teleop_light_owner "$teleop_pid"
    echo "灯光由生产 teleop PID=$teleop_pid 管理；隔离建图不发送外设命令。"
else
    echo "明确无灯测试：隔离建图不检查 teleop，也不发送灯光/外设命令。"
fi

# The camera publisher has no GUI. NoMachine DISPLAY creates an invalid
# Argus EGL context; clear display variables only for this child so RViz
# and the operator control window retain their desktop display. env exec
# preserves the PID for record_child and exact executable monitoring.
bridge_command=(/usr/bin/env -u DISPLAY -u WAYLAND_DISPLAY "$test_binary")
echo "RGB-D 发布器: ${bridge_command[*]}"
if ! owned_process_alive "$launcher_lock_broker_pid"; then
    critical_failure_reason='launcher lock broker identity was lost before camera startup'
    exit 1
fi
check_camera_profile_binary || {
    critical_failure_reason='camera bridge identity changed after preflight'
    exit 1
}
"${bridge_command[@]}" >"$run_dir/zed_rgbd.log" 2>&1 &
bridge_pid=$!
record_child bridge "$bridge_pid" "$test_binary" "$test_binary_expected_sha256"

check_frozen_runtime_file "$static_tf_binary" "$static_tf_binary_sha256" true || {
    critical_failure_reason='static TF executable identity changed before exec'
    exit 1
}
"$static_tf_binary" \
    --x 0 --y 0 --z 0 \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
    --frame-id base_link \
    --child-frame-id zed_left_camera_optical_frame \
    >"$run_dir/static_tf.log" 2>&1 &
tf_pid=$!
record_child tf "$tf_pid" "$static_tf_binary" "$static_tf_binary_sha256"

export APRILTAG_LOG="$run_dir/apriltag_detections.csv"
export APRILTAG_MAP_JSON="$run_dir/apriltag_map.json"
check_camera_health_file "$apriltag_detector" "$apriltag_detector_sha256" || {
    critical_failure_reason='AprilTag detector identity changed after preflight'
    exit 1
}
PYTHONNOUSERSITE=1 "$python3_runtime" \
    "$run_apriltag_detector" \
    "${apriltag_camera_profile_args[@]}" \
    >"$run_dir/apriltag.log" 2>&1 &
apriltag_pid=$!
record_child apriltag "$apriltag_pid" "$python3_runtime" "$python3_runtime_sha256" "$run_apriltag_detector"

check_frozen_runtime_file "$run_graph_display_script" "$graph_display_script_sha256" false || exit 1
check_frozen_runtime_file "$run_graph_display_module" "$graph_display_module_sha256" false || exit 1
check_frozen_runtime_file "$run_submap_display_module" "$submap_display_module_sha256" false || exit 1
WBROV_POSE_DIAGNOSTICS_PATH="$run_dir/pose_diagnostics.jsonl" \
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" \
    "$run_graph_display_script" --ros-args -r __node:=rtabmap_graph_display \
    >"$run_dir/graph_display.log" 2>&1 &
graph_display_pid=$!
record_child graph_display "$graph_display_pid" "$python3_runtime" "$python3_runtime_sha256" "$run_graph_display_script"

# The passive local-submap/Tag display has the same owned-process lifecycle.
check_frozen_runtime_file "$submap_display_script" "$submap_display_script_sha256" false || exit 1
mkdir -m 700 -- "$run_dir/submap_views"
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -B \
    "$submap_display_script" --rviz-output-dir "$run_dir/submap_views" \
    >"$run_dir/submap_display.log" 2>&1 &
submap_display_pid=$!
record_child submap_display "$submap_display_pid" "$python3_runtime" "$python3_runtime_sha256" "$submap_display_script"


initial_frontend_stable=true
for ((frontend_attempt=0; frontend_attempt<10; frontend_attempt++)); do
    if ! owned_process_alive "$bridge_pid" || ! owned_process_alive "$tf_pid" ||
       ! owned_process_alive "$apriltag_pid" ||
       ! owned_process_alive "$graph_display_pid" || ! owned_process_alive "$submap_display_pid" ||
       ! owned_process_alive "$launcher_lock_broker_pid"; then
        initial_frontend_stable=false
        break
    fi
    sleep 0.1
done
if [[ "$initial_frontend_stable" != true ]]; then
    if ! owned_process_alive "$bridge_pid"; then
        critical_failure_reason='ZED相机进程退出或身份失效；请查看下面zed_rgbd.log中的开流错误。'
    else
        critical_failure_reason='TF、AprilTag、显示节点或锁管理进程未通过启动检查。'
    fi
    echo "相机、TF、AprilTag或图结构显示节点未稳定启动：" >&2
    tail -n 120 -- "$run_dir/zed_rgbd.log" "$run_dir/static_tf.log" \
        "$run_dir/apriltag.log" "$run_dir/graph_display.log" "$run_dir/submap_display.log" >&2
    exit 1
fi

check_reset_quarantine_runtime >"$run_dir/reset_runtime_odom_preexec_receipt.txt" || {
    critical_failure_reason='odometry executable identity changed before exec'
    exit 1
}
WBROV_DEPTH_RECOVERY=1 WBROV_PRIVATE_OPTIMIZER_TRACE="$optimizer_trace_root/odom" \
LD_LIBRARY_PATH="$managed_runtime_ld_library_path" \
"$odom_binary" \
    --ros-args \
    -r __node:=rtabmap_rgbd_odometry \
    "${odom_transport_args[@]}" \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=true \
    -p managed_external_reset_disabled:=true \
    -p approx_sync:=false \
    -p sync_queue_size:=10 \
    -p qos:=2 \
    -p qos_camera_info:=2 \
    -p publish_null_when_lost:=false \
    -p "Vis/MaxDepth:='$RTAB_TRACKING_MAX_M'" \
    -p "Vis/MinInliers:='15'" \
    -p "Vis/DepthAsMask:='true'" \
    -p "GFTT/MinDistance:='3'" \
    -p "Vis/GridRows:='2'" \
    -p "Vis/GridCols:='3'" \
    -p "Vis/CorGuessMatchToProjection:='true'" \
    -p "Odom/ResetCountdown:='8'" \
    -p "Odom/GuessMotion:='false'" \
    -p "OdomF2M/MaxSize:='2000'" \
    "${odom_camera_profile_args[@]}" \
    "${odom_imu_args[@]}" \
    >"$run_dir/odometry.log" 2>&1 &
odom_pid=$!
record_child odom "$odom_pid" "$odom_binary" "$odom_binary_sha256"

check_reset_quarantine_runtime >"$run_dir/reset_runtime_preexec_receipt.txt" || {
    critical_failure_reason='safe transaction runtime identity changed before exec'
    exit 1
}
verify_run_database_binding >"$run_dir/database_preexec_binding_receipt.txt" || {
    critical_failure_reason='run-local database identity/SHA changed immediately before Core exec'
    exit 1
}
WBROV_PRIVATE_OPTIMIZER_TRACE="$optimizer_trace_root/slam" \
LD_LIBRARY_PATH="$managed_runtime_ld_library_path" \
"$reset_runtime_node" \
    --ros-args \
    -r __node:=rtabmap \
    -r "rgb/image:=$map_rgb_topic" \
    -r "depth/image:=$map_depth_topic" \
    -r "rgb/camera_info:=$map_info_topic" \
    -r odom:=/odom \
	-r reset_quarantine_odom:=/odom \
    -r reset_quarantine_odom_info:=/odom_info \
	-r "reset_quarantine_rgb:=$map_rgb_topic" \
	-r "reset_quarantine_depth:=$map_depth_topic" \
	-r "reset_quarantine_camera_info:=$map_info_topic" \
	-r odom_info:=/odom_info \
	"${slam_imu_args[@]}" \
    -r landmark_detections:=/landmark_detections \
    -p database_path:="$database" \
    -p frame_id:=base_link \
    -p map_frame_id:=map \
    -p subscribe_depth:=true \
    -p subscribe_rgb:=true \
    -p subscribe_scan:=false \
	-p subscribe_odom_info:=true \
	-p odom_sensor_sync:=false \
    -p approx_sync:=false \
    -p topic_queue_size:="$slam_topic_queue_size" \
    -p sync_queue_size:="$slam_sync_queue_size" \
    -p qos_image:=1 \
    -p qos_camera_info:=1 \
    -p qos_odom:=2 \
    -p wait_for_transform:=0.5 \
    -p latch:=true \
    -p reset_quarantine_enabled:=true \
    -p mapping_continuity_vo_width:="$odom_width" \
    -p mapping_continuity_vo_height:="$odom_height" \
    -p reset_quarantine_lost_count:=8 \
    -p reset_quarantine_odom_info_max_age_sec:=0.5 \
    -p reset_quarantine_max_unannounced_translation:=0.5 \
    -p reset_quarantine_max_unannounced_rotation_deg:=30.0 \
    -p reset_quarantine_expected_odom_info_node:=rtabmap_rgbd_odometry \
	-p reset_quarantine_expected_odom_namespace:=/ \
	-p reset_quarantine_expected_sensor_node:=zed_rgbd_dual_stream_publisher \
	-p reset_quarantine_expected_sensor_namespace:=/ \
	-p reset_quarantine_expected_sensor_frame:=zed_left_camera_optical_frame \
	-p "reset_quarantine_expected_sensor_to_base:='0 0 0 -0.5 0.5 -0.5 0.5'" \
    -p safe_relocalization_enabled:=true \
	-p safe_relocalization_expected_proposal_node:=safe_reloc_gid_frontend \
	-p safe_relocalization_expected_proposal_namespace:=/ \
    -p "safe_relocalization_session_token:='$safe_relocalization_session_token'" \
    -p "safe_relocalization_database_sha256:='$safe_relocalization_database_sha256'" \
    -p safe_relocalization_start_new_map_on_first_update:="$safe_relocalization_start_new_map_on_first_update" \
    -p safe_relocalization_timeout_sec:=30.0 \
    -p safe_relocalization_max_translation:=0.08 \
    -p safe_relocalization_max_rotation:=8.0 \
    -p safe_relocalization_min_bridge_translation:=0.02 \
    -p safe_relocalization_min_bridge_rotation:=2.0 \
    -p safe_relocalization_max_anchor_z:=0.02 \
    -p safe_relocalization_max_anchor_tilt:=2.0 \
    -p safe_relocalization_max_old_pose_translation:=0.005 \
    -p safe_relocalization_max_old_pose_rotation:=0.5 \
    -p safe_relocalization_max_new_rigid_translation:=0.03 \
    -p safe_relocalization_max_new_rigid_rotation:=3.0 \
    -p safe_relocalization_max_new_rigid_z_error:=0.02 \
    -p safe_relocalization_max_new_rigid_tilt_error:=2.0 \
    -p safe_relocalization_max_gravity_residual_increase:=0.5 \
    -p safe_relocalization_max_information_condition:=100000000.0 \
    -p "Rtabmap/DetectionRate:='$RTAB_DETECTION_RATE'" \
    -p "Rtabmap/StartNewMapOnLoopClosure:='false'" \
    -p "Rtabmap/CrossMapLoopPending:='false'" \
    -p "RGBD/LinearUpdate:='0.05'" \
    -p "RGBD/AngularUpdate:='0.05'" \
    -p "RGBD/NeighborLinkRefining:='false'" \
    -p "RGBD/ProximityBySpace:='false'" \
    -p "RGBD/OptimizeMaxError:='1.0'" \
    -p "RGBD/OptimizeMaxErrorRepairRadius:='0'" \
    -p "RGBD/CreateOccupancyGrid:='true'" \
    -p "Optimizer/LandmarksIgnored:='true'" \
    -p "Optimizer/PriorsIgnored:='true'" \
    -p "Rtabmap/LoopGPS:='false'" \
    -p "Reg/Force3DoF:='false'" \
    -p "Optimizer/Iterations:='20'" \
    -p "Optimizer/Strategy:='1'" \
    -p "Optimizer/VarianceIgnored:='false'" \
    -p "Mem/IncrementalMemory:='true'" \
    -p "Mem/InitWMWithAllNodes:='true'" \
    -p "Mem/ReduceGraph:='false'" \
    -p "Mem/RehearsalIdUpdatedToNewOne:='false'" \
    -p "Mem/RehearsalSimilarity:='1.0'" \
    -p "Rtabmap/TimeThr:='0'" \
    -p "Rtabmap/MemoryThr:='0'" \
	-p "Rtabmap/CreateIntermediateNodes:='false'" \
    "${slam_gravity_args[@]}" \
    -p "Mem/SaveDepth16Format:='true'" \
    -p "Vis/MaxDepth:='$RTAB_TRACKING_MAX_M'" \
    -p "Vis/MinInliers:='25'" \
    -p "Grid/RangeMin:='$DENSE_MIN_M'" \
    -p "Grid/RangeMax:='$DENSE_MAX_M'" \
    -p "Grid/CellSize:='0.03'" \
    -p "Grid/DepthDecimation:='2'" \
    -p "Grid/NoiseFilteringRadius:='0.05'" \
    -p "Grid/NoiseFilteringMinNeighbors:='3'" \
    >"$run_dir/rtabmap.log" 2>&1 &
slam_pid=$!
record_child slam "$slam_pid" "$reset_runtime_node" "$reset_runtime_node_sha256"

wait_for_runtime_process_maps "$odom_pid" odom || {
    critical_failure_reason='odometry process did not map the complete private RTAB-Map runtime'
    exit 1
}
wait_for_runtime_process_maps "$slam_pid" slam || {
    critical_failure_reason='SLAM process did not map the complete private RTAB-Map runtime'
    exit 1
}

check_camera_health_file "$apriltag_keyframe_binder" "$apriltag_keyframe_binder_sha256" || {
    critical_failure_reason='AprilTag keyframe binder identity changed after preflight'
    exit 1
}
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" \
    "$run_apriltag_keyframe_binder" \
    --output "$run_dir/apriltag_keyframe_bindings.jsonl" \
    --state "$run_dir/apriltag_keyframe_binding_state.json" \
    --max-pending-age-s 2.5 \
    >"$run_dir/apriltag_keyframe_binder.log" 2>&1 &
apriltag_keyframe_binder_pid=$!
record_child apriltag_keyframe_binder "$apriltag_keyframe_binder_pid" \
    "$python3_runtime" "$python3_runtime_sha256" "$run_apriltag_keyframe_binder"

check_frozen_runtime_file "$corner_recovery_consumer" "$corner_recovery_consumer_sha256" false || {
    critical_failure_reason='corner recovery consumer identity changed after preflight'
    exit 1
}
check_frozen_runtime_file "$corner_recovery_renderer" "$corner_recovery_renderer_sha256" false || {
    critical_failure_reason='corner recovery config renderer identity changed after preflight'
    exit 1
}
check_frozen_runtime_file "$corner_recovery_template" "$corner_recovery_template_sha256" false || {
    critical_failure_reason='corner recovery config template identity changed after preflight'
    exit 1
}
check_frozen_runtime_file "$corner_recovery_message_info" "$corner_recovery_message_info_sha256" false || {
    critical_failure_reason='corner recovery native metadata shim identity changed after preflight'
    exit 1
}

PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$python3_runtime" -B \
    "$corner_recovery_renderer" \
    --template "$corner_recovery_template" \
    --output "$run_dir/corner_recovery_config.json" \
    --session-token "$safe_relocalization_session_token" \
    --database-sha256 "$safe_relocalization_database_sha256" \
    >"$run_dir/corner_recovery_config_receipt.json" || {
        critical_failure_reason='could not render the run-bound corner recovery config'
        exit 1
    }

PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
LD_LIBRARY_PATH="$managed_runtime_ld_library_path" \
    "$python3_runtime" -B "$corner_recovery_consumer" \
    "$run_dir/corner_recovery_config.json" "$run_dir/corner_recovery" \
    >"$run_dir/corner_recovery.log" 2>&1 &
safe_reloc_gid_frontend_pid=$!
record_child safe_reloc_gid_frontend "$safe_reloc_gid_frontend_pid" \
    "$python3_runtime" "$python3_runtime_sha256" "$corner_recovery_consumer"

# BEGIN RECOVERY_STARTUP_ADMISSION
safe_relocalization_handshake_ready=false
# /proc/uptime is monotonic. Include process/source checks in the budget;
# counting sleep calls made the old "30 second" wait last over 100 seconds.
read -r handshake_uptime _ < /proc/uptime
handshake_deadline=$(( ${handshake_uptime%%.*} + 30 ))
echo '正在确认恢复模块的数据来源和图状态；完整关键帧配对将在后台等待，不阻挡RViz。'
while :; do
    read -r handshake_uptime _ < /proc/uptime
    (( ${handshake_uptime%%.*} < handshake_deadline )) || break
    if ! owned_process_alive "$bridge_pid" || ! owned_process_alive "$tf_pid" || \
       ! owned_process_alive "$apriltag_pid" || ! owned_process_alive "$graph_display_pid" || ! owned_process_alive "$submap_display_pid" || \
       ! owned_process_alive "$odom_pid" || ! owned_process_alive "$slam_pid" || \
       ! owned_process_alive "$apriltag_keyframe_binder_pid" || \
       ! owned_process_alive "$safe_reloc_gid_frontend_pid" || \
       ! owned_process_alive "$launcher_lock_broker_pid"; then
        echo 'RTAB-Map、锁broker或AprilTag关键帧/四角断图恢复进程启动失败：' >&2
        tail -n 100 -- "$run_dir/odometry.log" "$run_dir/rtabmap.log" \
            "$run_dir/apriltag_keyframe_binder.log" "$run_dir/corner_recovery.log" >&2
        exit 1
    fi
    if safe_reloc_state_status admitted; then
        safe_relocalization_handshake_ready=true
        break
    else
        handshake_status=$?
        if ((handshake_status == 1)); then
            echo '[启动失败] 四角断图恢复的进程、配置、来源或状态证据损坏。' >&2
            exit 1
        fi
    fi
    sleep 0.1
done
if [[ "$safe_relocalization_handshake_ready" != true ]]; then
    critical_failure_reason='Recovery source admission or checked Core graph state unavailable within 30 seconds'
    echo '[启动失败] 30秒内未完成恢复模块的数据来源与Core图状态检查。' >&2
    exit 1
fi
# END RECOVERY_STARTUP_ADMISSION

if ! landmark_time_guard_mapped "$slam_pid" "${owned_start_ticks[$slam_pid]:-}" \
    "$landmark_time_guard_receipt" >"$run_dir/landmark_time_guard_mapping.json"; then
    critical_failure_reason='SLAM private tag-time conversion library mapping or recorded instance is unverified'
    echo '[标签时间守卫失效] 私有转换库映射验证失败，按原流程停止保存；不自动重启。' >&2
    exit 1
fi

if [[ "$camera_profile" == "$default_camera_profile" ]]; then
    check_frozen_runtime_file "$camera_health_observer" "$camera_health_observer_sha256" true || {
        critical_failure_reason='camera health observer identity changed before exec'
        exit 1
    }
    LD_LIBRARY_PATH="$managed_runtime_ld_library_path" "$camera_health_observer" \
        --duration-s 0 \
        --window-s "$camera_health_window_s" \
        --emit-period-s 1 \
        --odom-min-hz "$camera_health_odom_min_hz" \
        --high-min-hz "$camera_health_high_min_hz" \
        --coverage-min "$camera_health_coverage_min" \
        --integration-min-hz "$camera_health_integration_min_hz" \
        --expected-high-width "$map_width" \
        --expected-high-height "$map_height" \
        --odom-topic /odom \
        --odom-info-topic /odom_info_lite \
        --high-camera-info-topic "$map_info_topic" \
        --rtabmap-info-topic /info \
        --expected-odom-node rtabmap_rgbd_odometry \
        --expected-odom-info-node rtabmap_rgbd_odometry \
        --expected-high-node zed_rgbd_dual_stream_publisher \
        --expected-info-node rtabmap \
        --expected-node-namespace / \
        --require-integration-proxy \
        --jsonl "$run_dir/header_observer.jsonl" \
        --state-json "$run_dir/header_observer_state.json" \
        >"$run_dir/header_observer.log" 2>&1 &
    camera_health_pid=$!
    record_child camera_health "$camera_health_pid" \
        "$camera_health_observer" "$camera_health_observer_sha256"
    camera_health_stable=true
    for ((camera_health_attempt=0; camera_health_attempt<10; camera_health_attempt++)); do
        if ! owned_process_alive "$camera_health_pid" ||
           ! owned_process_alive "$launcher_lock_broker_pid"; then
            camera_health_stable=false
            break
        fi
        sleep 0.1
    done
    if [[ "$camera_health_stable" != true ]]; then
        critical_failure_reason='camera health observer exited during startup admission'
        tail -n 100 -- "$run_dir/header_observer.log" >&2 || true
        exit 1
    fi
    echo "相机/里程计健康验收在后台滚动；RViz不等待20秒，失败不关闭窗口，但退出时禁止成功续建。"
fi

cloud_frame_bin="$cloud_frame_binary"
if ! check_frozen_runtime_file "$cloud_frame_bin" "$cloud_frame_binary_sha256" true; then
    echo "缺少 RTAB-Map 单帧点云工具，无法向 RViz 发布实时 RGB-D 点云。"
    exit 1
fi

"$cloud_frame_bin" \
    --ros-args \
    -r __node:=rtabmap_cloud_frame \
    -r "rgb/image:=$odom_rgb_topic" \
    -r "depth/image:=$odom_display_depth_topic" \
    -r "rgb/camera_info:=$odom_info_topic" \
    -r cloud:=/rtabmap/frame_cloud \
    -p approx_sync:=false \
    -p sync_queue_size:=10 \
    -p qos:=2 \
    -p qos_camera_info:=2 \
    -p decimation:=2 \
    -p min_depth:="$frame_min_depth" \
    -p max_depth:="$frame_max_depth" \
    -p voxel_size:=0.02 \
    -p noise_filter_radius:=0.05 \
    -p noise_filter_min_neighbors:=3 \
    -p filter_nans:=true \
    >"$run_dir/cloud_frame.log" 2>&1 &
cloud_frame_pid=$!
record_child cloud_frame "$cloud_frame_pid" "$cloud_frame_binary" "$cloud_frame_binary_sha256"

cloud_frame_stable=true
for ((cloud_attempt=0; cloud_attempt<10; cloud_attempt++)); do
    if ! owned_process_alive "$cloud_frame_pid" ||
       ! owned_process_alive "$launcher_lock_broker_pid"; then
        cloud_frame_stable=false
        break
    fi
    sleep 0.1
done
if [[ "$cloud_frame_stable" != true ]]; then
    echo "单帧 RGB-D 点云节点启动失败："
    tail -n 100 -- "$run_dir/cloud_frame.log"
    exit 1
fi

runtime_admitted=true

echo "RViz 累计地图将直接使用 /cloud_map；回环优化后会按节点新位姿重建。"
echo "本轮已记录同步位姿诊断；窗口打开后先静止10秒，再平稳平移约30秒，便于核对位姿稳定性。"

duration_s="$caller_duration_s"
if awk -v duration="$duration_s" 'BEGIN {exit !(duration > 0)}'; then
    echo "无界面定时测试 ${duration_s} 秒；结束后自动保存。"
    check_frozen_runtime_file "$sleep_binary" "$sleep_binary_sha256" true || exit 1
    "$sleep_binary" "$duration_s" &
    timer_pid=$!
    record_child timer "$timer_pid" "$sleep_binary" "$sleep_binary_sha256"
    wait_for_display_session "$timer_pid" timer
else
    export DISPLAY="${DISPLAY:-:0}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
    export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"
    export XAUTHORITY="${XAUTHORITY:-$XDG_RUNTIME_DIR/gdm/Xauthority}"
    # Keep the desktop's native Qt/XCB OpenGL integration. Forcing the
    # "none" backend can leave both RViz Image and OGRE panels black even
    # while the subscribed RGB and PointCloud2 messages are valid.
    check_frozen_runtime_file "$rviz_binary" "$rviz_binary_sha256" true || exit 1
    check_frozen_runtime_file "$run_rviz_config" "$test_rviz_sha256" false || exit 1
    LD_LIBRARY_PATH="$managed_runtime_ld_library_path:/opt/ros/humble/opt/rviz_ogre_vendor/lib" \
    "$rviz_binary" -d "$run_rviz_config" &
    rviz_pid=$!
    record_child rviz "$rviz_pid" "$rviz_binary" "$rviz_binary_sha256" "$run_rviz_config"
    wait_for_display_session "$rviz_pid" rviz
fi
