#!/usr/bin/bash
# Isolated candidate: no SDK instance, no calibration capture, no motion command.
set -euo pipefail
set +m  # A background setsid child must exec in-place, not fork a job leader.
umask 077

# Resolve every helper from a closed, root-owned path before the first external
# command.  The wrapper never inherits Python/ROS/library search paths from a
# shell which launched the desktop entry.
readonly wrapper_system_path='/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
readonly wrapper_python3_runtime='/usr/bin/python3.10'
readonly wrapper_python3_runtime_sha256='03bb5d246e83c44204ef38044b062fdc46c2835881874e763c44c48b4304c490'
readonly wrapper_bash_runtime='/usr/bin/bash'
readonly wrapper_bash_runtime_sha256='c5f8a98c674631609902846fae6df219b3b16d97db58cf1c1334f8eb14962bde'
export PATH="$wrapper_system_path"
if [[ -n "${BASH_ENV:-}" || -n "${ENV:-}" ]]; then
    printf '%s\n' '[拒绝启动/继续] 请从受审8号桌面入口启动；Bash启动环境未清理。' >&2
    exit 1
fi
if [[ -n "${LD_PRELOAD:-}" || -n "${LD_AUDIT:-}" ]]; then
    printf '%s\n' '[拒绝启动/继续] 调用方设置了LD_PRELOAD或LD_AUDIT。' >&2
    exit 1
fi
unset BASH_ENV ENV AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
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

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
validator="$script_dir/validate_mapping_light_state.py"
lifecycle_validator="$script_dir/validate_mapping_lifecycle.py"
teleop_launcher="/home/exocean/Desktop/机器人键盘遥控/.tools/run_water.sh"
teleop_script="/home/exocean/code/poa-aco-integration/integration/teleop_keyboard.py"
normal_mapping_launcher="/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/start_rtabmap_calib_ab.sh"
old_map_localization_launcher="/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/old_map_localization/start_rtabmap_old_map_localization.sh"
default_localization_source_db="/home/exocean/maps/rtabmap_calibration_ab/custom_20260409_194923/rtabmap.db"
# Release tooling replaces these three sentinels only after the paired
# Core/ROS runtime and launcher are frozen.  Leaving any sentinel in place is a
# deliberate fail-closed state, not a development fallback.
readonly FINAL_MAPPING_LAUNCHER_SHA256='8b6adf33e0550465ce5e4f10ffa7d8e5f5d43db022084c2496aa81b8b3de8bb3'
readonly FINAL_RESET_RUNTIME_ROOT='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/rtabmap_runtime_candidates/sha256-74d38aba069fcfbb0872b705ca44b6ea5b95e96c5362a4f1aa214e44bb04a77f'
readonly FINAL_RESET_RUNTIME_NODE_SHA256='204ef15544749422549ebff440b05a3d148c2e7527f990e67f7431a7b2306516'
readonly FINAL_RESET_RUNTIME_NODE="$FINAL_RESET_RUNTIME_ROOT/bin/rtabmap"
# The full immutable-library checks alone take about 27s before any node starts.
# Bound the complete preflight + node startup, including repeated pre-exec checks.
readonly mapping_child_admission_timeout_s=180
readonly mapping_child_admission_max_polls=720
mapping_mode="${WBROV_MAPPING_MODE:-}"
localization_source_db="${WBROV_LOCALIZATION_SOURCE_DB:-}"
continuation_run="${WBROV_MAPPING_CONTINUE_RUN-latest}"
continuation_run_explicit=0
[[ ${WBROV_MAPPING_CONTINUE_RUN+x} ]] && continuation_run_explicit=1
mapping_launcher="$normal_mapping_launcher"
mapping_launcher_pattern='start_rtabmap_calib_ab\.sh'
calibration_file="/home/exocean/calibration/zedx_charuco/water_calib_trial_20260827_from_unique86/results/run_c2ce22daf28d/zed_opencv_calibration_40408468.yml"
calibration_expected_sha256="d0a5f4486794add80330be1b89bea5aa890d3fed124456418ff1c98a59708389"
control_state_dir="$HOME/.cache/wbrov_control/mapping_light"
ready_file="$control_state_dir/ready.json"
state_dir="$HOME/.cache/wbrov_water_mapping"
mapping_state_dir="$HOME/.cache/rtabmap_dense_calib_ab"
mapping_lifecycle_file="$mapping_state_dir/lifecycle.json"
brightness="${WBROV_MAPPING_LIGHT_BRIGHTNESS:-30}"
settle_s="${WBROV_MAPPING_LIGHT_SETTLE_S:-4}"
check_only=0
range_snapshot=""
current_uid="$(id -u)"
IFS= read -r boot_id < /proc/sys/kernel/random/boot_id
teleop_pid="" teleop_ticks="" mapping_pid="" mapping_ticks=""
spawn_monotonic="" run_dir="" cleanup_started=0
failure_reason=""
mapping_stop_requested=0 mapping_reaped=0 mapping_exit_status=0
final_core_realpath="" final_core_device="" final_core_inode=""
mapping_launcher_realpath="" mapping_launcher_device="" mapping_launcher_inode=""
run_mapping_launcher="" run_mapping_launcher_device="" run_mapping_launcher_inode=""
wrapper_bash_device="" wrapper_bash_inode=""
declare -A child_pids=() child_ticks=()
declare -A frozen_paths=(
    [teleop_launcher]="$teleop_launcher"
    [teleop_script]="$teleop_script"
    [mapping_launcher]="$mapping_launcher"
    [calibration_file]="$calibration_file"
)
declare -A frozen_hashes=()
frozen_names=(teleop_launcher teleop_script mapping_launcher calibration_file)
declare -A child_patterns=(
    # Accept the frozen legacy bridges and the current rate-controlled HD1200 bridge.
    # This is an identity check on the recorded PID, not a process-name search.
    [bridge]='zed_rgbd_publisher_(calib_test|dual_stream_candidate|rate_ab)'
    [tf]='static_transform_publisher'
    [apriltag]='apriltag_landmark_node\.py'
    [apriltag_keyframe_binder]='apriltag_keyframe_binder\.py'
    [safe_reloc_gid_frontend]='wbrov_corner_recovery/consumer\.py'
    [safe_reloc_ipc_engine]='safe_reloc_ipc_engine\.py'
    [camera_health]='(^|[ /])header_health_observer( |$)'
    [graph_display]='rtabmap_graph_display\.py'
    [submap_display]='rtabmap_submap_display\.py'
    [apriltag_seed]='apriltag_initialpose_seed\.py'
    [localization_status]='localization_status_observer\.py'
    [odom]='rgbd_odometry'
    [slam]='rtabmap_slam/rtabmap'
    [cloud_frame]='point_cloud_xyzrgb'
    [lock_broker]='rtabmap_dense_calib_ab/launcher\.lock'
)
declare -a expected_mapping_children=()

fail() {
    failure_reason="$*"
    # Once a child exists, even a blocked stdout/stderr must not delay the
    # safety signals. cleanup prints this reason only after both signal steps.
    if [[ -z "$teleop_pid" && -z "$mapping_pid" ]]; then
        printf '[拒绝启动/继续] %s\n' "$failure_reason" >&2
    fi
    exit 1
}
usage() { echo "用法: $0 [--check-only] [--mode new|continue|localize-old] [--continue-run latest|/绝对运行目录] [--source-db /绝对路径/rtabmap.db] [--brightness 1..100] [--settle 3..5]"; }
while (($#)); do
    case "$1" in
        --brightness) (($# >= 2)) || { usage; exit 2; }; brightness="$2"; shift 2 ;;
        --settle) (($# >= 2)) || { usage; exit 2; }; settle_s="$2"; shift 2 ;;
        --mode) (($# >= 2)) || { usage; exit 2; }; mapping_mode="$2"; shift 2 ;;
        --continue-run) (($# >= 2)) || { usage; exit 2; }; continuation_run="$2"; continuation_run_explicit=1; shift 2 ;;
        --source-db) (($# >= 2)) || { usage; exit 2; }; localization_source_db="$2"; shift 2 ;;
        --check-only) check_only=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) usage; exit 2 ;;
    esac
done

select_mapping_mode() {
    local answer selected_source
    if [[ -z "$mapping_mode" ]]; then
        if ((check_only)); then
            mapping_mode=mapping
        else
            printf '\n请选择运行方式：\n  1) 新建一张地图\n  2) 续建最近一次正常保存的地图（失锁段经三视角Tag安全合并）\n  3) 旧地图只定位（不新增节点、不覆盖旧地图）\n'
            read -r -p '输入 1、2 或 3：' answer
            case "$answer" in
                1) mapping_mode=new ;;
                2) mapping_mode=continue ;;
                3) mapping_mode=localize-old ;;
                *) fail '运行方式只能选择1、2或3。' ;;
            esac
        fi
    fi
    case "$mapping_mode" in
        mapping|new)
            mapping_mode=new
            [[ -z "$localization_source_db" ]] || fail '正常建图模式不能同时传--source-db。'
            ((continuation_run_explicit == 0)) || fail '新建地图模式不能同时传--continue-run。'
            mapping_launcher="$normal_mapping_launcher"
            mapping_launcher_pattern='start_rtabmap_calib_ab\.sh'
            expected_mapping_children=(lock_broker bridge tf apriltag graph_display submap_display apriltag_keyframe_binder safe_reloc_gid_frontend odom slam camera_health cloud_frame)
            ;;
        continue)
            [[ -z "$localization_source_db" ]] || fail '续建模式不能同时传--source-db。'
            [[ "$continuation_run" == latest || "$continuation_run" == /* ]] || \
                fail '续建来源只能是latest或受管运行目录的绝对路径。'
            mapping_launcher="$normal_mapping_launcher"
            mapping_launcher_pattern='start_rtabmap_calib_ab\.sh'
            expected_mapping_children=(lock_broker bridge tf apriltag graph_display submap_display apriltag_keyframe_binder safe_reloc_gid_frontend odom slam camera_health cloud_frame)
            ;;
        localize-old)
            ((continuation_run_explicit == 0)) || fail '旧地图只定位模式不能同时传--continue-run。'
            mapping_launcher="$old_map_localization_launcher"
            mapping_launcher_pattern='start_rtabmap_old_map_localization\.sh'
            expected_mapping_children=(bridge tf apriltag apriltag_seed localization_status odom slam cloud_frame)
            if [[ -z "$localization_source_db" && $check_only -eq 0 ]]; then
                printf '旧地图默认路径：%s\n' "$default_localization_source_db"
                read -r -p '直接回车使用默认路径，或输入另一个绝对rtabmap.db路径：' selected_source
                localization_source_db="${selected_source:-$default_localization_source_db}"
            fi
            localization_source_db="${localization_source_db:-$default_localization_source_db}"
            [[ "$localization_source_db" == /* ]] || fail '旧地图数据库必须是绝对路径。'
            frozen_names+=(localization_source_db)
            frozen_paths[localization_source_db]="$localization_source_db"
            ;;
        *) fail '运行方式必须是new、continue或localize-old。' ;;
    esac
    frozen_paths[mapping_launcher]="$mapping_launcher"
}

proc_ticks() {
    local record tail
    local -a fields=()
    [[ "$1" =~ ^[0-9]+$ && -r "/proc/$1/stat" ]] || return 1
    IFS= read -r record < "/proc/$1/stat" || return 1
    tail="${record##*) }"
    read -r -a fields <<< "$tail"
    [[ "${fields[19]:-}" =~ ^[0-9]+$ ]] || return 1
    printf '%s\n' "${fields[19]}"
}

monotonic_uptime_s() {
    local uptime remainder
    IFS=' ' read -r uptime remainder < /proc/uptime || return 1
    [[ "$uptime" =~ ^[0-9]+([.][0-9]+)?$ ]] || return 1
    printf '%s\n' "$uptime"
}

capture_final_core_runtime_identity() {
    local actual_sha canonical owner links mode device inode
    [[ "$mapping_mode" != localize-old ]] || return 0
    [[ "$FINAL_MAPPING_LAUNCHER_SHA256" != FINAL_MAPPING_LAUNCHER_SHA256 &&
       "$FINAL_MAPPING_LAUNCHER_SHA256" =~ ^[0-9a-f]{64}$ ]] ||
        fail '安全建图launcher的最终SHA256尚未由发布工具冻结。'
    [[ "$FINAL_RESET_RUNTIME_ROOT" != FINAL_RESET_RUNTIME_ROOT &&
       "$FINAL_RESET_RUNTIME_ROOT" == /* ]] ||
        fail '自定义Core最终运行目录尚未由发布工具冻结。'
    [[ "$FINAL_RESET_RUNTIME_NODE_SHA256" != FINAL_RESET_RUNTIME_NODE_SHA256 &&
       "$FINAL_RESET_RUNTIME_NODE_SHA256" =~ ^[0-9a-f]{64}$ ]] ||
        fail '自定义Core最终可执行文件SHA256尚未由发布工具冻结。'
    [[ -f "$FINAL_RESET_RUNTIME_NODE" && ! -L "$FINAL_RESET_RUNTIME_NODE" &&
       -r "$FINAL_RESET_RUNTIME_NODE" && -x "$FINAL_RESET_RUNTIME_NODE" ]] ||
        fail '自定义Core最终可执行文件缺失、不可执行或为符号链接。'
    canonical="$(readlink -f -- "$FINAL_RESET_RUNTIME_NODE")" ||
        fail '无法解析自定义Core最终可执行文件。'
    [[ "$canonical" == "$FINAL_RESET_RUNTIME_NODE" ]] ||
        fail '自定义Core最终路径含符号链接或不是规范绝对路径。'
    read -r owner links mode device inode < <(
        stat -Lc '%u %h %a %d %i' -- "$FINAL_RESET_RUNTIME_NODE"
    ) || fail '无法读取自定义Core最终inode身份。'
    [[ "$owner" == "$current_uid" && "$links" == 1 && "$mode" =~ ^[0-7]+$ ]] ||
        fail '自定义Core最终文件的属主、硬链接数或权限格式不安全。'
    (( (8#$mode & 8#22) == 0 )) ||
        fail '自定义Core最终文件不能允许组或其他用户写入。'
    actual_sha="$(sha256sum -- "$FINAL_RESET_RUNTIME_NODE")" ||
        fail '无法计算自定义Core最终可执行文件SHA256。'
    actual_sha="${actual_sha%% *}"
    [[ "$actual_sha" == "$FINAL_RESET_RUNTIME_NODE_SHA256" ]] ||
        fail '自定义Core最终可执行文件SHA256与发布冻结值不一致。'
    if [[ -n "$final_core_realpath" ]]; then
        [[ "$canonical" == "$final_core_realpath" &&
           "$device" == "$final_core_device" && "$inode" == "$final_core_inode" ]] ||
            fail '自定义Core最终可执行文件在预检后发生替换。'
    else
        final_core_realpath="$canonical"
        final_core_device="$device"
        final_core_inode="$inode"
    fi
}

reject_existing_final_core() {
    local matches
    [[ "$mapping_mode" != localize-old ]] || return 0
    matches="$(PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$wrapper_python3_runtime" - \
        "$current_uid" "$final_core_device" "$final_core_inode" <<'FINAL_CORE_SCAN_PY'
import os
import sys

expected_uid, expected_device, expected_inode = map(int, sys.argv[1:])
matches = []
for entry in os.scandir("/proc"):
    if not entry.name.isdecimal():
        continue
    try:
        process = os.stat(entry.path, follow_symlinks=False)
        executable = os.stat(f"{entry.path}/exe")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if (process.st_uid == expected_uid and executable.st_dev == expected_device and
            executable.st_ino == expected_inode):
        matches.append(int(entry.name))
print(" ".join(str(pid) for pid in sorted(matches)))
FINAL_CORE_SCAN_PY
    )" || fail '无法可靠扫描既有自定义Core进程。'
    [[ -z "$matches" ]] || fail "最终自定义Core已有实例在运行(PID=$matches)，请先正常退出。"
}

same_final_core_process() {
    local pid="$1" ticks="$2" verify_hash="${3:-0}"
    local canonical owner device inode actual_sha
    [[ -n "$final_core_realpath" && "$final_core_device" =~ ^[0-9]+$ &&
       "$final_core_inode" =~ ^[0-9]+$ ]] || return 1
    same_process "$pid" "$ticks" '.' || return 1
    canonical="$(readlink -f -- "/proc/$pid/exe" 2>/dev/null)" || return 1
    [[ "$canonical" == "$final_core_realpath" ]] || return 1
    read -r owner device inode < <(stat -Lc '%u %d %i' -- "/proc/$pid/exe" 2>/dev/null) || return 1
    [[ "$owner" == "$current_uid" && "$device" == "$final_core_device" &&
       "$inode" == "$final_core_inode" ]] || return 1
    if ((verify_hash)); then
        actual_sha="$(sha256sum -- "/proc/$pid/exe" 2>/dev/null)" || return 1
        [[ "${actual_sha%% *}" == "$FINAL_RESET_RUNTIME_NODE_SHA256" ]] || return 1
    fi
}

mapping_child_identity_matches() {
    local child="$1" pid="$2" ticks="$3" verify_hash="${4:-0}"
    if [[ "$child" == slam && "$mapping_mode" != localize-old ]]; then
        same_final_core_process "$pid" "$ticks" "$verify_hash"
    else
        same_process "$pid" "$ticks" "${child_patterns[$child]}"
    fi
}

snapshot_mapping_pid_files() {
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$wrapper_python3_runtime" - \
        "$mapping_state_dir" "$@" <<'SAFE_PID_SNAPSHOT_PY'
import os
import re
import stat
import sys

path = os.path.abspath(sys.argv[1])
names = sys.argv[2:]
if not os.path.lexists(path):
    raise SystemExit(0)
if os.path.realpath(path) != path:
    raise SystemExit("mapping PID directory has a symlinked path")
flags = os.O_RDONLY | os.O_CLOEXEC | os.O_DIRECTORY | os.O_NOFOLLOW
directory = os.open(path, flags)
try:
    if os.path.realpath(f"/proc/self/fd/{directory}") != path:
        raise SystemExit("mapping PID directory descriptor does not match its canonical path")
    directory_info = os.fstat(directory)
    if (not stat.S_ISDIR(directory_info.st_mode) or
            directory_info.st_uid != os.getuid() or
            stat.S_IMODE(directory_info.st_mode) != 0o700):
        raise SystemExit("mapping PID directory is not an owned 0700 directory")
    for name in names:
        if re.fullmatch(r"[a-z][a-z0-9_]*", name) is None:
            raise SystemExit("unsafe mapping child name")
        filename = f"{name}.pid"
        file_flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
        try:
            descriptor = os.open(filename, file_flags, dir_fd=directory)
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise SystemExit(f"cannot safely open {filename}: {exc}") from exc
        try:
            before = os.fstat(descriptor)
            if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.getuid() or
                    before.st_nlink != 1 or stat.S_IMODE(before.st_mode) != 0o600 or
                    not 1 <= before.st_size <= 32):
                raise SystemExit(f"unsafe mapping PID file: {filename}")
            payload = os.read(descriptor, before.st_size + 1)
            after = os.fstat(descriptor)
            identity_before = (before.st_dev, before.st_ino, before.st_uid,
                               before.st_mode, before.st_nlink, before.st_size,
                               before.st_mtime_ns, before.st_ctime_ns)
            identity_after = (after.st_dev, after.st_ino, after.st_uid,
                              after.st_mode, after.st_nlink, after.st_size,
                              after.st_mtime_ns, after.st_ctime_ns)
            if identity_before != identity_after or len(payload) != before.st_size:
                raise SystemExit(f"mapping PID file changed while read: {filename}")
            if re.fullmatch(rb"[1-9][0-9]*\n?", payload) is None:
                raise SystemExit(f"invalid mapping PID payload: {filename}")
            pid = int(payload.rstrip(b"\n"))
            if pid > 2_147_483_647:
                raise SystemExit(f"mapping PID is out of range: {filename}")
            print(f"{name}\t{pid}")
        finally:
            os.close(descriptor)
finally:
    os.close(directory)
SAFE_PID_SNAPSHOT_PY
}

same_process() {
    local pid="$1" ticks="$2" pattern="${3:-.}" actual command_line
    [[ "$pid" =~ ^[0-9]+$ && "$ticks" =~ ^[0-9]+$ ]] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    actual="$(proc_ticks "$pid")" || return 1
    [[ "$actual" == "$ticks" ]] || return 1
    [[ "$(stat -c %u "/proc/$pid" 2>/dev/null)" == "$current_uid" ]] || return 1
    command_line="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)" || return 1
    [[ "$command_line" =~ $pattern ]]
}

process_cmdline_has_exact_argument() {
    local pid="$1" expected="$2" argument
    [[ -r "/proc/$pid/cmdline" ]] || return 1
    while IFS= read -r -d '' argument; do
        [[ "$argument" != "$expected" ]] || return 0
    done < "/proc/$pid/cmdline"
    return 1
}

capture_mapping_launcher_identity() {
    local canonical owner mode bash_sha
    [[ -f "$mapping_launcher" && ! -L "$mapping_launcher" ]] ||
        fail '建图launcher必须是非符号链接普通文件。'
    canonical="$(readlink -f -- "$mapping_launcher")" || fail '无法解析建图launcher。'
    [[ "$canonical" == "$mapping_launcher" ]] || fail '建图launcher路径不是规范绝对路径。'
    read -r owner mode mapping_launcher_device mapping_launcher_inode < <(
        stat -Lc '%u %a %d %i' -- "$mapping_launcher"
    ) || fail '无法记录建图launcher身份。'
    [[ "$owner" == "$current_uid" && "$mode" =~ ^[0-7]{3,4}$ ]] ||
        fail '建图launcher属主或权限格式异常。'
    (( (8#$mode & 8#22) == 0 )) || fail '建图launcher不能被组或其他用户写入。'
    mapping_launcher_realpath="$canonical"

    [[ -x "$wrapper_bash_runtime" && -f "$wrapper_bash_runtime" && ! -L "$wrapper_bash_runtime" ]] ||
        fail '冻结Bash运行时缺失、非普通文件或是符号链接。'
    bash_sha="$(sha256sum -- "$wrapper_bash_runtime")" || fail '无法核验冻结Bash运行时。'
    [[ "${bash_sha%% *}" == "$wrapper_bash_runtime_sha256" ]] ||
        fail '冻结Bash运行时SHA256不匹配。'
    read -r owner wrapper_bash_device wrapper_bash_inode < <(
        stat -Lc '%u %d %i' -- "$wrapper_bash_runtime"
    ) || fail '无法记录冻结Bash运行时身份。'
    [[ "$owner" == 0 ]] || fail '冻结Bash运行时不属于root。'
}

verify_mapping_launcher_identity() {
    local canonical device inode
    canonical="$(readlink -f -- "$mapping_launcher")" || return 1
    [[ "$canonical" == "$mapping_launcher_realpath" ]] || return 1
    read -r device inode < <(stat -Lc '%d %i' -- "$mapping_launcher") || return 1
    [[ "$device" == "$mapping_launcher_device" && "$inode" == "$mapping_launcher_inode" ]]
}

same_mapping_launcher_process() {
    local pid="$1" ticks="$2" canonical owner device inode
    same_process "$pid" "$ticks" "$mapping_launcher_pattern" || return 1
    canonical="$(readlink -f -- "/proc/$pid/exe" 2>/dev/null)" || return 1
    [[ "$canonical" == "$wrapper_bash_runtime" ]] || return 1
    read -r owner device inode < <(stat -Lc '%u %d %i' -- "/proc/$pid/exe" 2>/dev/null) || return 1
    [[ "$owner" == 0 && "$device" == "$wrapper_bash_device" &&
       "$inode" == "$wrapper_bash_inode" ]] || return 1
    [[ -n "$run_mapping_launcher" ]] || return 1
    read -r device inode < <(stat -Lc '%d %i' -- "$run_mapping_launcher") || return 1
    [[ "$device" == "$run_mapping_launcher_device" &&
       "$inode" == "$run_mapping_launcher_inode" ]] || return 1
    process_cmdline_has_exact_argument "$pid" "$run_mapping_launcher"
}

materialize_mapping_launcher() {
    local runtime_dir target actual owner mode links
    runtime_dir="$run_dir/runtime_scripts"
    mkdir -m 700 -- "$runtime_dir" || fail '无法创建本次私有launcher目录。'
    target="$runtime_dir/$(basename -- "$mapping_launcher")"
    cp --reflink=never -- "$mapping_launcher" "$target" || fail '无法复制冻结建图launcher。'
    chmod 500 -- "$target" || fail '无法锁定本次launcher权限。'
    actual="$(sha256sum -- "$target")" || fail '无法核验本次launcher副本。'
    [[ "${actual%% *}" == "${frozen_hashes[mapping_launcher]}" ]] ||
        fail '本次launcher副本与预检哈希不一致。'
    verify_input_hashes
    read -r owner mode links run_mapping_launcher_device run_mapping_launcher_inode < <(
        stat -Lc '%u %a %h %d %i' -- "$target"
    ) || fail '无法记录本次launcher副本身份。'
    [[ "$owner" == "$current_uid" && "$mode" == 500 && "$links" == 1 ]] ||
        fail '本次launcher副本身份或权限异常。'
    run_mapping_launcher="$target"
}

pid_may_be_running() {
    [[ "$1" =~ ^[0-9]+$ ]] || return 1
    kill -0 "$1" 2>/dev/null || [[ -e "/proc/$1" ]]
}

environment_has() {
    tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null | grep -Fx -- "$2" >/dev/null
}

verify_teleop_owner() {
    local line socket_snapshot
    local -a sockets=() owners=()
    same_process "$teleop_pid" "$teleop_ticks" '(^|[ /])teleop_keyboard\.py( |$)' || return 1
    [[ "$(readlink -f "/proc/$teleop_pid/cwd")" == "$(dirname -- "$teleop_script")" ]] || return 1
    environment_has "$teleop_pid" 'DRY_RUN=0' || return 1
    environment_has "$teleop_pid" 'WBROV_LIGHT_ON_START=1' || return 1
    environment_has "$teleop_pid" "WBROV_LIGHT_BRIGHTNESS=$brightness" || return 1
    environment_has "$teleop_pid" "WBROV_LIGHT_READY_FILE=$ready_file" || return 1
    socket_snapshot="$(ss -H -u -a -n -p 'sport = :3333' 2>/dev/null)" || return 1
    [[ -n "$socket_snapshot" ]] || return 1
    mapfile -t sockets <<< "$socket_snapshot"
    ((${#sockets[@]} > 0)) || return 1
    mapfile -t owners < <(printf '%s\n' "${sockets[@]}" | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u)
    ((${#owners[@]} == 1)) && [[ "${owners[0]}" == "$teleop_pid" ]] || return 1
    for line in "${sockets[@]}"; do
        [[ "$line" == *"pid=$teleop_pid,"* ]] || return 1
    done
}

check_light_state() {
    local phase="$1"
    shift
    "$wrapper_python3_runtime" "$validator" "$ready_file" --pid "$teleop_pid" \
        --brightness "$brightness" --phase "$phase" \
        --boot-id "$boot_id" --start-ticks "$teleop_ticks" \
        --not-before "$spawn_monotonic" "$@"
}

check_mapping_lifecycle() {
    local phase="$1"
    shift
    "$wrapper_python3_runtime" "$lifecycle_validator" "$mapping_lifecycle_file" \
        --pid "$mapping_pid" --start-ticks "$mapping_ticks" \
        --boot-id "$boot_id" --phase "$phase" "$@"
}

mapping_is_exiting() {
    check_mapping_lifecycle stopping --quiet || \
        check_mapping_lifecycle stopped --allow-incomplete --quiet
}

reap_mapping_status() {
    # A failed identity check does not prove exit. Never block in wait while
    # an unverified/reused PID is still present, even if its cmdline changed.
    pid_may_be_running "$mapping_pid" && return 1
    if ((mapping_reaped == 0)); then
        if wait "$mapping_pid"; then
            mapping_exit_status=0
        else
            mapping_exit_status=$?
        fi
        mapping_reaped=1
    fi
    return 0
}

signal_if_same() {
    local pid="$1" ticks="$2" pattern="$3" signal="$4"
    if same_process "$pid" "$ticks" "$pattern"; then
        kill -"$signal" "$pid" 2>/dev/null || return 1
    fi
}

wait_same() {
    local pid="$1" ticks="$2" pattern="$3" attempts="$4" count
    for ((count=0; count<attempts; count++)); do
        if ! same_process "$pid" "$ticks" "$pattern"; then
            pid_may_be_running "$pid" && return 2
            return 0
        fi
        sleep 0.1
    done
    return 1
}

wait_mapping_launcher() {
    local attempts="$1" count
    for ((count=0; count<attempts; count++)); do
        if ! same_mapping_launcher_process "$mapping_pid" "$mapping_ticks"; then
            pid_may_be_running "$mapping_pid" && return 2
            return 0
        fi
        sleep 0.1
    done
    return 1
}

signal_mapping_remainders() {
    # Only this private setsid group, same uid, start-time not older than launch,
    # and a known mapping command may receive a signal. Never use broad pkill.
    local signal="$1" pid ticks command_line failed=0
    # Never terminate the exporter or launcher while a database/export save
    # could still be in progress. Only leftover acquisition/visualization nodes.
    local allowed='zed_rgbd_publisher_(calib_test|dual_stream_candidate|rate_ab)|static_transform_publisher|apriltag_landmark_node\.py|apriltag_keyframe_binder\.py|wbrov_corner_recovery/consumer\.py|safe_reloc_gid_frontend|safe_reloc_ipc_engine\.py|header_health_observer|rtabmap_graph_display\.py|rtabmap_submap_display\.py|apriltag_initialpose_seed\.py|localization_status_observer\.py|rgbd_odometry|rtabmap_slam/rtabmap|point_cloud_xyzrgb|rtabmap_dense_calib_ab/launcher\.lock|rviz2'
    [[ "$mapping_pid" =~ ^[0-9]+$ && "$mapping_ticks" =~ ^[0-9]+$ ]] || return 0
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        ticks="$(proc_ticks "$pid")" || continue
        ((ticks >= mapping_ticks)) || continue
        command_line="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)" || continue
        if [[ "$mapping_mode" != localize-old ]] && same_final_core_process "$pid" "$ticks" 0; then
            if ! kill -"$signal" "$pid" 2>/dev/null; then
                printf '[安全] 无法向已核验inode的自定义Core发送%s: PID=%s\n' "$signal" "$pid" >&2
                failed=1
            fi
        elif [[ "$command_line" =~ $allowed ]]; then
            if ! signal_if_same "$pid" "$ticks" "$allowed" "$signal"; then
                printf '[安全] 无法向已核验的建图子进程发送%s: PID=%s\n' "$signal" "$pid" >&2
                failed=1
            fi
        else
            printf '[安全] 保留保存过程或无法确认的进程，不发送信号: PID=%s\n' "$pid" >&2
        fi
    done < <(pgrep -g "$mapping_pid" || true)
    return "$failed"
}

cleanup() {
    local failed=0 mapping_signal_failed=0 teleop_signal_failed=0 wait_status
    local mapping_identity_uncertain=0
    ((cleanup_started == 0)) || return 0
    cleanup_started=1
    [[ -n "$teleop_pid" || -n "$mapping_pid" ]] || return 0
    if [[ -n "$mapping_pid" ]]; then
        if ! mapping_is_exiting && \
           same_mapping_launcher_process "$mapping_pid" "$mapping_ticks"; then
            mapping_stop_requested=1
            if ! kill -TERM "$mapping_pid" 2>/dev/null; then
                mapping_signal_failed=1
                failed=1
            fi
        fi
    fi
    if [[ -n "$teleop_pid" ]]; then
        if ! signal_if_same "$teleop_pid" "$teleop_ticks" 'run_water\.sh|(^|[ /])teleop_keyboard\.py( |$)' TERM; then
            teleop_signal_failed=1
            failed=1
        fi
    fi
    # All console output comes after map-stop and teleop-safe-exit signals.
    [[ -z "$failure_reason" ]] || printf '[拒绝继续] %s\n' "$failure_reason" >&2
    echo '已完成停止信号步骤；现在分别等待遥控锁定/关灯退出与建图保存。'
    if ((mapping_signal_failed)); then
        echo '[安全] 无法通知已核验的建图入口停止摄取。' >&2
    fi
    if ((teleop_signal_failed)); then
        echo '[危险] 无法向已核验的遥控发送安全退出信号，请立即现场物理急停。' >&2
    fi
    # Do not wait for database export before the teleop's safety signal above.
    if [[ -n "$teleop_pid" ]]; then
        if wait_same "$teleop_pid" "$teleop_ticks" 'run_water\.sh|(^|[ /])teleop_keyboard\.py( |$)' 100; then
            wait "$teleop_pid" 2>/dev/null || true
            if ! check_light_state shutdown --max-age 12; then
                echo '[安全] 未取得本次明确关灯退出状态，不能宣称锁定/关灯成功；请现场确认，必要时物理断电。' >&2
                failed=1
            fi
        else
            wait_status=$?
            if ((wait_status == 2)); then
                echo '[危险] 遥控PID仍存在但身份无法确认；不向不明进程发信号、不阻塞wait。请立即现场物理断电。' >&2
            else
                echo '[危险] 遥控未正常退出。没有启动第二个SDK，也没有强杀仍可能发送命令的遥控；请立即按现场流程物理断电。' >&2
            fi
            failed=1
        fi
    fi
    if [[ -n "$mapping_pid" ]]; then
        if wait_mapping_launcher 1200; then
            :
        else
            wait_status=$?
            if ((wait_status == 2)); then
                mapping_identity_uncertain=1
                echo '[保存未确认] 建图PID仍存在但身份无法确认；不向不明进程发信号、不阻塞wait，保留证据。' >&2
            else
                echo '[保存未完成] 建图保存超过120秒；不强杀启动器或export，不宣称已保存。请查看日志。' >&2
            fi
            failed=1
        fi
        if ((mapping_identity_uncertain == 0)) && ! signal_mapping_remainders TERM; then
            failed=1
        fi
        if ! pid_may_be_running "$mapping_pid"; then
            if ! reap_mapping_status; then
                echo '[保存未确认] 建图退出码暂时无法安全取得，不宣称正常保存。' >&2
                failed=1
            fi
            if [[ "$mapping_mode" == localize-old ]]; then
                check_mapping_lifecycle stopped --allow-incomplete || {
                    echo '[定位退出未确认] 缺少本次旧地图定位的stopped结果。' >&2
                    failed=1
                }
            elif ! check_mapping_lifecycle stopped; then
                echo '[保存未确认] 缺少本次stopped完整成功结果，或节点/导出/清理失败。' >&2
                failed=1
            fi
            if ((mapping_exit_status != 0)); then
                if ((mapping_stop_requested && (mapping_exit_status == 130 || mapping_exit_status == 143))); then
                    echo "建图响应本入口停止信号退出（状态码$mapping_exit_status）；保存结果仍以上述生命周期校验为准。"
                else
                    echo "[建图失败] 建图自然退出状态码=$mapping_exit_status，不能当作成功。" >&2
                    failed=1
                fi
            fi
        fi
    fi
    echo '注意：EXLink没有这些命令的协议ACK；状态文件只证明本机提交路径，物理灯光和推进器状态仍需现场确认。'
    [[ -z "$run_dir" ]] || printf '联合启动日志: %s\n' "$run_dir"
    return "$failed"
}

queue_completed_navigation_map() {
    # Called only after both teleop safety cleanup and mapping save succeeded.
    # No DB scan or converter runs in this wrapper. The detached job waits for
    # all mapping/control clients to leave and yields if new work starts.
    local completed_run='' actual=''
    local navigation_root='/home/exocean/.cache/kcap_navigation_candidates/20260920/KCAP_Navigation_20260920_candidate_v12'
    local completion_helper="$navigation_root/completed_map_job.py"
    local completion_manifest="$navigation_root/navigation/tag_entry_manifest.json"
    local expected_helper='11bb2dadd7eff11889297533e32fb006a4c915f4c505c4507ffcf81fc79d69b4'
    local expected_manifest='ab30a95e3c800fa40b4f9f1bffc80a22ed8408d09bb94679e2f5d36da769e312'
    [[ "$mapping_mode" != localize-old && -n "$mapping_pid" && -n "$mapping_ticks" ]] || return 0
    [[ "$expected_helper" =~ ^[0-9a-f]{64}$ && "$expected_manifest" =~ ^[0-9a-f]{64}$ ]] || return 1
    [[ -f "$completion_helper" && ! -L "$completion_helper" && -f "$completion_manifest" && ! -L "$completion_manifest" ]] || return 1
    actual="$(sha256sum -- "$completion_helper")" || return 1
    [[ "${actual%% *}" == "$expected_helper" ]] || return 1
    actual="$(sha256sum -- "$completion_manifest")" || return 1
    [[ "${actual%% *}" == "$expected_manifest" ]] || return 1
    [[ -f "$mapping_state_dir/run_dir" && ! -L "$mapping_state_dir/run_dir" ]] || return 1
    IFS= read -r completed_run < "$mapping_state_dir/run_dir" || return 1
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "$wrapper_python3_runtime" \
        "$completion_helper" --config "$navigation_root/localization/completed_map_config.json" \
        enqueue --run "$completed_run" --expected-launcher-pid "$mapping_pid" \
        --expected-launcher-start-ticks "$mapping_ticks" --expected-boot "$boot_id"
}

show_mapping_failure_log() {
    [[ -n "$run_dir" && -f "$run_dir/mapping.log" && ! -L "$run_dir/mapping.log" ]] || return 0
    printf '\n[建图故障详情] 本次 mapping.log 最后80行：\n' >&2
    tail -n 80 -- "$run_dir/mapping.log" >&2 || true
}

on_exit() {
    local status=$? cleanup_status=0
    trap - EXIT
    trap '' INT TERM HUP
    cleanup || cleanup_status=$?
    if ((cleanup_status != 0 && status == 0)); then
        status=1
    fi
    if ((cleanup_status == 0 && mapping_exit_status == 0)) && [[ -z "$failure_reason" && -n "$run_dir" ]]; then
        case "$status" in
            0|129|130|143)
                if ! queue_completed_navigation_map >"$run_dir/navigation_completion.log" 2>&1; then
                    printf '[Navigation preparation not queued] Saved mapping artifacts remain unchanged; see %s/navigation_completion.log\n' "$run_dir" >&2
                fi
                ;;
        esac
    fi
    if ((status != 0)); then
        if [[ -z "$failure_reason" ]] && ((mapping_exit_status != 0)); then
            failure_reason="建图进程退出（状态码$mapping_exit_status）；具体原因见本次建图日志。"
        fi
        show_mapping_failure_log
    fi
    # Preserve the diagnosis after the safety cleanup, even if the terminal closes.
    if [[ -n "$run_dir" && -d "$run_dir" ]]; then
        printf 'exit_status=%s\nfailure_reason=%s\nmapping_exit_status=%s\n' \
            "$status" "${failure_reason:-未记录显式拒绝原因；请查看mapping.log和teleop.log}" \
            "$mapping_exit_status" >"$run_dir/exit_summary.txt" || true
    fi
    if ((status != 0)) && [[ -t 0 && -t 1 ]]; then
        printf '\n启动/运行失败（状态码%s）。%s\n' "$status" "${failure_reason:-请查看上方错误信息。}" >&2
        [[ -z "$run_dir" ]] || printf '本次日志目录：%s\n' "$run_dir" >&2
        read -r -p '按回车关闭此窗口：' _ || true
    fi
    exit "$status"
}
require_interactive() {
    [[ -t 0 ]] || fail '必须在现场交互终端输入OK，不能无确认后台启动。'
}

check_directory_readonly() {
    local directory="$1" ancestor="$1" mode
    [[ ! -L "$directory" ]] || fail "状态目录不能是符号链接: $directory"
    if [[ -e "$directory" ]]; then
        [[ -d "$directory" ]] || fail "状态路径不是目录: $directory"
        [[ "$(readlink -f "$directory")" == "$directory" ]] || fail "状态目录祖先包含符号链接: $directory"
        [[ "$(stat -c %u "$directory")" == "$current_uid" ]] || fail '状态目录属主不匹配。'
        [[ "$(stat -c %a "$directory")" == 700 ]] || fail "专属状态目录必须为0700: $directory"
        return 0
    fi
    while [[ ! -e "$ancestor" ]]; do
        ancestor="$(dirname -- "$ancestor")"
    done
    [[ -d "$ancestor" && -w "$ancestor" && -x "$ancestor" ]] || fail "无法在最近现有父目录创建状态目录: $ancestor"
    [[ "$(readlink -f "$ancestor")" == "$ancestor" ]] || fail "状态目录祖先包含符号链接: $ancestor"
    [[ "$(stat -c %u "$ancestor")" == "$current_uid" ]] || fail "状态目录最近现有父目录不属于当前用户: $ancestor"
    mode="$(stat -c %a "$ancestor")"
    (( (8#$mode & 2) == 0 )) || fail "状态目录父目录可被所有人写入: $ancestor"
    printf '[只读预检] 专属目录尚不存在；正常运行确认OK后才创建0700目录: %s\n' "$directory"
}

check_existing_lock_readonly() {
    local lock_file="$state_dir/orchestrator.lock" check_lock_fd
    [[ ! -L "$lock_file" ]] || fail '编排锁不能是符号链接。'
    [[ -e "$lock_file" ]] || return 0
    [[ -f "$lock_file" && "$(stat -c %u "$lock_file")" == "$current_uid" && "$(stat -c %a "$lock_file")" == 600 ]] || fail '既有编排锁的类型/属主/权限不安全。'
    # Read-only open: no lock file or directory is ever created by preflight.
    exec {check_lock_fd}<"$lock_file"
    flock -n "$check_lock_fd" || fail '另一个联合入口正在运行。'
    flock -u "$check_lock_fd"
    exec {check_lock_fd}<&-
}

preflight_read_only() {
    wbrov_check_dds_profile || fail 'DDS 图像通信配置校验失败。'
    local tool process_status socket_snapshot launcher_sha
    [[ "$brightness" =~ ^[0-9]+$ ]] && ((brightness >= 1 && brightness <= 100)) || fail '亮度必须是1–100整数。'
    for tool in awk basename chmod cp cut dirname flock grep mkdir mktemp ping pgrep ps readlink rm setsid sha256sum sleep sort ss stat tr; do
        command -v "$tool" >/dev/null || fail "缺少工具: $tool"
    done
    [[ -x "$wrapper_python3_runtime" && -f "$wrapper_python3_runtime" && ! -L "$wrapper_python3_runtime" ]] ||
        fail '冻结Python运行时缺失、非普通文件或是符号链接。'
    local python_sha
    python_sha="$(sha256sum -- "$wrapper_python3_runtime")" || fail '无法核验冻结Python运行时。'
    [[ "${python_sha%% *}" == "$wrapper_python3_runtime_sha256" ]] ||
        fail '冻结Python运行时SHA256不匹配。'
    awk -v value="$settle_s" 'BEGIN {exit !(value >= 3 && value <= 5)}' || fail '开灯稳定时间必须是3–5秒。'
    [[ -n "${DISPLAY:-}" ]] || fail '需要本机图形桌面DISPLAY以显示遥控和RViz。'
    [[ -r "$validator" && -r "$lifecycle_validator" && -x "$teleop_launcher" && -r "$teleop_script" && -x "$mapping_launcher" && -r "$calibration_file" ]] || fail '遥控、状态校验器、隔离建图或水下YAML缺失。'
    if [[ "$mapping_mode" != localize-old ]]; then
        capture_final_core_runtime_identity
        launcher_sha="$(sha256sum -- "$mapping_launcher")" || fail '无法在执行前计算安全建图launcher SHA256。'
        [[ "${launcher_sha%% *}" == "$FINAL_MAPPING_LAUNCHER_SHA256" ]] ||
            fail '安全建图launcher不是发布工具冻结的最终版本，禁止执行。'
        reject_existing_final_core
    fi
    grep -Fq 'WBROV_LIGHT_READY_FILE' "$teleop_script" || fail '遥控尚未安装就绪状态接口，本候选不能启动。'
    grep -Eq '^export WBROV_LIGHT_ON_START=0$|^export WBROV_LIGHT_BRIGHTNESS=60$' "$teleop_launcher" && fail 'run_water.sh会覆盖开灯参数，必须先合入保留调用方环境的候选。'
    grep -Eq '^export RTAB_DEPTH_MODE=NEURAL$' "$mapping_launcher" || fail '隔离建图未确认使用NEURAL。'
    grep -Fq "trap '' INT TERM HUP" "$mapping_launcher" && \
        grep -Fq "trap 'trap \"\" INT TERM HUP; exit 143' TERM" "$mapping_launcher" || fail '隔离建图尚未合入重复信号保护，禁止联合启动。'
    grep -Fq 'lifecycle.json' "$mapping_launcher" || fail '隔离建图尚未合入原子生命周期结果，禁止联合启动。'
    if [[ "$mapping_mode" != localize-old ]]; then
        grep -Fq 'safe_relocalization_enabled:=true' "$mapping_launcher" || \
            fail '建图入口尚未启用核心安全重定位事务。'
        grep -Fq 'CALIB_AB_CONTINUE_RUN_DIR' "$mapping_launcher" || \
            fail '建图入口尚未实现受管旧图续建。'
    fi
    # This dedicated mode exits before ROS/global setup or any runtime writes.
    # Reject invalid ranges before starting the real controller or lights.
    grep -Fq -- '--print-range-config' "$mapping_launcher" || fail '隔离建图缺少只读距离配置检查。'
    range_snapshot="$("$mapping_launcher" --print-range-config)" || fail '定位/标签/点云距离配置无效。'
    [[ -z "${CALIB_AB_LIGHT_HOOK:-}" ]] || fail '独立灯光hook已禁止。'
    if pgrep -f '[t]eleop_keyboard\.py|[p]id_feedback_probe\.py|[c]apture_zedx_charuco|[z]ed_orb_mapping|[z]ed_rgbd_(publisher|vio_shadow)|[a]priltag_keyframe_binder\.py|[a]priltag_safe_relocalizer\.py|[w]brov_corner_recovery/consumer\.py|[s]afe_reloc_gid_frontend|[s]afe_reloc_ipc_engine\.py|[h]eader_health_observer|[a]priltag_initialpose_seed\.py|[l]ocalization_status_observer\.py|[r]gbd_odometry|[r]tabmap_slam/rtabmap|[s]tart_rtabmap_(calib_ab|old_map_localization)\.sh|[r]tabmap-export' >/dev/null; then
        fail '已有遥控、标定采集或建图占用控制/相机；请先正常退出。'
    else
        process_status=$?
        ((process_status == 1)) || fail '无法可靠检查控制/相机进程占用。'
    fi
    socket_snapshot="$(ss -H -u -a -n 'sport = :3333')" || fail '无法可靠检查UDP3333占用。'
    [[ -z "$socket_snapshot" ]] || fail 'UDP3333已被占用，拒绝第二个控制器。'
    ping -c 1 -W 1 192.168.8.3 >/dev/null 2>&1 || fail '飞控192.168.8.3不可达。'
    check_directory_readonly "$state_dir"
    check_directory_readonly "$control_state_dir"
    # Match the launcher's existing-directory contract before starting teleop.
    # Legacy 0775 roots require an explicit file-only migration, never chmod here.
    check_directory_readonly "$mapping_state_dir"
    check_directory_readonly "$HOME/maps/rtabmap_calibration_ab"
    check_existing_lock_readonly
    if [[ -e "$ready_file" || -L "$ready_file" ]]; then
        [[ -f "$ready_file" && ! -L "$ready_file" ]] || fail '旧灯光状态不是普通文件。'
        [[ "$(stat -c %u "$ready_file")" == "$current_uid" && "$(stat -c %a "$ready_file")" == 600 ]] || fail '旧灯光状态的属主/权限不安全。'
    fi
}

capture_input_hashes() {
    local name result
    for name in "${frozen_names[@]}"; do
        result="$(sha256sum -- "${frozen_paths[$name]}")" || fail "无法计算$name SHA256。"
        frozen_hashes[$name]="${result%% *}"
        [[ "${frozen_hashes[$name]}" =~ ^[0-9a-f]{64}$ ]] || fail "$name SHA256格式异常。"
    done
    [[ "${frozen_hashes[calibration_file]}" == "$calibration_expected_sha256" ]] || fail '水下YAML不是本次交接的冻结版本，禁止试建图。'
    if [[ "$mapping_mode" != localize-old ]]; then
        [[ "${frozen_hashes[mapping_launcher]}" == "$FINAL_MAPPING_LAUNCHER_SHA256" ]] ||
            fail '安全建图launcher SHA256未绑定到最终发布值。'
    fi
    capture_mapping_launcher_identity
}

verify_input_hashes() {
    local name result
    for name in "${frozen_names[@]}"; do
        result="$(sha256sum -- "${frozen_paths[$name]}")" || fail "无法复核$name SHA256。"
        [[ "${result%% *}" == "${frozen_hashes[$name]}" ]] || fail "$name 在预检后发生变化，请重新运行入口。"
    done
    verify_mapping_launcher_identity || fail '建图launcher在预检后身份发生变化。'
    capture_final_core_runtime_identity
}

emit_frozen_manifest() {
    printf 'dds_profile=%s\ndds_profile_sha256=%s\ndds_shm_bytes=67108864\n' "$wbrov_dds_profile" "$wbrov_dds_profile_sha256"
    local name
    for name in "${frozen_names[@]}"; do
        printf '%s=%s\n%s_sha256=%s\n' "$name" "${frozen_paths[$name]}" "$name" "${frozen_hashes[$name]}"
    done
    printf 'calibration_expected_sha256=%s\n' "$calibration_expected_sha256"
    printf 'mapping_mode=%s\ncontinuation_run=%s\n' "$mapping_mode" "$continuation_run"
    if [[ "$mapping_mode" != localize-old ]]; then
        printf 'final_core_node=%s\nfinal_core_sha256=%s\nfinal_core_device=%s\nfinal_core_inode=%s\n' \
            "$final_core_realpath" "$FINAL_RESET_RUNTIME_NODE_SHA256" \
            "$final_core_device" "$final_core_inode"
    fi
    printf 'mapping_launcher_realpath=%s\nmapping_launcher_device=%s\nmapping_launcher_inode=%s\n' \
        "$mapping_launcher_realpath" "$mapping_launcher_device" "$mapping_launcher_inode"
    printf 'mapping_shell=%s\nmapping_shell_sha256=%s\nmapping_shell_device=%s\nmapping_shell_inode=%s\n' \
        "$wrapper_bash_runtime" "$wrapper_bash_runtime_sha256" \
        "$wrapper_bash_device" "$wrapper_bash_inode"
    printf '%s\n' "$range_snapshot"
}

confirm_operator() {
    local answer
    if [[ "$mapping_mode" == localize-old ]]; then
        printf '\n水下遥控＋自动开灯＋RTAB-Map旧地图只定位\n灯光: %s%%；稳定等待: %s秒；旧地图: %s\n' "$brightness" "$settle_s" "$localization_source_db"
        echo '本模式只在旧地图中识别位姿：不新增地图节点、不导出、不覆盖旧地图；未定位时不能当作位姿有效。'
    elif [[ "$mapping_mode" == continue ]]; then
        printf '\n水下遥控＋自动开灯＋RTAB-Map安全续建\n灯光: %s%%；稳定等待: %s秒；续建来源: %s\n' "$brightness" "$settle_s" "$continuation_run"
        echo '旧地图只从正常停止且锚点SHA一致的副本载入；新段不会凭单帧或裸/add_link直接焊接。'
    else
        printf '\n水下遥控＋自动开灯＋RTAB-Map新建地图\n灯光: %s%%；稳定等待: %s秒；水下YAML: %s\n' "$brightness" "$settle_s" "$calibration_file"
    fi
    echo '请确认：机器人已可靠系留、推进器附近无人/线缆/杂物；现场人员能立即物理断电；厂家地面站已关闭。'
    echo '相机窗口清洁、灯向两侧斜照；不会启动ChArUco采集。'
    echo '初始化及可信通讯就绪后，以零运动请求进入纯手动BODY起步；请先确认机身各轴方向与周围安全条件。'
    echo 'H仅开关主机R/P（横滚/俯仰）回正，无自动入水；本入口不自动按H。'
    echo 'W/S、A/D、R/F是机身轴向油门；人工转动输入逐轴优先于主机R/P回正。'
    echo '实际断联、急停或退出仍执行停机流程；需要时立即现场物理断电。'
    echo 'mode0不承诺固件姿态内环OFF；零运动请求不等于物理零运动保证。'
    echo 'EXLink无灯光/锁定ACK；请现场看见灯亮后再移动。改变灯开关/亮度会停止本次建图。'
    read -r -p '确认后输入 OK：' answer
    [[ "$answer" == OK ]]
}

prepare_runtime() {
    local directory stamp
    # Only main's confirmed normal-run path may call this mutating step.
    for directory in "$state_dir" "$control_state_dir"; do
        mkdir -p -- "$directory"
        check_directory_readonly "$directory"
    done
    [[ ! -L "$state_dir/orchestrator.lock" ]] || fail '编排锁不能是符号链接。'
    exec 9>>"$state_dir/orchestrator.lock"
    flock -n 9 || fail '另一个联合入口正在运行。'
    # Exact, previously validated transient status path; retain prior run logs.
    rm -f -- "$ready_file"
    printf -v stamp '%(%Y%m%d_%H%M%S)T' -1
    run_dir="$(mktemp -d "$state_dir/run_${stamp}.XXXXXX")"
    materialize_mapping_launcher
    {
        printf 'manifest_version=1\nboot_id=%s\nlight_brightness=%s\nlight_settle_s=%s\nprofile=custom\ndepth_mode=NEURAL\n' "$boot_id" "$brightness" "$settle_s"
        emit_frozen_manifest
    } >"$run_dir/launch_manifest.txt"
}

start_teleop() {
    spawn_monotonic="$("$wrapper_python3_runtime" -I -c 'import time; print(time.monotonic())')"
    export WBROV_LIGHT_ON_START=1 WBROV_LIGHT_BRIGHTNESS="$brightness" WBROV_LIGHT_READY_FILE="$ready_file"
    setsid "$teleop_launcher" >"$run_dir/teleop.log" 2>&1 <<< 'OK' &
    teleop_pid=$!
    teleop_ticks="$(proc_ticks "$teleop_pid")" || fail '无法记录遥控PID身份。'
    printf 'spawn_monotonic_s=%s\nteleop_pid=%s\nteleop_start_ticks=%s\n' \
        "$spawn_monotonic" "$teleop_pid" "$teleop_ticks" >>"$run_dir/launch_manifest.txt"
}

wait_for_teleop_ready() {
    local ready=0 attempt
    for ((attempt=0; attempt<200; attempt++)); do
        same_process "$teleop_pid" "$teleop_ticks" || fail "遥控在准备阶段退出；查看$run_dir/teleop.log"
        if verify_teleop_owner && check_light_state startup --max-age 2 --quiet; then
            [[ "$(ps -o pgid= -p "$teleop_pid" | tr -d ' ')" == "$teleop_pid" ]] || fail '遥控未处于独立进程组。'
            ready=1
            break
        fi
        sleep 0.1
    done
    ((ready)) || fail "等待同一遥控控制器灯光就绪超时；查看$run_dir/teleop.log"
    echo "已确认遥控PID=$teleop_pid、UDP3333唯一属主、10ms心跳和${brightness}%双灯提交；等待${settle_s}秒。"
}

settle_teleop_light() {
    local settle_steps attempt
    settle_steps="$(awk -v value="$settle_s" 'BEGIN {printf "%d", value * 10}')"
    for ((attempt=0; attempt<settle_steps; attempt++)); do
        verify_teleop_owner && check_light_state running --quiet || fail '稳定等待期间控制器/灯光状态失效。'
        sleep 0.1
    done
}

start_mapping() {
    wbrov_enable_dds_profile || fail 'DDS 图像通信配置校验失败。'
    local attempt
    export CALIB_AB_PROFILE=custom CALIB_AB_CALIBRATION_FILE="$calibration_file" CALIB_AB_LIGHT_MODE=teleop CALIB_AB_DURATION_S=0
    unset CALIB_AB_LIGHT_HOOK
    if [[ "$mapping_mode" == localize-old ]]; then
        unset CALIB_AB_CAMERA_PROFILE RTAB_ENABLE_IMU
        export CALIB_AB_LOCALIZATION_SOURCE_DB="$localization_source_db"
        export CALIB_AB_LOCALIZATION_SOURCE_SHA256="${frozen_hashes[localization_source_db]}"
        unset CALIB_AB_CONTINUE_RUN_DIR
    elif [[ "$mapping_mode" == continue ]]; then
        export CALIB_AB_CAMERA_PROFILE=dual_hd1200_60_odom384x240_20_map960x600_1_odom_ack
        export RTAB_ENABLE_IMU=1
        unset CALIB_AB_LOCALIZATION_SOURCE_DB CALIB_AB_LOCALIZATION_SOURCE_SHA256
        export CALIB_AB_CONTINUE_RUN_DIR="$continuation_run"
    else
        export CALIB_AB_CAMERA_PROFILE=dual_hd1200_60_odom384x240_20_map960x600_1_odom_ack
        export RTAB_ENABLE_IMU=1
        unset CALIB_AB_LOCALIZATION_SOURCE_DB CALIB_AB_LOCALIZATION_SOURCE_SHA256
        export CALIB_AB_CONTINUE_RUN_DIR=new
    fi
    setsid "$run_mapping_launcher" >"$run_dir/mapping.log" 2>&1 &
    mapping_pid=$!
    mapping_ticks="$(proc_ticks "$mapping_pid")" || fail '无法记录建图PID身份。'
    for ((attempt=0; attempt<20; attempt++)); do
        [[ "$(ps -o pgid= -p "$mapping_pid" | tr -d ' ')" == "$mapping_pid" ]] && break
        same_process "$mapping_pid" "$mapping_ticks" || fail '建图在独立进程组准备阶段退出。'
        sleep 0.05
    done
    [[ "$(ps -o pgid= -p "$mapping_pid" | tr -d ' ')" == "$mapping_pid" ]] || fail '建图未处于独立进程组。'
    for ((attempt=0; attempt<20; attempt++)); do
        same_mapping_launcher_process "$mapping_pid" "$mapping_ticks" && break
        pid_may_be_running "$mapping_pid" || fail '建图在精确根进程准入前退出。'
        sleep 0.05
    done
    same_mapping_launcher_process "$mapping_pid" "$mapping_ticks" ||
        fail '建图根进程未绑定到冻结Bash inode及精确launcher参数。'
    printf 'mapping_pid=%s\nmapping_start_ticks=%s\n' "$mapping_pid" "$mapping_ticks" >>"$run_dir/launch_manifest.txt"
    if [[ "$mapping_mode" == localize-old ]]; then
        echo '联合入口正在启动旧地图只定位：水下YAML＋NEURAL；不会新增或覆盖旧地图。'
    elif [[ "$mapping_mode" == continue ]]; then
        echo "联合入口正在续建地图：来源=$continuation_run；旧图只从正常停止副本载入，新段需安全事务后才合图。"
    else
        echo '联合入口正在新建地图：水下YAML＋NEURAL；关闭RViz或遥控窗口都会联合退出。'
    fi
    printf '正在核验运行库并启动建图节点（启动检查最多等待%s秒，启动后不限制建图时长）；完成前请保持静止。\n' "$mapping_child_admission_timeout_s"
}

monitor_mapping() {
    local mapping_children_ready=0 admission_polls=0
    local ready_count child child_pid candidate_ticks candidate_group pid_snapshot observed_child observed_pid extra
    local admission_started admission_deadline admission_now
    local -a missing_children=()
    local -A observed_pids=()
    admission_started="$(monotonic_uptime_s)" || fail '无法建立建图子进程准入的单调时钟。'
    admission_deadline="$(awk -v start="$admission_started" -v timeout="$mapping_child_admission_timeout_s" \
        'BEGIN {printf "%.6f", start + timeout}')" || fail '无法计算建图子进程准入截止时间。'
    while :; do
        if mapping_is_exiting; then
            ((mapping_children_ready)) || fail '建图在全部必需子进程完成有界身份准入前进入退出。'
            # The launcher publishes "stopping" before its orderly child
            # cleanup. Its final lifecycle result distinguishes a critical
            # child failure from a normal RViz/session close without racing
            # this wrapper against that cleanup.
            return 0
        fi
        same_process "$teleop_pid" "$teleop_ticks" '(^|[ /])teleop_keyboard\.py( |$)' || fail '遥控已退出，立即停止建图。'
        if ! same_mapping_launcher_process "$mapping_pid" "$mapping_ticks"; then
            if pid_may_be_running "$mapping_pid"; then
                fail '建图PID仍存在但身份失效；禁止无界wait，进入联合安全退出。'
            fi
            reap_mapping_status || fail '无法安全取得建图退出码。'
            return "$mapping_exit_status"
        fi
        verify_teleop_owner && check_light_state running --quiet || fail '控制器属主或灯光状态变化，停止本次建图。'

        observed_pids=()
        if ((mapping_children_ready == 0)); then
            pid_snapshot="$(snapshot_mapping_pid_files "${expected_mapping_children[@]}")" ||
                fail '无法安全读取建图子进程PID文件快照。'
            while IFS=$'\t' read -r observed_child observed_pid extra; do
                [[ -n "$observed_child" ]] || continue
                [[ -z "$extra" && "$observed_pid" =~ ^[1-9][0-9]*$ ]] ||
                    fail '安全PID快照输出格式异常。'
                [[ -v "child_patterns[$observed_child]" ]] ||
                    fail "安全PID快照含未知子进程名: $observed_child"
                [[ -z "${observed_pids[$observed_child]:-}" ]] ||
                    fail "安全PID快照重复记录子进程: $observed_child"
                observed_pids[$observed_child]="$observed_pid"
            done <<< "$pid_snapshot"
        fi

        # Bind every required child to this private mapping group and process
        # instance. PID files are used only for first admission; afterward the
        # immutable PID/startticks identity is checked directly via /proc.
        ready_count=0
        missing_children=()
        for child in "${expected_mapping_children[@]}"; do
            if [[ -n "${child_pids[$child]:-}" ]]; then
                child_pid="${child_pids[$child]}"
                candidate_ticks="${child_ticks[$child]}"
                candidate_group="$(ps -o pgid= -p "$child_pid" 2>/dev/null | tr -d ' ')" || candidate_group=""
                if [[ "$candidate_group" != "$mapping_pid" ]] ||
                   ! mapping_child_identity_matches "$child" "$child_pid" "$candidate_ticks" 0; then
                    mapping_is_exiting && return 0
                    fail "建图必需子进程$child退出或精确身份失效。"
                fi
                ready_count=$((ready_count + 1))
                continue
            fi

            child_pid="${observed_pids[$child]:-}"
            if [[ -n "$child_pid" ]]; then
                candidate_ticks="$(proc_ticks "$child_pid" || true)"
                candidate_group="$(ps -o pgid= -p "$child_pid" 2>/dev/null | tr -d ' ')" || candidate_group=""
                if [[ "$candidate_ticks" =~ ^[0-9]+$ ]] &&
                   ((candidate_ticks >= mapping_ticks)) &&
                   [[ "$candidate_group" == "$mapping_pid" ]] &&
                   mapping_child_identity_matches "$child" "$child_pid" "$candidate_ticks" 1; then
                    child_pids[$child]="$child_pid"
                    child_ticks[$child]="$candidate_ticks"
                    ready_count=$((ready_count + 1))
                    continue
                fi
            fi
            missing_children+=("$child")
        done
        if ((ready_count == ${#expected_mapping_children[@]} && mapping_children_ready == 0)); then
            mapping_children_ready=1
            echo '建图各必需子进程身份（含最终Core inode/SHA）已确认；请先目视确认灯亮和深度图，再用遥控移动。'
        elif ((mapping_children_ready == 0)); then
            admission_polls=$((admission_polls + 1))
            admission_now="$(monotonic_uptime_s)" || fail '无法读取建图子进程准入的单调时钟。'
            if ((admission_polls >= mapping_child_admission_max_polls)) ||
               awk -v now="$admission_now" -v deadline="$admission_deadline" \
                   'BEGIN {exit !(now >= deadline)}'; then
                fail "运行库预检及建图子进程准入超过${mapping_child_admission_timeout_s}秒；未通过: ${missing_children[*]}"
            fi
        fi
        sleep 0.25
    done
}

main() {
    if ((check_only == 0)); then
        require_interactive
    fi
    select_mapping_mode
    preflight_read_only
    capture_input_hashes
    if ((check_only)); then
        emit_frozen_manifest
        echo '[只读预检通过] 未创建目录/锁文件/运行记录，未删除ready，未请求OK，未启动遥控、灯光或相机。'
        echo '这里只是当前先决条件快照；正常启动仍会重新检查并要求现场OK。'
        return 0
    fi
    if ! confirm_operator; then
        echo '已取消，没有连接飞控或打开相机。'
        return 0
    fi
    # Recheck after the operator's potentially long confirmation delay.
    preflight_read_only
    verify_input_hashes
    prepare_runtime
    start_teleop
    wait_for_teleop_ready
    settle_teleop_light
    verify_input_hashes
    start_mapping
    monitor_mapping
}

trap on_exit EXIT
trap 'trap "" INT TERM HUP; exit 130' INT
trap 'trap "" INT TERM HUP; exit 143' TERM
trap 'trap "" INT TERM HUP; exit 129' HUP
main
