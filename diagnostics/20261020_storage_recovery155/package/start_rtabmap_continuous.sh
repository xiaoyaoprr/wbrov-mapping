#!/usr/bin/bash
set -euo pipefail
umask 077
export RTAB_DEPTH_MODE=NEURAL
readonly package=/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018/mapping_round6_candidate155_recovery
readonly runner="$package/tools/continuous_mapping_session.py"
readonly config="$package/entry.json"
readonly config_pin=6a3f5675da088ff6f2d5b27ab081a353d2d899c9c2856ac600867e1a27f1e104
[[ "$(sha256sum -- "$runner" | cut -d ' ' -f 1)" == 8c2dad903c7326a33e8c594f1167dbb453546af27b07a013835d0b3bbada86a8 ]] || exit 1
if [[ "${1:-}" == --print-range-config ]]; then
    printf '%s\n' 'tracking_depth_z_m=0.20-1.50' 'dense_depth_z_m=0.20-1.50' 'apriltag_radial_m=0.20-0.75'
    exit 0
fi
if [[ "${1:-}" == --check-only ]]; then
    exec /usr/bin/python3 -B "$runner" --config "$config" --config-sha256 "$config_pin" --check-only
fi
(($# == 0)) || exit 2
[[ "${CALIB_AB_CONTINUE_RUN_DIR:-new}" == new ]] || { echo '此入口仅支持新建连续地图。' >&2; exit 2; }
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
# lifecycle.json is owned by the session and bound to this Bash PID/start time.
runner_pid=''
request_stop() { [[ -z "$runner_pid" ]] || kill -USR1 "$runner_pid" 2>/dev/null || true; }
interrupted=0
trap 'interrupted=1; request_stop' INT TERM HUP
/usr/bin/python3 -B "$runner" --config "$config" --config-sha256 "$config_pin" --state "$HOME/.cache/rtabmap_dense_calib_ab" --output-root "$HOME/maps/rtabmap_calibration_ab" --launcher-pid "$$" &
runner_pid=$!
set +e
while :; do
    interrupted=0
    wait "$runner_pid"; result=$?
    ((interrupted)) && continue
    kill -0 "$runner_pid" 2>/dev/null || break
done
exit "$result"
