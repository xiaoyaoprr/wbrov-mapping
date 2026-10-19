#!/usr/bin/env python3
"""Render a new-map-only entry from the reviewed teleop/light wrapper.

Control launch, admission and shutdown code stays in that wrapper. Legacy
continue/localization retain their separate installed entry. No motor tests.
"""
from pathlib import Path
import argparse
import hashlib
import re
import shlex


def render(package,config_pin,runner_pin,base,worker_pin):
    if not all(re.fullmatch(r'[0-9a-f]{64}',pin) for pin in (config_pin,runner_pin,worker_pin)):
        raise ValueError('explicit SHA-256 identities required')
    q=shlex.quote
    launcher='''#!/usr/bin/bash
set -euo pipefail
umask 077
export RTAB_DEPTH_MODE=NEURAL
readonly package=PACKAGE
readonly runner="$package/tools/continuous_mapping_session.py"
readonly config="$package/entry.json"
readonly config_pin=CONFIGPIN
[[ "$(sha256sum -- "$runner" | cut -d ' ' -f 1)" == RUNNERPIN ]] || exit 1
if [[ "${1:-}" == --print-range-config ]]; then
    printf '%s\\n' 'tracking_depth_z_m=0.20-1.50' 'dense_depth_z_m=0.20-1.50' 'apriltag_radial_m=0.20-0.75'
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
'''.replace('PACKAGE',q(str(package))).replace('CONFIGPIN',q(config_pin)).replace('RUNNERPIN',q(runner_pin))
    launcher_pin=hashlib.sha256(launcher.encode()).hexdigest()
    wrapper=base
    old='/home/exocean/.cache/rtabmap_dense/calib_ab_20260826/start_rtabmap_calib_ab.sh'
    wrapper=wrapper.replace(old,str(package/'start_rtabmap_continuous.sh'))
    wrapper=wrapper.replace("start_rtabmap_calib_ab\\.sh","start_rtabmap_continuous\\.sh")
    wrapper=re.sub(r"readonly FINAL_MAPPING_LAUNCHER_SHA256='[^']+'",f"readonly FINAL_MAPPING_LAUNCHER_SHA256='{launcher_pin}'",wrapper)
    wrapper=re.sub(r"readonly FINAL_RESET_RUNTIME_ROOT='[^']+'",f"readonly FINAL_RESET_RUNTIME_ROOT='{package}'",wrapper)
    wrapper=re.sub(r"readonly FINAL_RESET_RUNTIME_NODE_SHA256='[^']+'",f"readonly FINAL_RESET_RUNTIME_NODE_SHA256='{worker_pin}'",wrapper)
    wrapper=wrapper.replace('"$FINAL_RESET_RUNTIME_ROOT/bin/rtabmap"','"$FINAL_RESET_RUNTIME_ROOT/bin/wbrov_replay_worker"')
    start=wrapper.index('select_mapping_mode() {');end=wrapper.index('\nproc_ticks() {',start)
    wrapper=wrapper[:start]+'''select_mapping_mode() {
    [[ -z "$mapping_mode" || "$mapping_mode" == new || "$mapping_mode" == mapping ]] || fail '连续采集入口本轮只验收新建地图；续建与旧图定位请使用原入口。'
    [[ -z "$localization_source_db" ]] && ((continuation_run_explicit == 0)) || fail '新建地图不能带续建/旧图参数。'
    mapping_mode=new
    expected_mapping_children=(bridge processing graph_display slam rviz)
    frozen_paths[mapping_launcher]="$mapping_launcher"
    child_patterns[processing]='deferred_mapping\\.py'
    child_patterns[graph_display]='deferred_map_display\\.py'
    child_patterns[rviz]='(^|[ /])rviz2( |$)'
}
'''+wrapper[end:]
    start=wrapper.index('    grep -Fq "trap');end=wrapper.index('    # This dedicated mode',start)
    wrapper=wrapper[:start]+'''    "$mapping_launcher" --check-only || fail '连续建图运行文件或配置校验失败。'
'''+wrapper[end:]
    wrapper=wrapper.replace('count<attempts; count++','attempts == 0 || count<attempts; count++')
    wrapper=wrapper.replace('if wait_mapping_launcher 1200; then','if wait_mapping_launcher 0; then')
    # Only after the session exited can remainder cleanup run. Never terminate
    # the ordered worker just because drain time exceeded a legacy threshold.
    wrapper=wrapper.replace('if ((mapping_identity_uncertain == 0)) && ! signal_mapping_remainders TERM; then',
        'if ((mapping_identity_uncertain == 0)) && ! pid_may_be_running "$mapping_pid" && ! signal_mapping_remainders TERM; then')
    start=wrapper.index('queue_completed_navigation_map() {');end=wrapper.index('\nshow_mapping_failure_log()',start)
    wrapper=wrapper[:start]+'''queue_completed_navigation_map() {
    # This release is mapping-only. Its historical pose is not live navigation.
    return 0
}
'''+wrapper[end:]
    wrapper=wrapper.replace('水下遥控＋自动开灯＋RTAB-Map新建地图','水下遥控＋自动开灯＋持续采集建图')
    wrapper=wrapper.replace('已完成停止信号步骤；现在分别等待遥控锁定/关灯退出与建图保存。',
        '已发送停止采集与遥控退出请求；遥控先锁定关灯，后台处理完全部积压帧再保存，请勿断开Jetson电源。')
    wrapper=wrapper.replace('联合入口正在新建地图：水下YAML＋NEURAL；关闭RViz或遥控窗口都会联合退出。',
        '正在持续采集建图：画面实时、点云按处理进度更新；关闭RViz或遥控会停止采集，处理完积压帧后保存。')
    return launcher,wrapper


def main():
    p=argparse.ArgumentParser();p.add_argument('--package',type=Path,required=True)
    p.add_argument('--config-sha256',required=True);p.add_argument('--runner-sha256',required=True)
    p.add_argument('--worker-sha256',required=True)
    p.add_argument('--base-wrapper',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(exist_ok=True)
    for name,data in zip(['start_rtabmap_continuous.sh','run_continuous_mapping_with_light.sh'],render(a.package,a.config_sha256,a.runner_sha256,a.base_wrapper.read_text(encoding='utf-8'),a.worker_sha256)):
        (a.output/name).write_text(data,encoding='utf-8',newline='\n')


if __name__=='__main__':main()
