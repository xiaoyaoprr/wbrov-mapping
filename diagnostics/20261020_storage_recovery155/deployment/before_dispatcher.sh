#!/usr/bin/bash
set -euo pipefail
mode="${WBROV_MAPPING_MODE:-}"
check_only=0
args=("$@")
while (($#)); do
    case "$1" in
        --mode) (($# >= 2)) || exit 2; mode="$2"; shift 2 ;;
        --check-only) check_only=1; shift ;;
        --help|-h) printf '%s\n' '用法: 8号 [--check-only] [--mode new|continue|localize-old]'; exit 0 ;;
        *) shift ;;
    esac
done
if [[ -z "$mode" ]]; then
    if ((check_only)); then
        mode=new
    else
        printf '\n请选择运行方式：\n  1) 新建地图（持续采集、后台逐帧处理）\n  2) 续建旧版正常保存的地图\n  3) 旧版地图只定位\n'
        read -r -p '输入 1、2 或 3：' choice
        case "$choice" in
            1) mode=new ;;
            2) mode=continue ;;
            3) mode=localize-old ;;
            *) echo '无效选择。' >&2; exit 2 ;;
        esac
    fi
fi
case "$mode" in
    new|mapping) target=/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018/mapping_combo42_camera6/run_continuous_mapping_with_light.sh; pin=727683484e856fc393e494810fe8d2a929a8bec740046162f82ff658e11ee1ec; mode=new ;;
    continue|localize-old) target='/home/exocean/Desktop/机器人键盘遥控/.tools/run_water_mapping_with_light.pre_combo42.sh'; pin=34cf9b3c35fa4f7977721bb94f4786d59bf662c6c552d202c14123ab06bb03d2 ;;
    *) echo '不支持的运行方式。' >&2; exit 2 ;;
esac
[[ -f "$target" && ! -L "$target" ]] || exit 1
[[ "$(sha256sum -- "$target" | cut -d ' ' -f 1)" == "$pin" ]] || { echo '入口文件校验失败。' >&2; exit 1; }
exec /usr/bin/bash "$target" "${args[@]}" --mode "$mode"
