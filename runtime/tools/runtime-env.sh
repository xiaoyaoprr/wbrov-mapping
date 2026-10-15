#!/usr/bin/env bash
set -euo pipefail
runtime_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
unset LD_PRELOAD LD_AUDIT PYTHONHOME
export PATH="$runtime_root/bin:/opt/ros/humble/bin:/usr/bin:/bin"
export LD_LIBRARY_PATH="$runtime_root/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu"
export AMENT_PREFIX_PATH="$runtime_root:/opt/ros/humble"
export COLCON_PREFIX_PATH="$runtime_root:/opt/ros/humble"
export PYTHONPATH="$runtime_root/local/lib/python3.10/dist-packages":/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages
exec "$@"
