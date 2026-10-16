#!/usr/bin/python3
"""Strict verifier for the isolated WBROV AArch64 RTAB-Map runtime.

This verifier has three modes:

* ``tree`` verifies the immutable, content-addressed package tree, every
  recorded file identity/hash, AArch64 ELF metadata, absence of RPATH and
  RUNPATH, and the recursive ``ldd -r`` closure.
* ``process`` additionally binds a live PID to an exact entrypoint inode,
  UID, start-ticks value and its current ``/proc/<pid>/maps`` mappings.
* ``proc-maps`` audits either a live PID's maps or a saved maps snapshot.

The checker deliberately does not source a ROS setup file and discards the
caller's LD_LIBRARY_PATH.  The only accepted dependency locations are the
verified runtime root, /opt/ros/humble, and operating-system library trees.
No database or ROS graph is opened by this program.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import stat
import subprocess
import sys
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Set, Tuple


SCHEMA_VERSION = 3
PACKAGE_KIND = "wbrov-private-rtabmap-runtime"
EXPECTED_PRIVATE_ROOT = '/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round3_deploy_20261016'
MANIFEST_NAME = "runtime-manifest.json"
MAX_MANIFEST_BYTES = 32 * 1024 * 1024
MAX_TREE_ENTRIES = 100000
MAX_MAPS_BYTES = 64 * 1024 * 1024
MAX_MAPS_RECORDS = 1000000
READELF = "/usr/bin/readelf"
LDD = "/usr/bin/ldd"

SOURCE_HASHES = {
    "rtabmap_core/corelib/src/Parameters.cpp": "fcf545c4e92742ee89517b4c6f69c930e593f46745c24b84a1571a8c8af8cc7b",
    "rtabmap_core/CMakeLists.txt": "9d11b078e265cbf55b6f55564fa9bb255bce513e41fad66aa39aa2a5291f189d",
    "rtabmap_core/corelib/include/rtabmap/core/DBDriver.h": "b356683ffbd4201721e10a6665880862cc699d37794bb9527abbd2ec44b933eb",
    "rtabmap_core/corelib/include/rtabmap/core/DBDriverSqlite3.h": "0a0c42db53cbeaffe4af958a0e6eb0d092108274f1425998628ac2b15de8862c",
    "rtabmap_core/corelib/include/rtabmap/core/ImageInputReceipt.h": "9c0a7fb9d4775d4b5184b3114b3eb40b0e94f3d29f0484325df17e182de04cfe",
    "rtabmap_core/corelib/include/rtabmap/core/Memory.h": "99a6c1d70cfcc6f8f0537040440962b0625bead8d4eb51677219443d56510c23",
    "rtabmap_core/corelib/include/rtabmap/core/Parameters.h": "7b67106cdc48ae11ac025cb08e29ef976682bb98cf8018d986e76c224b58e94b",
    "rtabmap_core/corelib/include/rtabmap/core/Rtabmap.h": "6df7a708295d8ab9c3b0f0daf344f291a337d49f9ae7071453940cf3fc690c0d",
    "rtabmap_core/corelib/include/rtabmap/core/optimizer/OptimizerG2O.h": "75ff6492cb7016e1f3f1a4d0012ad33000d9334da5eb9b2a1ba1da0e9cf4f752",
    "rtabmap_core/corelib/src/DBDriver.cpp": "bf6667683ac68cb4c0ffd7344fff5ffbce3888fd8023affb88309140cda70b91",
    "rtabmap_core/corelib/src/DBDriverSqlite3.cpp": "fb0e6cde7bc7f1470374cc5ce9518d80b92bce48e0ae32f1c1688f80beaf142e",
    "rtabmap_core/corelib/src/EarlyGravityCertificate.h": "143cf85afb1503bcdaf25a91abdf7baaea27f689c11ee9f65f801f8606715bca",
    "rtabmap_core/corelib/src/Memory.cpp": "ec5f623b71890df86eecd0ebdc5aa2bcbfc3ccfaa0a0e1b14e0bdb7b444e812f",
    "rtabmap_core/corelib/src/Rtabmap.cpp": "8706f82eebcb0c444f89edb26683be4cf0d2678e8ea3b490b6c27cb6e14d9fd6",
    "rtabmap_core/corelib/src/early_component_gate.inc": "9a64253c4af72d7aebab96353d130920c305094c61700442fe917a035b5be16e",
    "rtabmap_core/corelib/src/early_shadow_guard.inc": "240682e319bc57cb45f840c874e02b7d737e2d87fbd429f11941bace1a2192d4",
    "rtabmap_core/corelib/src/odometry/DepthGeometryRecovery.h": "ac904ee5a0e3c9c6de35c4b049f39828f79636a92a3d4b0c81f59bd2cb17c76a",
    "rtabmap_core/corelib/src/odometry/OdometryF2M.cpp": "6529a78d285aece8408795384c7b0f1ddb5956a2ad966facf018fcceadd80bbc",
    "rtabmap_core/corelib/src/optimizer/OptimizerG2O.cpp": "e7e6a4fb2c2ba8fac1f9f7b891ec7e872394ef711ff244ce5dc213d8e12198fc",
    "rtabmap_core/corelib/src/optimizer_trace.inc": "ca20901de5d2856aa0e45586c7f9e4cbfdd595ed84dd435d9d0bcd66518adc53",
    "rtabmap_core/corelib/src/recovery_geometry.inc": "c28a5c4b7ccaa9f533ac18da9056939f024fde241b31b7de017df265a3d43992",
    "rtabmap_core/corelib/src/safe_gravity_residual.inc": "ac88ab9dbfcfb934a32ee840ad1a793d1282c195a44a949274801e110b070a24",
    "rtabmap_core/corelib/src/visual_recovery.inc": "79874bf7510e6d6d1538082550e2f85daab93130ae5e6c1a5c3ab23674ed97d2",
    "rtabmap_ros/rtabmap_conversions/src/MsgConversion.cpp": "ef9e331097a1aa4893b510c2aae7193a664b806eeca4a7019321809b9178eefa",
    "rtabmap_ros/rtabmap_odom/include/rtabmap_odom/OdometryROS.h": "5f56ae4da35a441085c68f7b2b727a9d5437bb42795bae461f5f637c0c3d3d29",
    "rtabmap_ros/rtabmap_odom/src/OdometryROS.cpp": "7f17a68d58985bc620a303b1437f5df422d001fc3555b3c4c558e27c0ad2aeee",
    "rtabmap_ros/rtabmap_slam/include/rtabmap_slam/CoreWrapper.h": "7628d97bf8a225451d2fb33f97d8e3bd79baf8ddb672daef032e4b8f51a68362",
    "rtabmap_ros/rtabmap_slam/include/rtabmap_slam/GlobalPublicationGate.h": "daedd4e70da4b7d207285d8f303ba9e114b8b006fe5f7e46ed1e8f528637bf51",
    "rtabmap_ros/rtabmap_slam/include/rtabmap_slam/JumpQuarantineGate.h": "a24d09c476743889db76171ef5f19393ad7c053549441de48eb9c4bb9236e0ca",
    "rtabmap_ros/rtabmap_slam/include/rtabmap_slam/ManagedImuEpochBuffer.h": "7b72f4071377c8e66bdc4d2c6daaab654be0802cdeba19142022575aa2a5d22f",
    "rtabmap_ros/rtabmap_slam/include/rtabmap_slam/MappingContinuityGate.h": "a243c950b19e17cb5c807876017c5736ab81f1cfe428c4bef90b5af61dd4ce3d",
    "rtabmap_ros/rtabmap_slam/include/rtabmap_slam/SubmapPublisher.h": "2bf792796cb378ef5ea65f595c403fa358da7d8778e0b20de2b635e9d851cd57",
    "rtabmap_ros/rtabmap_slam/src/CoreNode.cpp": "ee64bddc87e3e35b5d063fbe5f0dd4fb37ccc26e23479ce78bf131a6f2a0d851",
    "rtabmap_ros/rtabmap_slam/src/CoreWrapper.cpp": "b745387ad9601a03fb09ae004053bdc2180121946fbb0351e3288ffd7f5f19ea"
}

ENTRYPOINTS = {
    "rtabmap": "bin/rtabmap",
    "rgbd_odometry": "bin/rgbd_odometry",
    "point_cloud_xyzrgb": "bin/point_cloud_xyzrgb",
    "rtabmap_export": "bin/rtabmap-export",
    "header_health_observer": "bin/header_health_observer",
    "safe_reloc_gid_frontend": "bin/safe_reloc_gid_frontend",
}

RUNTIME_HELPERS = {
    "corner_recovery_config_renderer": "libexec/wbrov_corner_recovery/render_config.py",
    "corner_recovery_config_template": "libexec/wbrov_corner_recovery/production_config_template.json",
    "corner_recovery_consumer": "libexec/wbrov_corner_recovery/consumer.py",
    "corner_recovery_module_bridge_status": "libexec/wbrov_corner_recovery/bridge_status.py",
    "corner_recovery_module_core_pixel_consumer": "libexec/wbrov_corner_recovery/core_pixel_consumer.py",
    "corner_recovery_module_core_pixel_join": "libexec/wbrov_corner_recovery/core_pixel_join.py",
    "corner_recovery_module_decoded_pixel_metadata": "libexec/wbrov_corner_recovery/decoded_pixel_metadata.py",
    "corner_recovery_module_edge_refinement": "libexec/wbrov_corner_recovery/edge_refinement.py",
    "corner_recovery_module_live_bridge": "libexec/wbrov_corner_recovery/live_bridge.py",
    "corner_recovery_module_managed_graph_state": "libexec/wbrov_corner_recovery/managed_graph_state.py",
    "corner_recovery_module_measured_bridge": "libexec/wbrov_corner_recovery/measured_bridge.py",
    "corner_recovery_module_message_info_executor": "libexec/wbrov_corner_recovery/message_info_executor.py",
    "corner_recovery_module_tag_pixel_record": "libexec/wbrov_corner_recovery/tag_pixel_record.py",
    "corner_recovery_module_verified_corners": "libexec/wbrov_corner_recovery/verified_corners.py",
    "corner_recovery_module_verified_pixel_observer": "libexec/wbrov_corner_recovery/verified_pixel_observer.py",
    "safe_reloc_engine_core": "libexec/safe_reloc_engine_core.py",
    "safe_reloc_ipc_engine": "libexec/safe_reloc_ipc_engine.py",
    "submap_display_graph_display_geometry": "libexec/graph_display_geometry.py",
    "submap_display_rtabmap_graph_display": "libexec/rtabmap_graph_display.py",
    "submap_display_rtabmap_submap_display": "libexec/rtabmap_submap_display.py",
    "submap_display_submap_display_geometry": "libexec/submap_display_geometry.py",
    "submap_display_submap_rviz_config": "libexec/submap_rviz_config.py",
    "write_process_identity": "libexec/write_process_identity.py"
}

# Bytes staged by the independently reviewed ARM observer/frontend/engine
# pipeline.  These are source inputs to the package rather than RTAB-Map
# build outputs, so bind them explicitly as well as through the final payload
# hashes recorded below.
AUXILIARY_INPUT_HASHES = {
    "corner_recovery_config_renderer": "74957100aa6f959868b55efa5c8effadb536342f7774e95fb1099aa3a9ca620e",
    "corner_recovery_config_template": "32646c8505394de531af3041fad50578ee7ab57ccfca2f0e23cf87efe2165063",
    "corner_recovery_consumer": "6b7b4b44526c3e9b472b0772531ec4292725ecbe83e7c30d1c27354d8dcee36f",
    "corner_recovery_module_bridge_status": "fbb02cf5795781f10b53f3099dcd2649944bde6a53f9175a14ef8ad843adc78f",
    "corner_recovery_module_core_pixel_consumer": "6a003bbf0e60163bd77ee10775f0690b06818fd1325ccdb39cc9bba0b7f85f80",
    "corner_recovery_module_core_pixel_join": "5b0bc22205258afde156ce3e1e4d0047fbddedb8f159b2a76380428c04db8d4d",
    "corner_recovery_module_decoded_pixel_metadata": "acb9f4778160b7fadac4db7fa1a3443a5f08cd238f634aba7a02704aab903bfc",
    "corner_recovery_module_edge_refinement": "2b030f5a6948290d62cb972b9b21be9df26f2de99d588e59ffc6d4cf34abdb33",
    "corner_recovery_module_live_bridge": "f43cdf06645c40152329dec303d718b5775fa29a83a47c964a84a05450ac1a15",
    "corner_recovery_module_managed_graph_state": "896826dfd4cd0c119513bb82fed10731ad70c5c638547ab30114722dbeea6de1",
    "corner_recovery_module_measured_bridge": "b035c7b9a1cdb8979e7c9d4011a20cf70c1b78a7e6993fdc90cbb3900d1030fa",
    "corner_recovery_module_message_info_executor": "fa4658355deb48d46e47595059ed151087322017bc3148c7ad64cdf6806797f0",
    "corner_recovery_module_tag_pixel_record": "d0dcf6bdc59f8f9522cacfa64bb7087c8ebb9e6ecbfac574632a38034f73db93",
    "corner_recovery_module_verified_corners": "e0a54d003d6f987e55e45f54cafea538a58ebb06e563e9daf1d19e4265bb9b72",
    "corner_recovery_module_verified_pixel_observer": "b8114d17f2da6a3115960d6c34dd0d49df09c696682176698add679002cb7f3d",
    "header_health_observer": "a04aa06117bae16265f4d836406af274b1de105dff9c53e3a6b6bad7c0886018",
    "safe_reloc_engine_core": "314314f138e4057c1b5e074cc31fc98e3c3652bfab26cb0fb9551e3d6464f555",
    "safe_reloc_gid_frontend": "a09155c9aa28dcf297c3373b27730ab0775ba9b50ebef1d00cc8d855611e39bd",
    "safe_reloc_ipc_engine": "e0568a37c397f99e1ae492119c6f3a41f763708ce4122e47024fa5e1c2611ba9",
    "submap_display_graph_display_geometry": "5bd28861322e8c98afedd9d61b313a2f740d143ebfd5e48429f39ba7025977df",
    "submap_display_rtabmap_graph_display": "ef46e5849d35482c14943f6d2ba4463ba1a60b91b2c9a957368af23d7db6c2e6",
    "submap_display_rtabmap_submap_display": "843a002677a2cba18b796f43815e57ff3b0956fd47c7e1929864db0d6e03efdc",
    "submap_display_submap_display_geometry": "21827d469bde4ab74e2892b79e7819ffb9b33d1e982274df6608cd7bd3bf69b0",
    "submap_display_submap_rviz_config": "b07466ecf249349c9048391f834e6f49ee8a21d8e269b7781a667c88a21e791a",
    "write_process_identity": "06b99b3f26955770a6bb3cfb7c6054f8e2fef6655c90ce513e4371d9161143f5"
}

REQUIRED_COMPONENT_KEYS = {
    "core",
    "gui",
    "utilite",
    "conversions_time_guard",
    "msgs",
    "sync",
    "util",
    "odom",
    "odom_plugins",
    "slam_plugins",
}

RESOURCE_PACKAGES = {
    "rtabmap_msgs",
    "rtabmap_conversions",
    "rtabmap_sync",
    "rtabmap_util",
    "rtabmap_odom",
    "rtabmap_slam",
}

ALLOWED_TOP_LEVEL_KEYS = {
    "schema_version",
    "package_kind",
    "architecture",
    "content_address",
    "content_address_algorithm",
    "manifest_excluded_from_payload",
    "build_private_root",
    "package_uid",
    "package_gid",
    "source_hashes",
    "auxiliary_input_hashes",
    "entrypoints",
    "runtime_helpers",
    "required_components",
    "resource_packages",
    "files",
    "provenance",
}

ALLOWED_ENTRY_KEYS = {
    "path",
    "type",
    "mode",
    "uid",
    "gid",
    "dev",
    "ino",
    "dev_major",
    "dev_minor",
    "nlink",
    "size",
    "sha256",
    "link_target",
    "elf",
}

_HEX64 = re.compile(r"^[0-9a-f]{64}$")
_CONTENT_ROOT = re.compile(r"^sha256-([0-9a-f]{64})$")
_LDD_ARROW = re.compile(r"^\s*(\S+)\s+=>\s+(\S+)\s+\(0x[0-9a-fA-F]+\)\s*$")
_LDD_DIRECT = re.compile(r"^\s*(/\S+)\s+\(0x[0-9a-fA-F]+\)\s*$")
_OCTAL_ESCAPE = re.compile(r"\\([0-7]{3})")


class VerificationError(RuntimeError):
    """A package or live-process invariant was violated."""


def fail(message: str) -> None:
    raise VerificationError(message)


def is_exact_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def contained(path: pathlib.Path, parent: pathlib.Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def safe_relative_path(value: Any) -> str:
    if not isinstance(value, str) or not value or "\x00" in value or "\\" in value:
        fail("manifest contains an invalid relative path")
    pure = pathlib.PurePosixPath(value)
    if pure.is_absolute() or value != pure.as_posix() or any(part in ("", ".", "..") for part in pure.parts):
        fail("manifest path is not a normalized contained POSIX path: {!r}".format(value))
    if value == MANIFEST_NAME:
        fail("the self-referential manifest must not appear in payload entries")
    return value


def duplicate_rejecting_object(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            fail("duplicate JSON object key: {!r}".format(key))
        result[key] = value
    return result


def read_manifest(path: pathlib.Path) -> Dict[str, Any]:
    try:
        before = path.lstat()
    except OSError as exc:
        fail("cannot stat manifest: {}".format(exc))
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
        fail("manifest must be a single-link regular file")
    if stat.S_IMODE(before.st_mode) != 0o444:
        fail("manifest mode must be 0444")
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(str(path), flags)
    try:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            fail("manifest identity changed while opening")
        if opened.st_size <= 0 or opened.st_size > MAX_MANIFEST_BYTES:
            fail("manifest size is outside the accepted range")
        raw = b""
        while len(raw) <= MAX_MANIFEST_BYTES:
            block = os.read(fd, min(1024 * 1024, MAX_MANIFEST_BYTES + 1 - len(raw)))
            if not block:
                break
            raw += block
        if len(raw) != opened.st_size:
            fail("manifest size changed while reading")
    finally:
        os.close(fd)
    try:
        parsed = json.loads(
            raw.decode("utf-8", errors="strict"),
            object_pairs_hook=duplicate_rejecting_object,
            parse_constant=lambda token: fail("invalid JSON numeric token: {}".format(token)),
        )
    except VerificationError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        fail("invalid manifest JSON: {}".format(exc))
    if not isinstance(parsed, dict):
        fail("manifest root must be an object")
    return parsed


def sha256_fd(fd: int) -> str:
    digest = hashlib.sha256()
    os.lseek(fd, 0, os.SEEK_SET)
    while True:
        block = os.read(fd, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    return digest.hexdigest()


def sha256_path_nofollow(path: pathlib.Path, expected_stat: os.stat_result) -> str:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(str(path), flags)
    try:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino) != (expected_stat.st_dev, expected_stat.st_ino):
            fail("identity changed while hashing {}".format(path))
        return sha256_fd(fd)
    finally:
        os.close(fd)


def file_semantic(entry: Mapping[str, Any]) -> Dict[str, Any]:
    semantic: Dict[str, Any] = {
        "path": entry["path"],
        "type": entry["type"],
        "mode": entry["mode"],
    }
    if entry["type"] == "regular":
        semantic["size"] = entry["size"]
        semantic["sha256"] = entry["sha256"]
        if "elf" in entry:
            semantic["elf"] = entry["elf"]
    elif entry["type"] == "symlink":
        semantic["link_target"] = entry["link_target"]
    return semantic


def content_basis(manifest: Mapping[str, Any]) -> Dict[str, Any]:
    return {
        "schema_version": manifest["schema_version"],
        "package_kind": manifest["package_kind"],
        "architecture": manifest["architecture"],
        "source_hashes": manifest["source_hashes"],
        "auxiliary_input_hashes": manifest["auxiliary_input_hashes"],
        "entrypoints": manifest["entrypoints"],
        "runtime_helpers": manifest["runtime_helpers"],
        "required_components": manifest["required_components"],
        "resource_packages": manifest["resource_packages"],
        "files": [file_semantic(entry) for entry in manifest["files"]],
    }


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def elf_header(path: pathlib.Path) -> Optional[Dict[str, Any]]:
    with path.open("rb") as stream:
        raw = stream.read(20)
    if len(raw) < 4 or raw[:4] != b"\x7fELF":
        return None
    if len(raw) < 20:
        fail("truncated ELF header: {}".format(path))
    elf_class = raw[4]
    data = raw[5]
    if elf_class != 2:
        fail("ELF is not 64-bit: {}".format(path))
    if data == 1:
        byteorder = "little"
    elif data == 2:
        byteorder = "big"
    else:
        fail("ELF has invalid byte order: {}".format(path))
    machine = int.from_bytes(raw[18:20], byteorder=byteorder, signed=False)
    if machine != 183:
        fail("ELF is not AArch64 (e_machine={}): {}".format(machine, path))
    return {"class": 64, "endianness": byteorder, "machine": "AArch64"}


def read_build_id(path: pathlib.Path) -> Optional[str]:
    result = subprocess.run(
        [READELF, "-n", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        env={"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C"},
        check=False,
    )
    if result.returncode != 0:
        fail("readelf -n failed for {}: {}".format(path, result.stdout.strip()))
    match = re.search(r"Build ID:\s*([0-9a-fA-F]+)", result.stdout)
    return match.group(1).lower() if match else None


def verify_dynamic_tags(path: pathlib.Path) -> None:
    result = subprocess.run(
        [READELF, "-d", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        env={"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C"},
        check=False,
    )
    if result.returncode != 0:
        fail("readelf -d failed for {}: {}".format(path, result.stdout.strip()))
    if "There is no dynamic section" in result.stdout:
        fail("static ELF is not accepted in the private runtime: {}".format(path))
    if re.search(r"\((?:RPATH|RUNPATH)\)", result.stdout):
        fail("RPATH/RUNPATH is forbidden: {}".format(path))


def normalize_runtime_dependency(path: pathlib.Path, runtime_root: pathlib.Path) -> str:
    resolved = path.resolve(strict=True)
    if contained(resolved, runtime_root):
        return "$RUNTIME_ROOT/" + resolved.relative_to(runtime_root).as_posix()
    return str(resolved)


def runtime_environment(runtime_root: pathlib.Path) -> Dict[str, str]:
    ros = pathlib.Path("/opt/ros/humble")
    library_dirs = [runtime_root / "lib", ros / "lib", ros / "lib/aarch64-linux-gnu"]
    python_dirs: List[pathlib.Path] = []
    for pattern in ("lib/python*/site-packages", "lib/python*/dist-packages",
                    "local/lib/python*/site-packages", "local/lib/python*/dist-packages"):
        python_dirs.extend(sorted(runtime_root.glob(pattern)))
    return {
        "PATH": "{}:/opt/ros/humble/bin:/usr/bin:/bin".format(runtime_root / "bin"),
        "LD_LIBRARY_PATH": ":".join(str(path) for path in library_dirs),
        "AMENT_PREFIX_PATH": "{}:/opt/ros/humble".format(runtime_root),
        "PYTHONPATH": ":".join(str(path) for path in python_dirs),
        "LANG": "C",
        "LC_ALL": "C",
    }


def is_external_path_allowed(path: pathlib.Path) -> bool:
    resolved = path.resolve(strict=True)
    allowed = (
        pathlib.Path("/usr/lib").resolve(strict=True),
        pathlib.Path("/lib").resolve(strict=True),
        pathlib.Path("/opt/ros/humble").resolve(strict=True),
    )
    return any(contained(resolved, prefix) or resolved == prefix for prefix in allowed)


def is_rtabmap_object(path: pathlib.Path) -> bool:
    name = path.name.lower()
    return bool(re.match(r"^librtabmap(?:[_.]|$)", name)) or name in {
        "rtabmap", "rgbd_odometry", "point_cloud_xyzrgb", "rtabmap-export"
    }


def rtabmap_mapping_key(path: pathlib.Path) -> str:
    """Return a version-independent identity for dual-mapping detection."""
    name = path.name.lower()
    match = re.match(r"^(librtabmap[^.]*)\.so(?:\..*)?$", name)
    return match.group(1) if match else name


def parse_ldd_output(output: str, subject: pathlib.Path) -> List[Tuple[str, pathlib.Path]]:
    resolved: List[Tuple[str, pathlib.Path]] = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("linux-vdso"):
            continue
        if "undefined symbol:" in line.lower():
            fail("ldd -r found an undefined symbol for {}: {}".format(subject, line))
        if "=> not found" in line or line.endswith(" not found"):
            fail("ldd -r found a missing dependency for {}: {}".format(subject, line))
        arrow = _LDD_ARROW.match(raw_line)
        if arrow:
            resolved.append((arrow.group(1), pathlib.Path(arrow.group(2))))
            continue
        direct = _LDD_DIRECT.match(raw_line)
        if direct:
            direct_path = pathlib.Path(direct.group(1))
            resolved.append((direct_path.name, direct_path))
            continue
        if line.startswith("statically linked") or line.startswith("not a dynamic executable"):
            fail("ldd rejected dynamic ELF {}: {}".format(subject, line))
        # glibc may emit harmless version headings.  Everything else is kept
        # visible as a hard error so a new ldd format cannot bypass the policy.
        if line.endswith(":") or line.startswith("Version information:"):
            continue
        if re.match(r"^\S+:\s+\(.+\)$", line):
            continue
        fail("unrecognized ldd -r output for {}: {}".format(subject, line))
    return resolved


def ldd_dependencies(path: pathlib.Path, runtime_root: pathlib.Path) -> List[Tuple[str, pathlib.Path]]:
    result = subprocess.run(
        [LDD, "-r", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        env=runtime_environment(runtime_root),
        check=False,
    )
    if result.returncode != 0:
        fail("ldd -r failed for {} (rc={}): {}".format(path, result.returncode, result.stdout.strip()))
    return parse_ldd_output(result.stdout, path)


def actual_payload_paths(runtime_root: pathlib.Path) -> Set[str]:
    found: Set[str] = set()
    stack = [runtime_root]
    while stack:
        directory = stack.pop()
        with os.scandir(directory) as iterator:
            for item in iterator:
                path = pathlib.Path(item.path)
                rel = path.relative_to(runtime_root).as_posix()
                if rel == MANIFEST_NAME:
                    continue
                found.add(rel)
                if item.is_dir(follow_symlinks=False):
                    stack.append(path)
                elif not (item.is_file(follow_symlinks=False) or item.is_symlink()):
                    fail("special file is forbidden in runtime tree: {}".format(path))
                if len(found) > MAX_TREE_ENTRIES:
                    fail("runtime tree exceeds entry limit")
    return found


def validate_manifest_schema(manifest: MutableMapping[str, Any], runtime_root: pathlib.Path) -> List[Dict[str, Any]]:
    if set(manifest) != ALLOWED_TOP_LEVEL_KEYS:
        fail("manifest top-level keys differ: missing={} extra={}".format(
            sorted(ALLOWED_TOP_LEVEL_KEYS - set(manifest)), sorted(set(manifest) - ALLOWED_TOP_LEVEL_KEYS)))
    if manifest["schema_version"] != SCHEMA_VERSION:
        fail("unsupported runtime manifest schema")
    if manifest["package_kind"] != PACKAGE_KIND or manifest["architecture"] != "AArch64":
        fail("wrong package kind or architecture")
    if manifest["manifest_excluded_from_payload"] != MANIFEST_NAME:
        fail("manifest self-exclusion contract changed")
    if manifest["build_private_root"] != EXPECTED_PRIVATE_ROOT:
        fail("runtime was not built from the reviewed private root")
    if manifest["source_hashes"] != SOURCE_HASHES:
        fail("frozen source hash set differs")
    if manifest["auxiliary_input_hashes"] != AUXILIARY_INPUT_HASHES:
        fail("reviewed auxiliary-input hash set differs")
    if manifest["entrypoints"] != ENTRYPOINTS:
        fail("entrypoint layout differs")
    if manifest["runtime_helpers"] != RUNTIME_HELPERS:
        fail("runtime helper layout differs")
    if not is_exact_int(manifest["package_uid"]) or not is_exact_int(manifest["package_gid"]):
        fail("package uid/gid must be integers")
    components = manifest["required_components"]
    if not isinstance(components, dict) or set(components) != REQUIRED_COMPONENT_KEYS:
        fail("required runtime component set differs")
    for component, paths in components.items():
        if not isinstance(paths, list) or not paths:
            fail("component {} has no packaged object".format(component))
        for value in paths:
            safe_relative_path(value)
            if not value.startswith("lib/"):
                fail("component {} does not resolve below runtime lib/".format(component))
    resources = manifest["resource_packages"]
    if not isinstance(resources, list) or set(resources) != RESOURCE_PACKAGES or len(resources) != len(RESOURCE_PACKAGES):
        fail("runtime resource package set differs")
    files = manifest["files"]
    if not isinstance(files, list) or not files or len(files) > MAX_TREE_ENTRIES:
        fail("manifest files array has invalid size")
    entries: List[Dict[str, Any]] = []
    seen: Set[str] = set()
    previous = ""
    for raw in files:
        if not isinstance(raw, dict) or not set(raw).issubset(ALLOWED_ENTRY_KEYS):
            fail("invalid manifest file entry keys")
        path = safe_relative_path(raw.get("path"))
        if path in seen or (previous and path <= previous):
            fail("manifest file paths must be unique and bytewise sorted")
        seen.add(path)
        previous = path
        kind = raw.get("type")
        required_identity = ("mode", "uid", "gid", "dev", "ino", "dev_major", "dev_minor", "nlink")
        for key in required_identity:
            if not is_exact_int(raw.get(key)) or raw[key] < 0:
                fail("invalid {} for {}".format(key, path))
        if kind == "regular":
            if not is_exact_int(raw.get("size")) or raw["size"] < 0 or not _HEX64.fullmatch(str(raw.get("sha256", ""))):
                fail("invalid regular-file hash/size for {}".format(path))
            if "link_target" in raw:
                fail("regular entry has link target")
            if "elf" in raw:
                elf = raw["elf"]
                if not isinstance(elf, dict) or set(elf) != {"class", "endianness", "machine", "build_id"}:
                    fail("invalid ELF record for {}".format(path))
                if elf["class"] != 64 or elf["endianness"] != "little" or elf["machine"] != "AArch64":
                    fail("manifest contains non-AArch64 ELF metadata")
                if elf["build_id"] is not None and not re.fullmatch(r"[0-9a-f]+", str(elf["build_id"])):
                    fail("invalid ELF build id")
        elif kind == "directory":
            if set(raw) != {"path", "type", *required_identity}:
                fail("directory entry contains non-identity fields")
        elif kind == "symlink":
            if set(raw) != {"path", "type", "link_target", *required_identity}:
                fail("symlink entry contains invalid fields")
            target = raw["link_target"]
            if not isinstance(target, str) or not target or "\x00" in target or pathlib.PurePosixPath(target).is_absolute():
                fail("symlink target must be nonempty and relative")
        else:
            fail("unknown manifest file type for {}".format(path))
        entries.append(raw)
    provenance = manifest["provenance"]
    if not isinstance(provenance, dict) or not set(provenance).issubset(seen):
        fail("provenance keys must be payload paths")
    if not isinstance(manifest["content_address_algorithm"], str) or manifest["content_address_algorithm"] != "sha256-canonical-json-v1":
        fail("content-address algorithm differs")
    content = manifest["content_address"]
    if not isinstance(content, str) or not _HEX64.fullmatch(content):
        fail("invalid content address")
    computed = canonical_sha256(content_basis(manifest))
    if computed != content:
        fail("payload content address mismatch: manifest={} computed={}".format(content, computed))
    match = _CONTENT_ROOT.fullmatch(runtime_root.name)
    if not match or match.group(1) != content:
        fail("runtime directory name is not its manifest content address")
    return entries


def identity_tuple(info: os.stat_result) -> Tuple[int, int, int, int, int, int, int, int]:
    return (
        stat.S_IMODE(info.st_mode), info.st_uid, info.st_gid, info.st_dev, info.st_ino,
        os.major(info.st_dev), os.minor(info.st_dev), info.st_nlink,
    )


def manifest_identity(entry: Mapping[str, Any]) -> Tuple[int, int, int, int, int, int, int, int]:
    return (
        entry["mode"], entry["uid"], entry["gid"], entry["dev"], entry["ino"],
        entry["dev_major"], entry["dev_minor"], entry["nlink"],
    )


def verify_symlink_containment(path: pathlib.Path, runtime_root: pathlib.Path) -> None:
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        fail("broken runtime symlink {}: {}".format(path, exc))
    if not contained(resolved, runtime_root):
        fail("runtime symlink escapes package root: {} -> {}".format(path, resolved))


def required_dynamic_type_support(relative: str) -> List[str]:
    # rosidl's generic dispatcher loads these by dlopen; DT_NEEDED/ldd cannot
    # discover them. Both the fixed Fast DDS path and introspection are bound.
    match = re.fullmatch(r"lib/lib(.+)__rosidl_typesupport_(c|cpp)\.so", relative)
    if not match:
        return []
    package, language = match.groups()
    return ["lib{}__rosidl_typesupport_{}_{}.so".format(package, backend, language)
            for backend in ("fastrtps", "introspection")]


def verify_dynamic_type_support(lookup: Mapping[str, Mapping[str, Any]]) -> None:
    for relative in lookup:
        for soname in required_dynamic_type_support(relative):
            entry = lookup.get("lib/" + soname)
            if not entry or entry.get("type") != "regular" or "elf" not in entry:
                fail("missing manifested ROS dynamic type support: {} -> {}".format(relative, soname))


def verify_tree(runtime_root_value: str) -> Tuple[pathlib.Path, Dict[str, Any], Dict[str, Dict[str, Any]], List[pathlib.Path]]:
    supplied = pathlib.Path(runtime_root_value)
    try:
        before_root = supplied.lstat()
        runtime_root = supplied.resolve(strict=True)
    except OSError as exc:
        fail("cannot resolve runtime root: {}".format(exc))
    if not stat.S_ISDIR(before_root.st_mode) or supplied.is_symlink():
        fail("runtime root must be a real directory, not a symlink")
    if runtime_root != supplied.absolute():
        fail("runtime root path must already be canonical")
    manifest_path = runtime_root / MANIFEST_NAME
    manifest_stat = manifest_path.lstat()
    manifest = read_manifest(manifest_path)
    entries = validate_manifest_schema(manifest, runtime_root)
    if stat.S_IMODE(before_root.st_mode) != 0o700:
        fail("runtime root mode must be 0700")
    if before_root.st_uid != manifest["package_uid"] or before_root.st_gid != manifest["package_gid"]:
        fail("runtime root ownership differs from package identity")
    if manifest_stat.st_uid != manifest["package_uid"] or manifest_stat.st_gid != manifest["package_gid"]:
        fail("manifest ownership differs from package identity")
    lookup: Dict[str, Dict[str, Any]] = {entry["path"]: entry for entry in entries}
    actual = actual_payload_paths(runtime_root)
    if actual != set(lookup):
        fail("runtime tree differs from manifest: missing={} extra={}".format(
            sorted(set(lookup) - actual), sorted(actual - set(lookup))))

    elf_paths: List[pathlib.Path] = []
    for rel, entry in lookup.items():
        path = runtime_root / pathlib.PurePosixPath(rel)
        current = path.lstat()
        if identity_tuple(current) != manifest_identity(entry):
            fail("file identity differs for {}".format(rel))
        if entry["uid"] != manifest["package_uid"] or entry["gid"] != manifest["package_gid"]:
            fail("mixed owner in package: {}".format(rel))
        kind = entry["type"]
        if kind == "directory":
            if not stat.S_ISDIR(current.st_mode):
                fail("expected directory: {}".format(rel))
        elif kind == "symlink":
            if not stat.S_ISLNK(current.st_mode) or os.readlink(path) != entry["link_target"]:
                fail("symlink differs: {}".format(rel))
            verify_symlink_containment(path, runtime_root)
        else:
            if not stat.S_ISREG(current.st_mode) or current.st_nlink != 1:
                fail("expected single-link regular file: {}".format(rel))
            if current.st_size != entry["size"]:
                fail("size differs: {}".format(rel))
            digest = sha256_path_nofollow(path, current)
            if digest != entry["sha256"]:
                fail("sha256 differs: {}".format(rel))
            header = elf_header(path)
            if header is None:
                if "elf" in entry:
                    fail("manifest marks non-ELF as ELF: {}".format(rel))
            else:
                expected_elf = dict(header)
                expected_elf["build_id"] = read_build_id(path)
                if entry.get("elf") != expected_elf:
                    fail("ELF metadata differs: {}".format(rel))
                verify_dynamic_tags(path)
                elf_paths.append(path)

    for name, rel in ENTRYPOINTS.items():
        entry = lookup.get(rel)
        if not entry or entry["type"] != "regular" or "elf" not in entry:
            fail("entrypoint is not a manifested ELF regular file: {}".format(name))
        if not (runtime_root / rel).stat().st_mode & stat.S_IXUSR:
            fail("entrypoint is not executable: {}".format(name))
    for name, rel in RUNTIME_HELPERS.items():
        entry = lookup.get(rel)
        if not entry or entry["type"] != "regular" or "elf" in entry:
            fail("runtime helper is not a manifested non-ELF regular file: {}".format(name))
        if stat.S_IMODE((runtime_root / rel).stat().st_mode) != 0o444:
            fail("runtime helper must be immutable mode 0444: {}".format(name))
    for name, expected in AUXILIARY_INPUT_HASHES.items():
        rel = ENTRYPOINTS.get(name, RUNTIME_HELPERS.get(name))
        if rel is None:
            fail("auxiliary input has no runtime destination: {}".format(name))
        entry = lookup.get(rel)
        if not entry or entry["type"] != "regular" or entry.get("sha256") != expected:
            fail("reviewed auxiliary input bytes differ in package: {}".format(name))
    for component, paths in manifest["required_components"].items():
        for rel in paths:
            entry = lookup.get(rel)
            if not entry or entry["type"] != "regular" or "elf" not in entry:
                fail("component {} is not backed by a manifested ELF: {}".format(component, rel))
    for package in RESOURCE_PACKAGES:
        marker = "share/ament_index/resource_index/packages/{}".format(package)
        package_xml = "share/{}/package.xml".format(package)
        if marker not in lookup or package_xml not in lookup:
            fail("missing ament resource marker or package.xml for {}".format(package))

    verify_dynamic_type_support(lookup)
    for path in elf_paths:
        for soname, dependency in ldd_dependencies(path, runtime_root):
            try:
                resolved = dependency.resolve(strict=True)
            except OSError as exc:
                fail("dependency cannot be resolved for {}: {} ({})".format(path, dependency, exc))
            if contained(resolved, runtime_root):
                rel = resolved.relative_to(runtime_root).as_posix()
                dep_entry = lookup.get(rel)
                if not dep_entry or dep_entry["type"] != "regular" or "elf" not in dep_entry:
                    fail("runtime dependency is not a manifested ELF: {} -> {}".format(path, resolved))
                dep_stat = resolved.lstat()
                if identity_tuple(dep_stat) != manifest_identity(dep_entry):
                    fail("runtime dependency identity differs: {}".format(resolved))
            else:
                if is_rtabmap_object(resolved):
                    fail("external RTAB-Map dependency is forbidden: {} -> {}".format(path, resolved))
                if not is_external_path_allowed(resolved):
                    fail("dependency escaped runtime/ROS/system allowlist: {} -> {}".format(path, resolved))
                if str(resolved).startswith("/tmp/") or str(resolved).startswith("/home/"):
                    fail("dependency resolved to external private tree: {} -> {}".format(path, resolved))
            if soname.lower().startswith("librtabmap") and not contained(resolved, runtime_root):
                fail("RTAB-Map SONAME resolved outside runtime: {} -> {}".format(soname, resolved))

    print("RUNTIME_TREE_OK root={} content={} files={} elf={}".format(
        runtime_root, manifest["content_address"], len(entries), len(elf_paths)))
    return runtime_root, manifest, lookup, elf_paths


def proc_start_ticks(pid: int) -> int:
    raw = pathlib.Path("/proc/{}/stat".format(pid)).read_text(encoding="ascii", errors="strict")
    close = raw.rfind(")")
    if close < 0:
        fail("malformed /proc/{}/stat".format(pid))
    fields = raw[close + 2:].split()
    if len(fields) < 20:
        fail("short /proc/{}/stat".format(pid))
    try:
        return int(fields[19])
    except ValueError:
        fail("invalid start ticks in /proc/{}/stat".format(pid))
    return -1


def decode_proc_path(value: str) -> str:
    return _OCTAL_ESCAPE.sub(lambda match: chr(int(match.group(1), 8)), value)


def parse_maps_lines(lines: Iterable[str]) -> List[Tuple[str, int, int, int, str]]:
    records: List[Tuple[str, int, int, int, str]] = []
    for number, raw in enumerate(lines, 1):
        if number > MAX_MAPS_RECORDS:
            fail("proc-maps input exceeds record limit")
        parts = raw.rstrip("\n").split(None, 5)
        if len(parts) < 5:
            fail("malformed proc-maps line {}".format(number))
        if len(parts) == 5:
            continue
        address, perms, offset, device, inode_text, path_text = parts
        if "-" not in address or not re.fullmatch(r"[r-][w-][x-][ps]", perms) or not re.fullmatch(r"[0-9a-fA-F]+", offset):
            fail("malformed proc-maps fields on line {}".format(number))
        if not re.fullmatch(r"[0-9a-fA-F]+:[0-9a-fA-F]+", device) or not inode_text.isdigit():
            fail("malformed proc-maps identity on line {}".format(number))
        path_value = decode_proc_path(path_text)
        if path_value.startswith("["):
            continue
        major_text, minor_text = device.split(":", 1)
        records.append((path_value, int(major_text, 16), int(minor_text, 16), int(inode_text), perms))
    return records


def verify_maps_records(
    records: Sequence[Tuple[str, int, int, int, str]],
    runtime_root: pathlib.Path,
    manifest: Mapping[str, Any],
    lookup: Mapping[str, Mapping[str, Any]],
    expected_exe: Optional[str],
) -> None:
    mapped_rtab: Dict[str, Set[Tuple[str, int, int, int]]] = {}
    runtime_mapped: Set[str] = set()
    for path_text, dev_major, dev_minor, inode, _perms in records:
        if path_text.endswith(" (deleted)"):
            original = pathlib.Path(path_text[:-10])
            if is_rtabmap_object(original) or str(original).startswith(str(runtime_root) + "/"):
                fail("deleted runtime/RTAB mapping is forbidden: {}".format(path_text))
            continue
        if not path_text.startswith("/"):
            continue
        path = pathlib.Path(path_text)
        try:
            resolved = path.resolve(strict=True)
        except OSError as exc:
            if is_rtabmap_object(path):
                fail("mapped RTAB object cannot be resolved: {} ({})".format(path, exc))
            continue
        if contained(resolved, runtime_root):
            rel = resolved.relative_to(runtime_root).as_posix()
            entry = lookup.get(rel)
            if not entry or entry["type"] != "regular" or "elf" not in entry:
                fail("unmanifested runtime mapping: {}".format(resolved))
            current = resolved.lstat()
            if (dev_major, dev_minor, inode) != (entry["dev_major"], entry["dev_minor"], entry["ino"]):
                fail("proc-maps identity differs from manifest for {}".format(resolved))
            if (os.major(current.st_dev), os.minor(current.st_dev), current.st_ino) != (dev_major, dev_minor, inode):
                fail("proc-maps identity differs from live file for {}".format(resolved))
            runtime_mapped.add(rel)
        elif is_rtabmap_object(resolved):
            fail("external RTAB mapping is forbidden: {}".format(resolved))
        elif str(resolved).startswith("/tmp/"):
            fail("process maps an object from another /tmp tree: {}".format(resolved))
        elif str(resolved).startswith("/home/exocean/opt/rtabmap/"):
            fail("process maps an unpackaged vendor object: {}".format(resolved))
        if is_rtabmap_object(resolved):
            mapped_rtab.setdefault(rtabmap_mapping_key(resolved), set()).add(
                (str(resolved), dev_major, dev_minor, inode))
    for basename, identities in mapped_rtab.items():
        if len(identities) != 1:
            fail("dual RTAB mapping for {}: {}".format(basename, sorted(identities)))

    if expected_exe:
        entry_rel = ENTRYPOINTS[expected_exe]
        entry_real = (runtime_root / entry_rel).resolve(strict=True).relative_to(runtime_root).as_posix()
        if entry_real not in runtime_mapped:
            fail("expected entrypoint is absent from proc maps: {}".format(entry_rel))
        expected_runtime_dependencies: Set[str] = set()
        for soname, dependency in ldd_dependencies(runtime_root / entry_rel, runtime_root):
            resolved = dependency.resolve(strict=True)
            if contained(resolved, runtime_root) and soname.lower().startswith("librtabmap"):
                expected_runtime_dependencies.add(resolved.relative_to(runtime_root).as_posix())
        missing = expected_runtime_dependencies - runtime_mapped
        if missing:
            fail("live process is missing RTAB closure mappings: {}".format(sorted(missing)))
    if not mapped_rtab:
        fail("proc maps contain no RTAB runtime objects")
    print("RUNTIME_MAPS_OK rtab_objects={} runtime_mappings={}".format(
        len(mapped_rtab), len(runtime_mapped)))


def verify_proc_maps(
    runtime_root_value: str,
    pid: Optional[int],
    maps_file: Optional[str],
    expected_exe: Optional[str],
) -> None:
    runtime_root, manifest, lookup, _ = verify_tree(runtime_root_value)
    if (pid is None) == (maps_file is None):
        fail("proc-maps mode requires exactly one of --pid or --maps-file")
    if pid is not None:
        maps_path = pathlib.Path("/proc/{}/maps".format(pid))
    else:
        supplied = pathlib.Path(str(maps_file))
        supplied_info = supplied.lstat()
        if supplied.is_symlink() or not stat.S_ISREG(supplied_info.st_mode) or supplied_info.st_nlink != 1:
            fail("maps snapshot must be a single-link regular file")
        if supplied_info.st_size > MAX_MAPS_BYTES:
            fail("maps snapshot exceeds byte limit")
        maps_path = supplied.resolve(strict=True)
    with maps_path.open("r", encoding="utf-8", errors="strict") as stream:
        records = parse_maps_lines(stream)
    verify_maps_records(records, runtime_root, manifest, lookup, expected_exe)


def verify_process(runtime_root_value: str, pid: int, start_ticks: int, expected_exe: str) -> None:
    runtime_root, manifest, lookup, _ = verify_tree(runtime_root_value)
    if pid <= 1 or start_ticks <= 0:
        fail("PID and start ticks must be positive")
    proc_dir = pathlib.Path("/proc/{}".format(pid))
    if not proc_dir.is_dir():
        fail("process does not exist: {}".format(pid))
    observed_ticks = proc_start_ticks(pid)
    if observed_ticks != start_ticks:
        fail("process start ticks differ: expected={} observed={}".format(start_ticks, observed_ticks))
    status = (proc_dir / "status").read_text(encoding="ascii", errors="strict")
    uid_match = re.search(r"^Uid:\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$", status, re.MULTILINE)
    if not uid_match:
        fail("cannot parse process UID")
    uids = tuple(int(uid_match.group(index)) for index in range(1, 5))
    if any(value != manifest["package_uid"] for value in uids):
        fail("process UID is not the package UID: {}".format(uids))
    exe_link = os.readlink(proc_dir / "exe")
    if exe_link.endswith(" (deleted)"):
        fail("process executable mapping is deleted")
    observed_exe = pathlib.Path(exe_link).resolve(strict=True)
    expected_path = (runtime_root / ENTRYPOINTS[expected_exe]).resolve(strict=True)
    if observed_exe != expected_path:
        fail("process executable differs: expected={} observed={}".format(expected_path, observed_exe))
    rel = expected_path.relative_to(runtime_root).as_posix()
    entry = lookup.get(rel)
    observed_stat = expected_path.lstat()
    if not entry or identity_tuple(observed_stat) != manifest_identity(entry):
        fail("process executable inode is not the manifested inode")
    with (proc_dir / "maps").open("r", encoding="utf-8", errors="strict") as stream:
        records = parse_maps_lines(stream)
    verify_maps_records(records, runtime_root, manifest, lookup, expected_exe)
    if proc_start_ticks(pid) != start_ticks:
        fail("PID identity changed during process verification")
    print("RUNTIME_PROCESS_OK pid={} start_ticks={} exe={}".format(pid, start_ticks, expected_path))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    tree = subparsers.add_parser("tree", help="verify a packaged runtime tree")
    tree.add_argument("--runtime-root", required=True)

    process = subparsers.add_parser("process", help="verify a live process and its mappings")
    process.add_argument("--runtime-root", required=True)
    process.add_argument("--pid", required=True, type=int)
    process.add_argument("--start-ticks", required=True, type=int)
    process.add_argument("--expected-exe", required=True, choices=sorted(ENTRYPOINTS))

    maps = subparsers.add_parser("proc-maps", help="verify live or captured proc maps")
    maps.add_argument("--runtime-root", required=True)
    group = maps.add_mutually_exclusive_group(required=True)
    group.add_argument("--pid", type=int)
    group.add_argument("--maps-file")
    maps.add_argument("--expected-exe", choices=sorted(ENTRYPOINTS))
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.mode == "tree":
            verify_tree(args.runtime_root)
        elif args.mode == "process":
            verify_process(args.runtime_root, args.pid, args.start_ticks, args.expected_exe)
        else:
            verify_proc_maps(args.runtime_root, args.pid, args.maps_file, args.expected_exe)
    except (VerificationError, OSError, subprocess.SubprocessError) as exc:
        print("RUNTIME_VERIFY_FAIL {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
