#!/usr/bin/python3
"""Build a content-addressed private AArch64 RTAB-Map runtime package.

The packer accepts only the reviewed ARM transaction root and validates the
complete frozen source set before and after packaging.  It copies the reviewed
Core, GUI, ROS and safety entrypoints, helper programs, every same-build
RTAB-Map library from the private Core/ROS prefixes, the generated
message/runtime resources, and every non-system recursive ELF dependency into
one private tree.  It never reads a database, starts ROS, or writes a
production path.

The resulting directory is named ``sha256-<payload-address>``.  Its manifest
records hashes plus dev/inode/owner/mode identity for each payload object.  The
strict checker is copied into the package and run against the final renamed
tree before success is reported.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from collections import deque
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple


SCRIPT_DIR = pathlib.Path(__file__).resolve(strict=True).parent
sys.path.insert(0, str(SCRIPT_DIR))
import check_private_runtime as checker  # noqa: E402


DEFAULT_CORE_PREFIX_REL = "install_core_cap27_c046_e7e9_b916_v2"
DEFAULT_ROS_PREFIX_REL = "install_ros_cap27_c046_e7e9_b916_v3"
DEFAULT_AUX_INPUT_REL = "extracted/runtime_aux_inputs"
DEFAULT_OUTPUT_PARENT_REL = "runtime_packages_c046_e7e9_b916_v3"

VENDOR_PREFIXES = (
    pathlib.Path("/home/exocean/opt/rtabmap/opt/ros/humble"),
    pathlib.Path("/home/exocean/opt/rtabmap/usr"),
)

# The full-feature Core was intentionally built with ZED support and therefore
# has a direct DT_NEEDED on this SDK library.  Copy this one reviewed byte
# identity into the immutable runtime; do not broadly trust /usr/local/zed or
# leave the final process dependent on an unbound external SDK path.
REVIEWED_EXTERNAL_DEPENDENCIES = {
    pathlib.Path("/usr/local/zed/lib/libsl_zed.so"):
        "16ad45ce79ddc1692dd0c2dd66517db44f54e2a6f52b35f88aeaf565fdcffe14",
}

COMPONENT_PATTERNS: Dict[str, re.Pattern[str]] = {
    "core": re.compile(r"^librtabmap_core\.so(?:\.|$)"),
    "gui": re.compile(r"^librtabmap_gui\.so(?:\.|$)"),
    "utilite": re.compile(r"^librtabmap_utilite\.so(?:\.|$)"),
    "conversions_time_guard": re.compile(r"^librtabmap_conversions\.so(?:\.|$)"),
    "msgs": re.compile(r"^librtabmap_msgs(?:__[^/]*)?\.so(?:\.|$)"),
    "sync": re.compile(r"^librtabmap_sync\.so(?:\.|$)"),
    "util": re.compile(r"^librtabmap_util\.so(?:\.|$)"),
    "odom": re.compile(r"^librtabmap_odom\.so(?:\.|$)"),
    "odom_plugins": re.compile(r"^librtabmap_odom_plugins\.so(?:\.|$)"),
    "slam_plugins": re.compile(r"^librtabmap_slam_plugins\.so(?:\.|$)"),
}

ENTRYPOINT_SOURCES = {
    "rtabmap": ("ros", pathlib.PurePosixPath("lib/rtabmap_slam/rtabmap")),
    "rgbd_odometry": ("ros", pathlib.PurePosixPath("lib/rtabmap_odom/rgbd_odometry")),
    "point_cloud_xyzrgb": ("ros", pathlib.PurePosixPath("lib/rtabmap_util/point_cloud_xyzrgb")),
    "rtabmap_export": ("core", pathlib.PurePosixPath("bin/rtabmap-export")),
}

AUXILIARY_INPUTS = {
    "header_health_observer": ("bin/header_health_observer", "bin/header_health_observer"),
    "safe_reloc_gid_frontend": ("bin/safe_reloc_gid_frontend", "bin/safe_reloc_gid_frontend"),
    "safe_reloc_ipc_engine": ("libexec/safe_reloc_ipc_engine.py", "libexec/safe_reloc_ipc_engine.py"),
    "safe_reloc_engine_core": ("libexec/safe_reloc_engine_core.py", "libexec/safe_reloc_engine_core.py"),
    "write_process_identity": ("libexec/write_process_identity.py", "libexec/write_process_identity.py"),
}


class PackagingError(RuntimeError):
    """A source, dependency or package-layout invariant was violated."""


def fail(message: str) -> None:
    raise PackagingError(message)


def sha256_path(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def contained(path: pathlib.Path, parent: pathlib.Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def require_canonical_directory(value: pathlib.Path, label: str) -> pathlib.Path:
    try:
        before = value.lstat()
        resolved = value.resolve(strict=True)
    except OSError as exc:
        fail("{} is unavailable: {}".format(label, exc))
    if value.is_symlink() or not stat.S_ISDIR(before.st_mode):
        fail("{} must be a real directory".format(label))
    if resolved != value.absolute():
        fail("{} must be passed as a canonical path".format(label))
    return resolved


def validate_sources(private_root: pathlib.Path) -> None:
    source_root = private_root / "extracted/source"
    for relative, expected in checker.SOURCE_HASHES.items():
        path = source_root / pathlib.PurePosixPath(relative)
        try:
            info = path.lstat()
        except OSError as exc:
            fail("frozen source is missing: {} ({})".format(relative, exc))
        if path.is_symlink() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            fail("frozen source must be a single-link regular file: {}".format(relative))
        actual = sha256_path(path)
        if actual != expected:
            fail("frozen source hash mismatch: {} expected={} actual={}".format(relative, expected, actual))


def validate_auxiliary_inputs(aux_input: pathlib.Path) -> None:
    expected_names = set(checker.AUXILIARY_INPUT_HASHES)
    if expected_names != set(AUXILIARY_INPUTS):
        fail("auxiliary-input layout and frozen hash contract differ")
    for name, expected in checker.AUXILIARY_INPUT_HASHES.items():
        source_relative, _destination_relative = AUXILIARY_INPUTS[name]
        source = aux_input / pathlib.PurePosixPath(source_relative)
        try:
            info = source.lstat()
            resolved = source.resolve(strict=True)
        except OSError as exc:
            fail("reviewed auxiliary input is missing: {} ({})".format(name, exc))
        if source.is_symlink() or resolved != source.absolute():
            fail("reviewed auxiliary input must be passed without symlinks: {}".format(name))
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            fail("reviewed auxiliary input must be a single-link regular file: {}".format(name))
        if info.st_uid != os.getuid() or info.st_gid != os.getgid():
            fail("reviewed auxiliary input ownership differs from packager: {}".format(name))
        expected_mode = 0o555 if name in checker.ENTRYPOINTS else 0o444
        if stat.S_IMODE(info.st_mode) != expected_mode:
            fail("reviewed auxiliary input has wrong mode: {} expected={:04o} actual={:04o}".format(
                name, expected_mode, stat.S_IMODE(info.st_mode)))
        actual = sha256_path(resolved)
        if actual != expected:
            fail("reviewed auxiliary input hash mismatch: {} expected={} actual={}".format(
                name, expected, actual))


def normalized_file_mode(source_mode: int, executable: bool = False) -> int:
    if executable or source_mode & 0o111:
        return 0o555
    return 0o444


def safe_destination_relative(value: str) -> pathlib.PurePosixPath:
    pure = pathlib.PurePosixPath(value)
    if (not value or pure.is_absolute() or pure.as_posix() != value or
            any(part in ("", ".", "..") for part in pure.parts)):
        fail("unsafe runtime destination: {!r}".format(value))
    if value == checker.MANIFEST_NAME:
        fail("payload cannot overwrite its manifest")
    return pure


class RuntimeBuilder:
    def __init__(
        self,
        private_root: pathlib.Path,
        core_prefix: pathlib.Path,
        ros_prefix: pathlib.Path,
        aux_input: pathlib.Path,
        staging: pathlib.Path,
    ) -> None:
        self.private_root = private_root
        self.core_prefix = core_prefix
        self.ros_prefix = ros_prefix
        self.aux_input = aux_input
        self.staging = staging
        self.provenance: Dict[str, Dict[str, Any]] = {}
        self.regular_hashes: Dict[str, str] = {}
        self._ensure_directory("")

    def _destination(self, relative: str) -> pathlib.Path:
        pure = safe_destination_relative(relative) if relative else pathlib.PurePosixPath(".")
        destination = self.staging / pure
        parent = destination.parent.resolve(strict=True) if destination != self.staging else self.staging
        if destination != self.staging and not contained(parent, self.staging):
            fail("destination parent escaped staging tree")
        return destination

    def _ensure_directory(self, relative: str) -> pathlib.Path:
        if not relative:
            self.staging.mkdir(mode=0o700, parents=False, exist_ok=False)
            return self.staging
        pure = safe_destination_relative(relative)
        current = self.staging
        for part in pure.parts:
            current = current / part
            try:
                info = current.lstat()
            except FileNotFoundError:
                current.mkdir(mode=0o755)
                continue
            if current.is_symlink() or not stat.S_ISDIR(info.st_mode):
                fail("directory collision in staging tree: {}".format(current))
        return current

    def copy_regular(
        self,
        source: pathlib.Path,
        relative: str,
        *,
        executable: bool = False,
        provenance_kind: str,
    ) -> bool:
        pure = safe_destination_relative(relative)
        try:
            source_real = source.resolve(strict=True)
            info = source_real.lstat()
        except OSError as exc:
            fail("cannot resolve packaging input {}: {}".format(source, exc))
        if not stat.S_ISREG(info.st_mode):
            fail("packaging input is not regular: {}".format(source_real))
        digest = sha256_path(source_real)
        destination = self.staging / pure
        self._ensure_directory(pure.parent.as_posix()) if pure.parent.as_posix() != "." else None
        if destination.exists() or destination.is_symlink():
            if destination.is_symlink() or not destination.is_file():
                fail("runtime destination type collision: {}".format(relative))
            existing = sha256_path(destination)
            if existing != digest:
                fail("runtime basename collision with different bytes: {}".format(relative))
            return False
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        fd = os.open(str(destination), flags, 0o600)
        try:
            with source_real.open("rb") as src, os.fdopen(fd, "wb", closefd=False) as dst:
                shutil.copyfileobj(src, dst, length=1024 * 1024)
                dst.flush()
                os.fsync(dst.fileno())
        finally:
            os.close(fd)
        os.chmod(destination, normalized_file_mode(info.st_mode, executable))
        copied = destination.lstat()
        if copied.st_nlink != 1 or sha256_path(destination) != digest:
            fail("copied payload failed identity/hash verification: {}".format(relative))
        self.regular_hashes[relative] = digest
        self.provenance[relative] = {
            "kind": provenance_kind,
            "source_path": str(source_real),
            "source_sha256": digest,
            "source_dev": info.st_dev,
            "source_ino": info.st_ino,
            "source_uid": info.st_uid,
            "source_gid": info.st_gid,
            "source_mode": stat.S_IMODE(info.st_mode),
        }
        return True

    def add_symlink(self, relative: str, target: str, *, provenance_kind: str) -> None:
        pure = safe_destination_relative(relative)
        target_pure = pathlib.PurePosixPath(target)
        if not target or target_pure.is_absolute() or "\x00" in target:
            fail("runtime symlink target must be relative")
        self._ensure_directory(pure.parent.as_posix()) if pure.parent.as_posix() != "." else None
        destination = self.staging / pure
        if destination.exists() or destination.is_symlink():
            if destination.is_symlink() and os.readlink(destination) == target:
                return
            fail("runtime symlink collision: {}".format(relative))
        os.symlink(target, destination)
        resolved = destination.resolve(strict=True)
        if not contained(resolved, self.staging):
            fail("generated runtime symlink escapes staging: {}".format(relative))
        self.provenance[relative] = {"kind": provenance_kind, "generated_target": target}

    def copy_library_reference(
        self,
        source: pathlib.Path,
        *,
        requested_name: Optional[str] = None,
        provenance_kind: str,
    ) -> pathlib.Path:
        try:
            resolved = source.resolve(strict=True)
        except OSError as exc:
            fail("cannot resolve library {}: {}".format(source, exc))
        if not resolved.is_file():
            fail("library target is not regular: {}".format(resolved))
        actual_relative = "lib/{}".format(resolved.name)
        self.copy_regular(resolved, actual_relative, provenance_kind=provenance_kind)
        aliases: Set[str] = set()
        aliases.add(requested_name or source.name)
        if source.name != resolved.name:
            aliases.add(source.name)
        for alias in sorted(aliases):
            if alias and alias != resolved.name:
                if "/" in alias or alias in (".", ".."):
                    fail("invalid library SONAME alias: {}".format(alias))
                self.add_symlink(
                    "lib/{}".format(alias),
                    resolved.name,
                    provenance_kind="{}-alias".format(provenance_kind),
                )
        return self.staging / actual_relative

    def copy_tree(self, source_root: pathlib.Path, destination_relative: str, provenance_kind: str) -> None:
        source_root = source_root.resolve(strict=True)
        if not source_root.is_dir():
            fail("resource source is not a directory: {}".format(source_root))
        self._ensure_directory(destination_relative)
        stack: List[Tuple[pathlib.Path, pathlib.PurePosixPath]] = [
            (source_root, pathlib.PurePosixPath(destination_relative))
        ]
        visited_directories: Set[Tuple[int, int]] = set()
        while stack:
            source_dir, destination_dir = stack.pop()
            directory_real = source_dir.resolve(strict=True)
            directory_info = directory_real.stat()
            identity = (directory_info.st_dev, directory_info.st_ino)
            if identity in visited_directories:
                fail("resource directory cycle or duplicate reached: {}".format(source_dir))
            visited_directories.add(identity)
            self._ensure_directory(destination_dir.as_posix())
            for child in sorted(source_dir.iterdir(), key=lambda item: item.name):
                child_real = child.resolve(strict=True)
                child_info = child_real.stat()
                child_dest = destination_dir / child.name
                if stat.S_ISDIR(child_info.st_mode):
                    self._ensure_directory(child_dest.as_posix())
                    stack.append((child_real, child_dest))
                elif stat.S_ISREG(child_info.st_mode):
                    self.copy_regular(
                        child_real,
                        child_dest.as_posix(),
                        provenance_kind=provenance_kind,
                    )
                else:
                    fail("special resource input is forbidden: {}".format(child))

    def copied_elf_paths(self) -> List[pathlib.Path]:
        paths: List[pathlib.Path] = []
        for relative in sorted(self.regular_hashes):
            path = self.staging / pathlib.PurePosixPath(relative)
            header = checker.elf_header(path)
            if header is not None:
                checker.verify_dynamic_tags(path)
                paths.append(path)
        return paths


def source_is_packagable_dependency(
    path: pathlib.Path,
    private_root: pathlib.Path,
    core_prefix: pathlib.Path,
    ros_prefix: pathlib.Path,
) -> bool:
    prefixes = (private_root, core_prefix, ros_prefix, *VENDOR_PREFIXES)
    if any(contained(path, prefix.resolve(strict=True)) for prefix in prefixes if prefix.exists()):
        return True
    resolved = path.resolve(strict=True)
    for reviewed, expected_hash in REVIEWED_EXTERNAL_DEPENDENCIES.items():
        if resolved == reviewed.resolve(strict=True):
            if not resolved.is_file() or resolved.is_symlink():
                fail("reviewed external dependency is not a regular non-symlink file: {}".format(resolved))
            actual_hash = sha256_path(resolved)
            if actual_hash != expected_hash:
                fail(
                    "reviewed external dependency hash mismatch: {} expected={} actual={}".format(
                        resolved, expected_hash, actual_hash
                    )
                )
            return True
    return False


def build_resolution_environment(
    builder: RuntimeBuilder,
    core_prefix: pathlib.Path,
    ros_prefix: pathlib.Path,
) -> Dict[str, str]:
    library_dirs = [
        builder.staging / "lib",
        ros_prefix / "lib",
        core_prefix / "lib",
        VENDOR_PREFIXES[0] / "lib",
        VENDOR_PREFIXES[0] / "lib/aarch64-linux-gnu",
        VENDOR_PREFIXES[1] / "lib",
        VENDOR_PREFIXES[1] / "lib/aarch64-linux-gnu",
        pathlib.Path("/usr/local/zed/lib"),
        pathlib.Path("/opt/ros/humble/lib"),
        pathlib.Path("/opt/ros/humble/lib/aarch64-linux-gnu"),
    ]
    return {
        "PATH": "/opt/ros/humble/bin:/usr/bin:/bin",
        "LD_LIBRARY_PATH": ":".join(str(path) for path in library_dirs if path.is_dir()),
        "LANG": "C",
        "LC_ALL": "C",
    }


def build_ldd_dependencies(path: pathlib.Path, environment: Mapping[str, str]) -> List[Tuple[str, pathlib.Path]]:
    result = subprocess.run(
        [checker.LDD, "-r", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        errors="replace",
        env=dict(environment),
        check=False,
    )
    if result.returncode != 0:
        fail("build-time ldd -r failed for {} (rc={}): {}".format(path, result.returncode, result.stdout.strip()))
    try:
        return checker.parse_ldd_output(result.stdout, path)
    except checker.VerificationError as exc:
        fail(str(exc))
    return []


def collect_dependency_closure(builder: RuntimeBuilder) -> None:
    queued: Set[str] = set()
    queue: deque[pathlib.Path] = deque()
    for path in builder.copied_elf_paths():
        relative = path.relative_to(builder.staging).as_posix()
        queued.add(relative)
        queue.append(path)
    environment = build_resolution_environment(builder, builder.core_prefix, builder.ros_prefix)
    while queue:
        subject = queue.popleft()
        subject_relative = subject.resolve(strict=True).relative_to(builder.staging).as_posix()
        for soname in checker.required_dynamic_type_support(subject_relative):
            if "lib/" + soname in builder.regular_hashes:
                continue
            source_value = builder.provenance[subject_relative].get(
                "dynamic_dependency_origin", builder.provenance[subject_relative].get("source_path"))
            if not isinstance(source_value, str):
                fail("ROS type-support dispatcher lacks source provenance: {}".format(subject_relative))
            if sha256_path(pathlib.Path(source_value)) != builder.regular_hashes[subject_relative]:
                fail("ROS dynamic dispatcher origin changed: {}".format(source_value))
            dependency = pathlib.Path(source_value).parent / soname
            if not dependency.is_file() or not source_is_packagable_dependency(
                    dependency.resolve(strict=True), builder.private_root, builder.core_prefix, builder.ros_prefix):
                fail("reviewed ROS dynamic type support is unavailable: {}".format(dependency))
            copied = builder.copy_library_reference(
                dependency, requested_name=soname, provenance_kind="rosidl-dynamic-type-support")
            copied_relative = copied.relative_to(builder.staging).as_posix()
            if copied_relative not in queued:
                checker.elf_header(copied)
                checker.verify_dynamic_tags(copied)
                queued.add(copied_relative)
                queue.append(copied)
        for soname, dependency in build_ldd_dependencies(subject, environment):
            try:
                resolved = dependency.resolve(strict=True)
            except OSError as exc:
                fail("unresolved dependency for {}: {} ({})".format(subject_relative, dependency, exc))
            if contained(resolved, builder.staging):
                continue
            if checker.is_rtabmap_object(resolved):
                if not (contained(resolved, builder.core_prefix) or contained(resolved, builder.ros_prefix)):
                    fail("external RTAB dependency entered build closure: {} -> {}".format(subject, resolved))
            if checker.is_external_path_allowed(resolved) and not checker.is_rtabmap_object(resolved):
                continue
            if not source_is_packagable_dependency(
                    resolved, builder.private_root, builder.core_prefix, builder.ros_prefix):
                fail("dependency is neither system/ROS nor reviewed private input: {} -> {}".format(subject, resolved))
            copied = builder.copy_library_reference(
                resolved,
                requested_name=soname,
                provenance_kind="recursive-private-dependency",
            )
            copied_relative = copied.relative_to(builder.staging).as_posix()
            environment = build_resolution_environment(builder, builder.core_prefix, builder.ros_prefix)
            if copied_relative not in queued:
                checker.elf_header(copied)
                checker.verify_dynamic_tags(copied)
                queued.add(copied_relative)
                queue.append(copied)

    # A second strict pass deliberately removes all build/vendor paths.  This
    # proves the recursive closure no longer depends on them before freezing.
    for path in builder.copied_elf_paths():
        for soname, dependency in checker.ldd_dependencies(path, builder.staging):
            resolved = dependency.resolve(strict=True)
            if contained(resolved, builder.staging):
                continue
            if checker.is_rtabmap_object(resolved):
                fail("strict closure still resolves RTAB externally: {} -> {}".format(path, resolved))
            if not checker.is_external_path_allowed(resolved):
                fail("strict closure escaped runtime/ROS/system: {} -> {}".format(path, resolved))
            if soname.lower().startswith("librtabmap"):
                fail("strict closure resolved RTAB SONAME externally: {} -> {}".format(soname, resolved))


def library_component_manifest(builder: RuntimeBuilder) -> Dict[str, List[str]]:
    regular_libraries: List[str] = []
    for relative in sorted(builder.regular_hashes):
        pure = pathlib.PurePosixPath(relative)
        if pure.parent == pathlib.PurePosixPath("lib") and pure.name.startswith("librtabmap"):
            regular_libraries.append(relative)
    components: Dict[str, List[str]] = {}
    for component, pattern in COMPONENT_PATTERNS.items():
        matched = [relative for relative in regular_libraries if pattern.match(pathlib.PurePosixPath(relative).name)]
        if not matched:
            fail("required same-build component is absent: {}".format(component))
        components[component] = matched
    if set(components) != checker.REQUIRED_COMPONENT_KEYS:
        fail("component manifest and checker contract differ")
    # Components are required to come from the matching private build prefix,
    # not from an ldd resolution fallback.
    for component, paths in components.items():
        expected_prefix = builder.core_prefix if component in {"core", "gui", "utilite"} else builder.ros_prefix
        for relative in paths:
            provenance = builder.provenance.get(relative, {})
            source_value = provenance.get("source_path")
            if not isinstance(source_value, str):
                fail("component lacks source provenance: {}".format(relative))
            if not contained(pathlib.Path(source_value).resolve(strict=True), expected_prefix):
                fail("component did not come from its same-build prefix: {} ({})".format(relative, source_value))
    return components


def copy_private_inputs(builder: RuntimeBuilder) -> None:
    # Entry points are normalized to private bin/ while ROS package paths are
    # relative symlinks, so ros2 run and direct execution resolve one inode.
    for name, (prefix_kind, source_relative) in ENTRYPOINT_SOURCES.items():
        source_prefix = builder.ros_prefix if prefix_kind == "ros" else builder.core_prefix
        source = source_prefix / source_relative
        if not contained(source.resolve(strict=True), source_prefix):
            fail("entrypoint escaped same-build {} prefix: {}".format(prefix_kind, source))
        builder.copy_regular(
            source,
            checker.ENTRYPOINTS[name],
            executable=True,
            provenance_kind="same-build-{}-entrypoint".format(prefix_kind),
        )
    builder.add_symlink(
        "lib/rtabmap_slam/rtabmap", "../../bin/rtabmap",
        provenance_kind="ros2-run-entrypoint-alias",
    )
    builder.add_symlink(
        "lib/rtabmap_odom/rgbd_odometry", "../../bin/rgbd_odometry",
        provenance_kind="ros2-run-entrypoint-alias",
    )
    builder.add_symlink(
        "lib/rtabmap_util/point_cloud_xyzrgb", "../../bin/point_cloud_xyzrgb",
        provenance_kind="ros2-run-entrypoint-alias",
    )

    for name, (source_relative, destination_relative) in AUXILIARY_INPUTS.items():
        source = builder.aux_input / pathlib.PurePosixPath(source_relative)
        if not contained(source.resolve(strict=True), builder.aux_input):
            fail("auxiliary input escaped reviewed directory: {}".format(source))
        expected_destination = checker.ENTRYPOINTS.get(name, checker.RUNTIME_HELPERS.get(name))
        if expected_destination != destination_relative:
            fail("auxiliary input and checker layout differ: {}".format(name))
        builder.copy_regular(
            source,
            destination_relative,
            executable=name in checker.ENTRYPOINTS,
            provenance_kind="reviewed-arm-auxiliary-input",
        )

    # Copy every RTAB library emitted by the two private install prefixes.  The
    # component check below still requires the exact operational subset.
    discovered: List[Tuple[pathlib.Path, str]] = []
    for prefix, kind in ((builder.core_prefix, "same-build-core-library"),
                         (builder.ros_prefix, "same-build-ros-library")):
        for path in sorted((prefix / "lib").rglob("librtabmap*.so*")):
            if path.is_file() or path.is_symlink():
                if not contained(path.resolve(strict=True), prefix):
                    fail("RTAB library escaped same-build prefix: {}".format(path))
                discovered.append((path, kind))
    if not discovered:
        fail("no same-build RTAB libraries were found")
    for path, kind in discovered:
        builder.copy_library_reference(path, provenance_kind=kind)

    # Ament/plugin metadata and generated rtabmap_msgs Python/typesupport data.
    builder.copy_tree(builder.ros_prefix / "share", "share", "same-build-ros-resource")
    for relative_parent in ("lib", "local/lib"):
        parent = builder.ros_prefix / relative_parent
        if not parent.is_dir():
            continue
        for python_dir in sorted(parent.glob("python*")):
            if not python_dir.is_dir():
                continue
            for leaf in ("site-packages", "dist-packages"):
                source = python_dir / leaf
                if source.is_dir():
                    destination = pathlib.PurePosixPath(relative_parent) / python_dir.name / leaf
                    builder.copy_tree(source, destination.as_posix(), "same-build-ros-python")

    builder.copy_regular(
        pathlib.Path(__file__).resolve(strict=True),
        "tools/package_private_runtime.py",
        executable=True,
        provenance_kind="reviewed-packager",
    )
    builder.copy_regular(
        SCRIPT_DIR / "check_private_runtime.py",
        "tools/check_private_runtime.py",
        executable=True,
        provenance_kind="reviewed-checker",
    )


def write_runtime_environment(builder: RuntimeBuilder) -> None:
    python_paths: List[str] = []
    for pattern in ("lib/python*/site-packages", "lib/python*/dist-packages",
                    "local/lib/python*/site-packages", "local/lib/python*/dist-packages"):
        for path in sorted(builder.staging.glob(pattern)):
            if path.is_dir():
                python_paths.append('"$runtime_root/{}"'.format(path.relative_to(builder.staging).as_posix()))
    python_value = ":".join(python_paths + [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
    ])
    body = """#!/usr/bin/env bash
set -euo pipefail
runtime_root="$(CDPATH= cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")/.." && pwd -P)"
unset LD_PRELOAD LD_AUDIT PYTHONHOME
export PATH="$runtime_root/bin:/opt/ros/humble/bin:/usr/bin:/bin"
export LD_LIBRARY_PATH="$runtime_root/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu"
export AMENT_PREFIX_PATH="$runtime_root:/opt/ros/humble"
export COLCON_PREFIX_PATH="$runtime_root:/opt/ros/humble"
export PYTHONPATH={python_value}
exec "$@"
""".format(python_value=python_value or '""')
    temporary = builder.staging / ".runtime-env.sh.tmp"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    fd = os.open(str(temporary), flags, 0o600)
    try:
        with os.fdopen(fd, "wb", closefd=False) as stream:
            stream.write(body.encode("utf-8"))
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(fd)
    digest = sha256_path(temporary)
    destination = builder.staging / "tools/runtime-env.sh"
    os.rename(temporary, destination)
    os.chmod(destination, 0o555)
    builder.regular_hashes["tools/runtime-env.sh"] = digest
    builder.provenance["tools/runtime-env.sh"] = {"kind": "generated-strict-runtime-environment"}

def normalize_payload_modes(staging: pathlib.Path) -> None:
    # File modes were normalized at creation.  Freeze directories only after
    # the final generated files have been added.
    directories: List[pathlib.Path] = []
    for root, names, _files in os.walk(staging, topdown=False, followlinks=False):
        for name in names:
            path = pathlib.Path(root) / name
            if path.is_symlink():
                continue
            directories.append(path)
    for path in directories:
        os.chmod(path, 0o555)
    os.chmod(staging, 0o700)


def manifest_entry(path: pathlib.Path, staging: pathlib.Path) -> Dict[str, Any]:
    relative = path.relative_to(staging).as_posix()
    info = path.lstat()
    base: Dict[str, Any] = {
        "path": relative,
        "mode": stat.S_IMODE(info.st_mode),
        "uid": info.st_uid,
        "gid": info.st_gid,
        "dev": info.st_dev,
        "ino": info.st_ino,
        "dev_major": os.major(info.st_dev),
        "dev_minor": os.minor(info.st_dev),
        "nlink": info.st_nlink,
    }
    if stat.S_ISDIR(info.st_mode):
        base["type"] = "directory"
    elif stat.S_ISLNK(info.st_mode):
        base["type"] = "symlink"
        base["link_target"] = os.readlink(path)
    elif stat.S_ISREG(info.st_mode):
        if info.st_nlink != 1:
            fail("payload regular file is hardlinked: {}".format(relative))
        base["type"] = "regular"
        base["size"] = info.st_size
        base["sha256"] = sha256_path(path)
        header = checker.elf_header(path)
        if header is not None:
            checker.verify_dynamic_tags(path)
            header["build_id"] = checker.read_build_id(path)
            base["elf"] = header
    else:
        fail("special payload object is forbidden: {}".format(relative))
    return base


def scan_manifest_entries(staging: pathlib.Path) -> List[Dict[str, Any]]:
    paths: List[pathlib.Path] = []
    for root, directories, files in os.walk(staging, topdown=True, followlinks=False):
        directories.sort()
        files.sort()
        root_path = pathlib.Path(root)
        for name in directories:
            paths.append(root_path / name)
        for name in files:
            path = root_path / name
            if path.name == checker.MANIFEST_NAME and path.parent == staging:
                continue
            paths.append(path)
    entries = [manifest_entry(path, staging) for path in paths]
    entries.sort(key=lambda entry: entry["path"])
    if len(entries) > checker.MAX_TREE_ENTRIES:
        fail("payload exceeds checker entry limit")
    return entries


def write_manifest(
    builder: RuntimeBuilder,
    components: Mapping[str, List[str]],
) -> Tuple[str, pathlib.Path]:
    normalize_payload_modes(builder.staging)
    entries = scan_manifest_entries(builder.staging)
    package_uid = os.getuid()
    package_gid = os.getgid()
    for entry in entries:
        if entry["uid"] != package_uid or entry["gid"] != package_gid:
            fail("mixed payload ownership before manifest: {}".format(entry["path"]))
    manifest: Dict[str, Any] = {
        "schema_version": checker.SCHEMA_VERSION,
        "package_kind": checker.PACKAGE_KIND,
        "architecture": "AArch64",
        "content_address": "",
        "content_address_algorithm": "sha256-canonical-json-v1",
        "manifest_excluded_from_payload": checker.MANIFEST_NAME,
        "build_private_root": checker.EXPECTED_PRIVATE_ROOT,
        "package_uid": package_uid,
        "package_gid": package_gid,
        "source_hashes": checker.SOURCE_HASHES,
        "auxiliary_input_hashes": checker.AUXILIARY_INPUT_HASHES,
        "entrypoints": checker.ENTRYPOINTS,
        "runtime_helpers": checker.RUNTIME_HELPERS,
        "required_components": dict(components),
        "resource_packages": sorted(checker.RESOURCE_PACKAGES),
        "files": entries,
        "provenance": {key: builder.provenance[key] for key in sorted(builder.provenance)},
    }
    content = checker.canonical_sha256(checker.content_basis(manifest))
    manifest["content_address"] = content
    encoded = (json.dumps(manifest, ensure_ascii=True, indent=2, sort_keys=True) + "\n").encode("ascii")
    temporary = builder.staging / ".runtime-manifest.json.tmp"
    fd = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "wb", closefd=False) as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(fd)
    manifest_path = builder.staging / checker.MANIFEST_NAME
    os.rename(temporary, manifest_path)
    os.chmod(manifest_path, 0o444)
    directory_fd = os.open(str(builder.staging), os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    return content, manifest_path


def resolve_child_argument(private_root: pathlib.Path, value: Optional[str], default_relative: str, label: str) -> pathlib.Path:
    path = pathlib.Path(value) if value else private_root / default_relative
    resolved = require_canonical_directory(path, label)
    if not contained(resolved, private_root):
        fail("{} must remain below reviewed private root".format(label))
    return resolved


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--private-root", required=True)
    parser.add_argument("--core-prefix", help="default: PRIVATE_ROOT/{}".format(DEFAULT_CORE_PREFIX_REL))
    parser.add_argument("--ros-prefix", help="default: PRIVATE_ROOT/{}".format(DEFAULT_ROS_PREFIX_REL))
    parser.add_argument("--aux-input", help="default: PRIVATE_ROOT/{}".format(DEFAULT_AUX_INPUT_REL))
    parser.add_argument("--output-parent", help="default: PRIVATE_ROOT/{}".format(DEFAULT_OUTPUT_PARENT_REL))
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    staging: Optional[pathlib.Path] = None
    try:
        private_root = require_canonical_directory(pathlib.Path(args.private_root), "private root")
        if str(private_root) != checker.EXPECTED_PRIVATE_ROOT:
            fail("refusing unexpected private root: {}".format(private_root))
        core_prefix = resolve_child_argument(private_root, args.core_prefix, DEFAULT_CORE_PREFIX_REL, "Core prefix")
        ros_prefix = resolve_child_argument(private_root, args.ros_prefix, DEFAULT_ROS_PREFIX_REL, "ROS prefix")
        aux_input = resolve_child_argument(private_root, args.aux_input, DEFAULT_AUX_INPUT_REL, "auxiliary input")
        validate_sources(private_root)
        validate_auxiliary_inputs(aux_input)
        output_parent_value = pathlib.Path(args.output_parent) if args.output_parent else private_root / DEFAULT_OUTPUT_PARENT_REL
        if output_parent_value.exists():
            output_parent = require_canonical_directory(output_parent_value, "output parent")
        else:
            output_parent_value.mkdir(mode=0o700, parents=False)
            output_parent = output_parent_value.resolve(strict=True)
        if not contained(output_parent, private_root) or output_parent == private_root:
            fail("output parent must be a dedicated directory below private root")
        if output_parent.is_symlink():
            fail("output parent cannot be a symlink")
        staging = pathlib.Path(tempfile.mkdtemp(prefix=".runtime-stage-", dir=str(output_parent)))
        os.chmod(staging, 0o700)
        # RuntimeBuilder creates the staging directory itself.  Replace the
        # mkdtemp reservation with a never-before-used sibling name without a
        # recursive delete.
        reserved = staging
        staging = output_parent / (reserved.name + "-payload")
        reserved.rmdir()
        builder = RuntimeBuilder(private_root, core_prefix, ros_prefix, aux_input, staging)
        copy_private_inputs(builder)
        write_runtime_environment(builder)
        collect_dependency_closure(builder)
        components = library_component_manifest(builder)
        validate_sources(private_root)
        validate_auxiliary_inputs(aux_input)
        content, _manifest_staging = write_manifest(builder, components)
        final_root = output_parent / ("sha256-" + content)
        if final_root.exists() or final_root.is_symlink():
            fail("content-addressed runtime already exists; refusing overwrite: {}".format(final_root))
        os.rename(staging, final_root)
        staging = None
        parent_fd = os.open(str(output_parent), os.O_RDONLY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
        checker_path = final_root / "tools/check_private_runtime.py"
        result = subprocess.run(
            [sys.executable, str(checker_path), "tree", "--runtime-root", str(final_root)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            check=False,
        )
        sys.stdout.write(result.stdout)
        if result.returncode != 0:
            fail("final private runtime checker failed with rc={}".format(result.returncode))
        manifest_path = final_root / checker.MANIFEST_NAME
        print("PRIVATE_RUNTIME_ROOT={}".format(final_root))
        print("PRIVATE_RUNTIME_CONTENT_SHA256={}".format(content))
        print("PRIVATE_RUNTIME_MANIFEST_SHA256={}".format(sha256_path(manifest_path)))
        print("PRIVATE_RUNTIME_CHECKER_SHA256={}".format(sha256_path(checker_path)))
        for name, relative in checker.ENTRYPOINTS.items():
            print("PRIVATE_RUNTIME_{}_SHA256={}".format(name.upper(), sha256_path(final_root / relative)))
        for name, relative in checker.RUNTIME_HELPERS.items():
            print("PRIVATE_RUNTIME_{}_SHA256={}".format(name.upper(), sha256_path(final_root / relative)))
        return 0
    except (PackagingError, checker.VerificationError, OSError, subprocess.SubprocessError) as exc:
        print("RUNTIME_PACKAGE_FAIL {}".format(exc), file=sys.stderr)
        if staging is not None:
            print("RUNTIME_PACKAGE_STAGING_PRESERVED={}".format(staging), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
