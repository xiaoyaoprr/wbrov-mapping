#!/usr/bin/env python3
"""Write one immutable launcher-observed process identity file."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import stat


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def start_ticks(pid):
    value = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    close = value.rfind(")")
    require(close >= 0, "malformed /proc stat")
    fields = value[close + 2:].split()
    require(len(fields) >= 20, "short /proc stat")
    result = int(fields[19])
    require(result > 0, "invalid process start ticks")
    return result


def file_identity(path, require_one_link=False):
    canonical = path.resolve(strict=True)
    info = canonical.stat()
    require(stat.S_ISREG(info.st_mode), "identity path is not regular")
    if require_one_link:
        require(info.st_nlink == 1, "script must be a single-link file")
    require("\n" not in str(canonical) and "\r" not in str(canonical),
            "identity path contains newline")
    return canonical, info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--script", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(args.pid > 1 and args.output.is_absolute() and
            args.output.parent.is_dir() and not args.output.exists() and
            not args.output.is_symlink(), "invalid output/PID")
    observed_start = start_ticks(args.pid)
    exe_path, exe_info = file_identity(Path(f"/proc/{args.pid}/exe"))
    script_path, script_info = file_identity(args.script, True)
    # Recheck after all filesystem work so a short-lived/reused PID cannot be
    # serialized as the intended instance.
    require(start_ticks(args.pid) == observed_start,
            "process instance changed during identity capture")
    payload = (
        "schema_version=1\n"
        f"pid={args.pid}\n"
        f"process_start_ticks={observed_start}\n"
        f"exe_dev={exe_info.st_dev}\n"
        f"exe_ino={exe_info.st_ino}\n"
        f"script_dev={script_info.st_dev}\n"
        f"script_ino={script_info.st_ino}\n"
        f"exe_path={exe_path}\n"
        f"script_path={script_path}\n").encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(args.output, flags, 0o600)
    try:
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            require(written > 0, "short identity file write")
            offset += written
        os.fchmod(descriptor, 0o600)
        os.fsync(descriptor)
        info = os.fstat(descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_uid == os.geteuid() and
                info.st_nlink == 1 and stat.S_IMODE(info.st_mode) == 0o600,
                "identity output identity/mode mismatch")
    finally:
        os.close(descriptor)
    directory = os.open(args.output.parent, os.O_RDONLY |
                        getattr(os, "O_DIRECTORY", 0) |
                        getattr(os, "O_CLOEXEC", 0) |
                        getattr(os, "O_NOFOLLOW", 0))
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
    print(f"PROCESS_IDENTITY_OK pid={args.pid} start_ticks={observed_start}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

