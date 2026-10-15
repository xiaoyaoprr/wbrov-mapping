#!/usr/bin/env python3
"""ROS-free safe-relocalization engine behind authenticated local IPC."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import selectors
import signal
import socket
import stat
import struct
import sys
import time


MAGIC = b"WBRSRIPC"
VERSION = 1
HEADER = struct.Struct("!8sHHQQQIIII32s")
FRAGMENT_BYTES = 49152
MAX_PAYLOAD_BYTES = 16 * 1024 * 1024
MAX_FRAGMENTS = 1024

HELLO = 1
METADATA = 10
INFO = 11
ODOM = 12
CORE_STATE = 13
CORE_STATUS = 14
PROPOSAL = 20
ACK = 21
DETACH = 22
ERROR = 23
QUIESCE = 24
QUIESCED = 25
KNOWN_TYPES = {HELLO, METADATA, INFO, ODOM, CORE_STATE, CORE_STATUS,
               PROPOSAL, ACK, DETACH, ERROR, QUIESCE, QUIESCED}


class IpcError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise IpcError(message)


def strict_json(raw):
    def pairs(values):
        result = {}
        for key, value in values:
            require(key not in result, f"duplicate JSON key: {key}")
            result[key] = value
        return result

    def constant(value):
        raise IpcError(f"non-finite JSON constant: {value}")

    return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def sha256_bytes(value):
    return hashlib.sha256(value).digest()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while True:
            block = source.read(1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def send_logical(connection, message_type, sequence, ack_sequence=0,
                 stamp_ns=0, payload=b""):
    require(type(payload) is bytes and len(payload) <= MAX_PAYLOAD_BYTES,
            "invalid or oversized IPC payload")
    require(message_type in KNOWN_TYPES, "unknown IPC message type")
    require((message_type == ACK and sequence == 0 and ack_sequence > 0) or
            (message_type != ACK and sequence > 0),
            "invalid IPC sequence semantics")
    digest = sha256_bytes(payload)
    count = max(1, (len(payload) + FRAGMENT_BYTES - 1) // FRAGMENT_BYTES)
    require(count <= MAX_FRAGMENTS, "too many IPC fragments")
    for index in range(count):
        fragment = payload[index * FRAGMENT_BYTES:(index + 1) * FRAGMENT_BYTES]
        header = HEADER.pack(MAGIC, VERSION, message_type, sequence,
                             ack_sequence, stamp_ns, index, count,
                             len(payload), len(fragment), digest)
        sent = connection.send(header + fragment)
        require(sent == len(header) + len(fragment),
                "SOCK_SEQPACKET partial send")


def send_ack(connection, sequence):
    send_logical(connection, ACK, 0, ack_sequence=sequence)


class Receiver:
    def __init__(self):
        self.partial = {}

    def receive_one(self, connection):
        packet, _ancillary, flags, _address = connection.recvmsg(
            HEADER.size + FRAGMENT_BYTES)
        require(packet, "IPC peer disconnected")
        require(not (flags & socket.MSG_TRUNC), "oversized IPC datagram")
        require(len(packet) >= HEADER.size, "short IPC datagram")
        fields = HEADER.unpack(packet[:HEADER.size])
        (magic, version, message_type, sequence, ack_sequence, stamp_ns,
         index, count, total_size, fragment_size, digest) = fields
        require(magic == MAGIC and version == VERSION and
                message_type in KNOWN_TYPES, "invalid IPC header")
        require(0 < count <= MAX_FRAGMENTS and index < count and
                total_size <= MAX_PAYLOAD_BYTES and
                fragment_size <= FRAGMENT_BYTES and
                len(packet) == HEADER.size + fragment_size,
                "invalid IPC fragment metadata")
        if message_type == ACK:
            require(sequence == 0 and ack_sequence > 0 and index == 0 and
                    count == 1 and total_size == 0 and fragment_size == 0 and
                    digest == sha256_bytes(b""), "invalid IPC ACK")
            return {"type": message_type, "sequence": 0,
                    "ack_sequence": ack_sequence, "stamp_ns": stamp_ns,
                    "digest": digest, "payload": b""}
        require(sequence > 0, "non-ACK sequence must be positive")
        identity = (message_type, ack_sequence, stamp_ns, count,
                    total_size, digest)
        previous = self.partial.get(sequence)
        if previous is None:
            previous = {"identity": identity, "fragments": {}}
            self.partial[sequence] = previous
        require(previous["identity"] == identity,
                "one sequence changed fragment identity")
        fragment = packet[HEADER.size:]
        old = previous["fragments"].get(index)
        require(old is None or old == fragment,
                "one fragment index changed payload")
        previous["fragments"][index] = fragment
        if len(previous["fragments"]) != count:
            return None
        require(set(previous["fragments"]) == set(range(count)),
                "IPC fragment set has a gap")
        payload = b"".join(previous["fragments"][part]
                           for part in range(count))
        del self.partial[sequence]
        require(len(payload) == total_size and sha256_bytes(payload) == digest,
                "IPC logical payload digest/size mismatch")
        return {"type": message_type, "sequence": sequence,
                "ack_sequence": ack_sequence, "stamp_ns": stamp_ns,
                "digest": digest, "payload": payload}


def process_start_ticks(pid):
    line = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    close = line.rfind(")")
    require(close >= 0, "malformed /proc stat")
    fields = line[close + 2:].split()
    require(len(fields) >= 20, "short /proc stat")
    value = int(fields[19])
    require(value > 0, "invalid process start ticks")
    return value


def identity(path, single_link=False):
    canonical = path.resolve(strict=True)
    info = canonical.stat()
    require(stat.S_ISREG(info.st_mode), "identity path is not regular")
    if single_link:
        require(info.st_nlink == 1, "identity path must have one link")
    return {"path": str(canonical), "device": info.st_dev,
            "inode": info.st_ino}


def process_exe_identity(pid):
    return identity(Path(f"/proc/{pid}/exe"))


def verify_existing_at(directory_fd, name):
    try:
        info = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return
    require(stat.S_ISREG(info.st_mode) and info.st_uid == os.geteuid() and
            info.st_nlink == 1 and stat.S_IMODE(info.st_mode) == 0o600,
            "unsafe existing state target")


class AtomicState:
    def __init__(self, path):
        require(path.is_absolute() and path.name not in ("", ".", ".."),
                "state path must be absolute")
        self.name = path.name
        self.directory_fd = os.open(
            path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
            getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0))
        info = os.fstat(self.directory_fd)
        require(stat.S_ISDIR(info.st_mode) and info.st_uid == os.geteuid(),
                "state parent must be a current-user directory")
        verify_existing_at(self.directory_fd, self.name)
        self.counter = 0

    def write(self, value):
        verify_existing_at(self.directory_fd, self.name)
        self.counter += 1
        temporary = f".{self.name}.tmp.{os.getpid()}.{self.counter}"
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(temporary, flags, 0o600, dir_fd=self.directory_fd)
        renamed = False
        try:
            encoded = (canonical_json(value) + "\n").encode("utf-8")
            offset = 0
            while offset < len(encoded):
                written = os.write(descriptor, encoded[offset:])
                require(written > 0, "short state write")
                offset += written
            os.fchmod(descriptor, 0o600)
            os.fsync(descriptor)
            info = os.fstat(descriptor)
            require(stat.S_ISREG(info.st_mode) and info.st_uid == os.geteuid()
                    and info.st_nlink == 1 and
                    stat.S_IMODE(info.st_mode) == 0o600,
                    "state temporary identity/mode mismatch")
            os.close(descriptor)
            descriptor = -1
            os.rename(temporary, self.name, src_dir_fd=self.directory_fd,
                      dst_dir_fd=self.directory_fd)
            renamed = True
            os.fsync(self.directory_fd)
            verify_existing_at(self.directory_fd, self.name)
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            if not renamed:
                try:
                    os.unlink(temporary, dir_fd=self.directory_fd)
                except FileNotFoundError:
                    pass

    def close(self):
        if self.directory_fd >= 0:
            os.close(self.directory_fd)
            self.directory_fd = -1


class EventWriter:
    def __init__(self, path):
        require(path.is_absolute() and path.parent.is_dir() and
                not path.exists() and not path.is_symlink(),
                "events must be a new absolute file")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        self.descriptor = os.open(path, flags, 0o600)
        info = os.fstat(self.descriptor)
        require(stat.S_ISREG(info.st_mode) and info.st_uid == os.geteuid() and
                info.st_nlink == 1 and stat.S_IMODE(info.st_mode) == 0o600,
                "event file identity/mode mismatch")

    def write(self, value):
        encoded = (canonical_json(value) + "\n").encode("utf-8")
        offset = 0
        while offset < len(encoded):
            written = os.write(self.descriptor, encoded[offset:])
            require(written > 0, "short event write")
            offset += written
        os.fsync(self.descriptor)

    def close(self):
        if self.descriptor >= 0:
            os.close(self.descriptor)
            self.descriptor = -1


def load_core(path, expected_sha256):
    require(path.is_absolute() and sha256_file(path) == expected_sha256,
            "pure engine module hash mismatch")
    specification = importlib.util.spec_from_file_location(
        "wbrov_safe_reloc_engine_core", path)
    require(specification is not None and specification.loader is not None,
            "cannot load pure engine module")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    require(getattr(module, "PURE_ENGINE_ROS_NETWORK_DISABLED", False) is True,
            "pure engine module lacks network-disabled provenance")
    return module


class Runtime:
    def __init__(self, arguments, core):
        self.arguments = arguments
        self.core = core
        self.base_t_camera = core.pose((0.0, 0.0, 0.0),
                                       (-0.5, 0.5, -0.5, 0.5))
        self.engine = core.SafeRelocalizationEngine(
            self.base_t_camera, session_token=arguments.session_token)
        self.engine.set_authoritative_epoch(0, 0)
        if arguments.anchors is not None:
            anchors = core.strict_json_loads(
                arguments.anchors.read_text(encoding="utf-8"))
            self.engine.load_anchor_library(anchors, arguments.database_sha256)
        self.pending_tags = {}
        self.pending_infos = {}
        self.metadata_ledger = core.StampPayloadLedger()
        self.info_ledger = core.StampPayloadLedger()
        self.odom_ledger = core.StampPayloadLedger()
        self.pending_wire = None
        self.outgoing = {}
        self.acknowledged_retries = {}
        self.output_sequence = 0
        self.input_seen = {}
        self.highest_input_sequence = 0
        self.receiver = Receiver()
        self.connection = None
        self.peer = None
        self.detached = False
        self.hello_authenticated = False
        self.quiescing = False
        self.quiesce_started = None
        self.quiesced_sequence = None
        self.failure = None
        self.signal_number = None
        self.events = EventWriter(arguments.events)
        self.state = AtomicState(arguments.state)
        self.self_exe = process_exe_identity(os.getpid())
        self.self_script = identity(Path(__file__), single_link=True)
        self.self_start_ticks = process_start_ticks(os.getpid())
        self.event_sequence = 0
        self.last_state_kind = "starting"

    def install_signals(self):
        def handler(number, _frame):
            if self.signal_number is None:
                self.signal_number = number
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)
        signal.signal(signal.SIGHUP, handler)

    def snapshot(self, kind, ready, terminal=False, exit_code=None):
        result = {
            "schema_version": 2,
            "producer": "wbrov_safe_reloc_ipc_engine",
            "kind": kind,
            "ready": bool(ready and self.hello_authenticated),
            "terminal": bool(terminal),
            "pid": os.getpid(),
            "process_start_ticks": self.self_start_ticks,
            "exe": self.self_exe,
            "script": self.self_script,
            "socket_path": str(self.arguments.socket),
            "socket_mode": "0600",
            "transport": "AF_UNIX_SOCK_SEQPACKET",
            "peer": self.peer,
            "peer_credentials_bound": self.peer is not None,
            "peer_authenticated": self.hello_authenticated,
            "hello_authenticated": self.hello_authenticated,
            "detached": self.detached,
            "quiescing": self.quiescing,
            "quiesced_sequence": self.quiesced_sequence,
            "failure": self.failure,
            "event_sequence": self.event_sequence,
            "engine_snapshot": self.engine.snapshot(),
            "monotonic_ns": time.monotonic_ns(),
        }
        if exit_code is not None:
            result["exit_code"] = int(exit_code)
        return result

    def record(self, kind, payload=None, ready=True, terminal=False,
               exit_code=None):
        self.event_sequence += 1
        event = {"schema_version": 2,
                 "producer": "wbrov_safe_reloc_ipc_engine",
                 "kind": kind, "sequence": self.event_sequence,
                 "monotonic_ns": time.monotonic_ns()}
        if payload:
            event.update(payload)
        self.events.write(event)
        self.state.write(self.snapshot(kind, ready, terminal, exit_code))
        self.last_state_kind = kind

    def verify_frontend_peer(self, connection):
        credentials = connection.getsockopt(
            socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
        pid, uid, _gid = struct.unpack("3i", credentials)
        require(uid == os.geteuid() and pid > 1, "frontend peer uid/pid mismatch")
        start_ticks = process_start_ticks(pid)
        executable = process_exe_identity(pid)
        require(pid == self.arguments.expected_frontend_pid,
                "frontend peer PID does not match launcher binding")
        require(start_ticks == self.arguments.expected_frontend_start_ticks,
                "frontend peer process instance changed")
        require(executable["device"] == self.arguments.expected_frontend_dev and
                executable["inode"] == self.arguments.expected_frontend_ino and
                executable["path"] == str(
                    self.arguments.expected_frontend_exe.resolve(strict=True)),
                "frontend executable identity mismatch")
        return {"pid": pid, "uid": uid, "process_start_ticks": start_ticks,
                "exe": executable}

    def verify_peer_still_same(self):
        require(self.peer is not None, "frontend peer was never authenticated")
        require(process_start_ticks(self.peer["pid"]) ==
                self.peer["process_start_ticks"],
                "frontend process instance changed")
        executable = process_exe_identity(self.peer["pid"])
        require(executable == self.peer["exe"],
                "frontend executable identity changed")

    def next_output(self):
        self.output_sequence += 1
        return self.output_sequence

    def send_with_tracking(self, message_type, payload, stamp_ns=0):
        sequence = self.next_output()
        encoded = payload.encode("utf-8")
        send_logical(self.connection, message_type, sequence,
                     stamp_ns=stamp_ns, payload=encoded)
        self.outgoing[sequence] = {
            "type": message_type, "payload": encoded, "stamp_ns": stamp_ns,
            "first": time.monotonic(), "last": time.monotonic(),
            "attempts": 1,
        }
        return sequence

    def send_hello(self):
        payload = canonical_json({
            "schema_version": 1,
            "role": "safe_reloc_ipc_engine",
            "pid": os.getpid(),
            "process_start_ticks": self.self_start_ticks,
            "exe": self.self_exe,
            "script": self.self_script,
            "session_token_sha256": hashlib.sha256(
                self.arguments.session_token.encode("ascii")).hexdigest(),
        })
        return self.send_with_tracking(HELLO, payload)

    def validate_frontend_hello(self, payload):
        value = strict_json(payload.decode("utf-8"))
        require(set(value) == {"schema_version", "role", "pid",
                               "process_start_ticks", "exe",
                               "session_token_sha256"},
                "frontend HELLO schema mismatch")
        require(value["schema_version"] == 1 and
                value["role"] == "safe_reloc_gid_frontend" and
                value["pid"] == self.peer["pid"] and
                value["process_start_ticks"] ==
                self.peer["process_start_ticks"] and
                value["exe"] == self.peer["exe"] and
                value["session_token_sha256"] == hashlib.sha256(
                    self.arguments.session_token.encode("ascii")).hexdigest(),
                "frontend HELLO identity/session mismatch")

    def publish(self, proposals=None, retry=False):
        if retry:
            require(self.pending_wire is not None,
                    "cannot retry an absent proposal transaction")
            encoded = self.pending_wire["data"]
        else:
            require(proposals is not None,
                    "new proposal publication requires evidence")
            payload = self.core.build_transaction_payload(self.engine, proposals)
            encoded = canonical_json(payload)
        sequence = self.send_with_tracking(PROPOSAL, encoded)
        now = time.monotonic()
        if not retry:
            transaction = self.engine.pending_transaction
            self.pending_wire = {
                "token": transaction["transaction_token"],
                "data": encoded,
                "created_monotonic_s": transaction["created_monotonic_s"],
                "last_publish_monotonic_s": now,
                "attempts": 1, "transport_sequence": sequence,
            }
            self.record("proposal_batch_sent", {
                "transaction_token": transaction["transaction_token"],
                "transport_sequence": sequence, "attempt": 1})
        else:
            self.pending_wire["last_publish_monotonic_s"] = now
            self.pending_wire["attempts"] += 1
            self.pending_wire["transport_sequence"] = sequence
            self.record("proposal_batch_retried", {
                "transaction_token": self.pending_wire["token"],
                "transport_sequence": sequence,
                "attempt": self.pending_wire["attempts"]})

    def join(self, stamp_ns):
        if stamp_ns not in self.pending_tags or stamp_ns not in self.pending_infos:
            return
        _, observations = self.pending_tags.pop(stamp_ns)
        _, ref_id, map_id = self.pending_infos.pop(stamp_ns)
        if stamp_ns < self.engine.epoch_start_stamp_ns:
            self.record("stale_pre_epoch_exact_join_rejected", {
                "image_stamp_ns": stamp_ns,
                "epoch_start_stamp_ns": self.engine.epoch_start_stamp_ns,
                "reset_epoch": self.engine.reset_epoch})
            return
        for observation in observations:
            binding = dict(observation)
            binding.update({"accepted": True, "image_stamp_ns": stamp_ns,
                            "keyframe_id": ref_id, "map_id": map_id,
                            "reset_epoch": self.engine.reset_epoch})
            proposals = self.engine.add_binding(binding, time.monotonic())
            if proposals:
                self.publish(proposals)

    def process_data(self, message):
        message_type = message["type"]
        stamp_ns = message["stamp_ns"]
        raw = message["payload"].decode("utf-8")
        now = time.monotonic()
        if message_type == METADATA:
            parsed_stamp, observations = self.core.parse_metadata(raw)
            require(parsed_stamp == stamp_ns, "metadata IPC/header stamp mismatch")
            if not self.metadata_ledger.remember(
                    stamp_ns, {"stamp_ns": stamp_ns,
                               "observations": observations}, now,
                    "AprilTag metadata"):
                return
            self.pending_tags[stamp_ns] = (now, observations)
            self.join(stamp_ns)
        elif message_type == INFO:
            value = strict_json(raw)
            require(set(value) == {"stamp_ns", "ref_id", "map_id"} and
                    type(value["stamp_ns"]) is int and value["stamp_ns"] > 0 and
                    type(value["ref_id"]) is int and value["ref_id"] > 0 and
                    type(value["map_id"]) is int and value["map_id"] >= 0 and
                    value["stamp_ns"] == stamp_ns,
                    "Info IPC payload invalid")
            if not self.info_ledger.remember(stamp_ns, value, now,
                                             "RTAB-Map Info"):
                return
            self.pending_infos[stamp_ns] = (
                now, value["ref_id"], value["map_id"])
            self.join(stamp_ns)
        elif message_type == ODOM:
            value = strict_json(raw)
            require(set(value) == {"stamp_ns", "lost"} and
                    type(value["stamp_ns"]) is int and value["stamp_ns"] > 0 and
                    type(value["lost"]) is bool and
                    value["stamp_ns"] == stamp_ns,
                    "OdomInfo IPC payload invalid")
            if value["stamp_ns"] < self.engine.epoch_start_stamp_ns:
                self.record("pre_epoch_odom_info_rejected", value)
                return
            if not self.odom_ledger.remember(stamp_ns, value, now,
                                             "OdomInfo"):
                return
            if self.engine.mark_odom(value["lost"]):
                self.pending_tags.clear()
                self.pending_infos.clear()
                self.record("awaiting_authoritative_reset_epoch", {
                    "last_reset_epoch": self.engine.reset_epoch})
        elif message_type == CORE_STATE:
            state = self.core.strict_json_loads(raw)
            previous_epoch = self.engine.reset_epoch
            self.engine.apply_core_state(state)
            if self.engine.reset_epoch != previous_epoch:
                self.pending_tags.clear()
                self.pending_infos.clear()
                self.metadata_ledger.clear()
                self.info_ledger.clear()
            proposals = [] if self.quiescing else self.engine.ready_proposals(now)
            if proposals:
                self.publish(proposals)
            self.record("authoritative_core_state", {
                "reset_epoch": self.engine.reset_epoch,
                "epoch_start_stamp_ns": self.engine.epoch_start_stamp_ns,
                "graph_revision": self.engine.core_graph_revision})
        elif message_type == CORE_STATUS:
            status = self.core.strict_json_loads(raw)
            committed = self.engine.apply_core_status(status)
            if (self.engine.pending_transaction is None and
                    self.pending_wire is not None and
                    status.get("transaction_token") ==
                    self.pending_wire["token"]):
                self.pending_wire = None
            self.record("core_terminal_status", {
                "committed": committed, "status": status})
        else:
            raise IpcError("unexpected frontend data message type")

    def process_message(self, message):
        if message["type"] == ACK:
            require(self.hello_authenticated,
                    "engine received ACK before authenticated frontend HELLO")
            acknowledged = message["ack_sequence"]
            if acknowledged not in self.outgoing:
                retry = self.acknowledged_retries.get(acknowledged)
                require(retry is not None and retry[0] > 0 and
                        time.monotonic() < retry[1],
                        "ACK names unknown/already-ACKed output sequence")
                retry[0] -= 1
                if retry[0] == 0:
                    del self.acknowledged_retries[acknowledged]
                return
            attempts = self.outgoing[acknowledged]["attempts"]
            if attempts > 1:
                self.acknowledged_retries[acknowledged] = [
                    attempts - 1, time.monotonic() + 5.0]
            del self.outgoing[acknowledged]
            return
        sequence = message["sequence"]
        identity_value = (message["type"], message["stamp_ns"],
                          message["digest"])
        old = self.input_seen.get(sequence)
        if old is not None:
            require(message["type"] != HELLO,
                    "frontend HELLO must be unique, not replayed")
            require(old == identity_value,
                    "replayed input sequence changed type/stamp/digest")
            send_ack(self.connection, sequence)
            return
        require(sequence > self.highest_input_sequence,
                "new input sequence is not strictly monotonic")
        require(len(self.input_seen) < 262144,
                "input replay ledger capacity exhausted")
        if message["type"] == HELLO:
            require(not self.hello_authenticated and self.peer is not None and
                    self.highest_input_sequence == 0 and sequence == 1,
                    "frontend HELLO order invalid")
            self.validate_frontend_hello(message["payload"])
            self.hello_authenticated = True
        elif message["type"] == QUIESCE:
            require(self.hello_authenticated and not self.quiescing and
                    not self.detached and message["payload"] == b"",
                    "invalid or duplicate frontend QUIESCE")
            self.quiescing = True
            self.quiesce_started = time.monotonic()
            self.pending_tags.clear()
            self.pending_infos.clear()
        elif message["type"] == DETACH:
            require(self.hello_authenticated and message["payload"] == b"" and
                    self.peer is not None and self.quiescing and
                    self.quiesced_sequence is not None and
                    self.quiesced_sequence not in self.outgoing and
                    self.safe_to_quiesce(),
                    "invalid frontend DETACH")
            self.verify_peer_still_same()
            self.detached = True
        elif message["type"] in {METADATA, INFO, ODOM, CORE_STATE, CORE_STATUS}:
            require(self.hello_authenticated and not self.detached,
                    "data received before HELLO or after frontend DETACH")
            require(not self.quiescing or message["type"] == CORE_STATUS,
                    "non-status input received after QUIESCE")
            self.process_data(message)
        else:
            raise IpcError("frontend sent forbidden IPC type")
        self.input_seen[sequence] = identity_value
        self.highest_input_sequence = sequence
        send_ack(self.connection, sequence)
        if message["type"] == HELLO:
            self.record("frontend_authenticated", {"peer": self.peer})
        elif message["type"] == DETACH:
            self.record("frontend_detached", {"peer": self.peer}, ready=False)
        elif message["type"] == QUIESCE:
            self.record("quiesce_started", {}, ready=False)

    def safe_to_quiesce(self):
        proposal_outputs = any(item["type"] == PROPOSAL
                               for item in self.outgoing.values())
        return (self.engine.pending_transaction is None and
                self.pending_wire is None and not proposal_outputs)

    def timers(self):
        now = time.monotonic()
        for sequence, retry in list(self.acknowledged_retries.items()):
            if now >= retry[1]:
                del self.acknowledged_retries[sequence]
        expired_tags = [stamp for stamp, item in self.pending_tags.items()
                        if now - item[0] > 2.0]
        expired_infos = [stamp for stamp, item in self.pending_infos.items()
                         if now - item[0] > 2.0]
        for stamp_ns in expired_tags:
            self.pending_tags.pop(stamp_ns, None)
        for stamp_ns in expired_infos:
            self.pending_infos.pop(stamp_ns, None)
        if expired_tags or expired_infos:
            self.record("exact_join_expired", {
                "metadata_count": len(expired_tags),
                "info_count": len(expired_infos)})
        for sequence, item in list(self.outgoing.items()):
            if self.quiescing and item["type"] == PROPOSAL:
                continue
            if now - item["first"] >= 5.0 or item["attempts"] >= 20:
                raise IpcError("IPC output acknowledgement timeout")
            if now - item["last"] >= 0.25:
                send_logical(self.connection, item["type"], sequence,
                             stamp_ns=item["stamp_ns"], payload=item["payload"])
                item["last"] = now
                item["attempts"] += 1
        if self.pending_wire is not None:
            transport_sequence = self.pending_wire["transport_sequence"]
            if transport_sequence not in self.outgoing and not self.quiescing:
                action = self.core.proposal_retry_action(
                    self.pending_wire, self.engine.pending_transaction, now)
                if action == "clear":
                    self.pending_wire = None
                elif action == "retry":
                    self.publish(retry=True)
                elif action == "timeout":
                    raise IpcError("proposal terminal status timeout")
        if self.quiescing and not self.detached:
            if (self.quiesced_sequence is None and self.safe_to_quiesce()):
                self.quiesced_sequence = self.send_with_tracking(
                    QUIESCED, canonical_json({"safe": True}))
                self.record("quiesced", {
                    "transport_sequence": self.quiesced_sequence}, ready=False)
            if (self.quiesce_started is not None and
                    now - self.quiesce_started >= 7.0 and
                    (not self.safe_to_quiesce() or
                     self.quiesced_sequence in self.outgoing)):
                raise IpcError("quiesce timed out with unknown Core transaction outcome")

    def run(self):
        self.install_signals()
        server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            require(not self.arguments.socket.exists() and
                    not self.arguments.socket.is_symlink(),
                    "IPC socket path already exists")
            server.bind(str(self.arguments.socket))
            os.chmod(self.arguments.socket, 0o600, follow_symlinks=False)
            socket_info = os.lstat(self.arguments.socket)
            require(stat.S_ISSOCK(socket_info.st_mode) and
                    socket_info.st_uid == os.geteuid() and
                    stat.S_IMODE(socket_info.st_mode) == 0o600,
                    "IPC socket identity/mode mismatch")
            server.listen(1)
            server.setblocking(False)
            self.record("socket_ready", {"ros_network": False}, ready=False)
            selector = selectors.DefaultSelector()
            selector.register(server, selectors.EVENT_READ)
            while self.connection is None:
                if self.signal_number is not None:
                    raise IpcError("signal before authenticated frontend")
                if not selector.select(0.1):
                    continue
                connection, _address = server.accept()
                connection.setblocking(True)
                self.peer = self.verify_frontend_peer(connection)
                self.connection = connection
            selector.unregister(server)
            selector.register(self.connection, selectors.EVENT_READ)
            self.send_hello()
            last_peer_check = time.monotonic()
            while True:
                if self.signal_number is not None:
                    if self.signal_number == signal.SIGINT and self.detached:
                        self.record("final", {"exit_reason": "SIGINT"},
                                    ready=False, terminal=True, exit_code=130)
                        return 130
                    raise IpcError("engine signal before clean frontend detach")
                if self.detached:
                    # DETACH was authenticated and ACKed while the frontend was
                    # alive.  The launcher now reaps that peer, validates its
                    # terminal state, and only then signals this engine.
                    time.sleep(0.02)
                    continue
                for _key, _mask in selector.select(0.05):
                    message = self.receiver.receive_one(self.connection)
                    if message is not None:
                        self.process_message(message)
                self.timers()
                if time.monotonic() - last_peer_check >= 0.25:
                    self.verify_peer_still_same()
                    last_peer_check = time.monotonic()
        # The reviewed pure state-machine module defines its own strict
        # RelocalizationError type.  Treat every ordinary exception from that
        # frozen module as a terminal refusal and persist it atomically, rather
        # than allowing Python to unwind with stale non-terminal state.
        except Exception as error:
            self.failure = str(error)
            try:
                self.record("failure", {"reason": self.failure}, ready=False,
                            terminal=True, exit_code=1)
            except BaseException:
                pass
            print(f"SAFE_RELOC_IPC_ENGINE_REFUSED: {error}", file=sys.stderr)
            return 1
        finally:
            if self.connection is not None:
                self.connection.close()
            server.close()
            try:
                self.arguments.socket.unlink()
            except FileNotFoundError:
                pass
            self.events.close()
            self.state.close()


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--core-module", type=Path, required=True)
    parser.add_argument("--core-module-sha256", required=True)
    parser.add_argument("--socket", type=Path)
    parser.add_argument("--state", type=Path)
    parser.add_argument("--events", type=Path)
    parser.add_argument("--session-token", required=True)
    parser.add_argument("--anchors", type=Path)
    parser.add_argument("--database-sha256")
    parser.add_argument("--expected-frontend-exe", type=Path)
    parser.add_argument("--expected-frontend-pid", type=int)
    parser.add_argument("--expected-frontend-start-ticks", type=int)
    parser.add_argument("--expected-frontend-dev", type=int)
    parser.add_argument("--expected-frontend-ino", type=int)
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args(argv)


def validate_args(arguments):
    require(arguments.core_module.is_absolute() and
            len(arguments.core_module_sha256) == 64 and
            all(item in "0123456789abcdef"
                for item in arguments.core_module_sha256),
            "core module path/hash invalid")
    require(len(arguments.session_token) == 64 and
            all(item in "0123456789abcdef"
                for item in arguments.session_token),
            "session token must be 64 lowercase hexadecimal characters")
    if arguments.anchors is None:
        require(arguments.database_sha256 is None,
                "database SHA requires anchors")
    else:
        require(arguments.anchors.is_absolute() and
                isinstance(arguments.database_sha256, str) and
                len(arguments.database_sha256) == 64,
                "anchors require database SHA")
    if arguments.check_only:
        require(arguments.socket is None and arguments.state is None and
                arguments.events is None and
                arguments.expected_frontend_exe is None and
                arguments.expected_frontend_pid is None and
                arguments.expected_frontend_start_ticks is None and
                arguments.expected_frontend_dev is None and
                arguments.expected_frontend_ino is None,
                "check-only cannot create runtime state or bind peer identity")
    else:
        for path in (arguments.socket, arguments.state, arguments.events,
                     arguments.expected_frontend_exe):
            require(path is not None and path.is_absolute(),
                    "runtime paths must be absolute")
        require(isinstance(arguments.expected_frontend_pid, int) and
                arguments.expected_frontend_pid > 1 and
                isinstance(arguments.expected_frontend_start_ticks, int) and
                arguments.expected_frontend_start_ticks > 0,
                "runtime requires exact expected frontend PID/start ticks")
        require(arguments.state.parent == arguments.events.parent,
                "state/events need one existing evidence directory")
        for directory in (arguments.socket.parent, arguments.state.parent):
            info = directory.lstat()
            require(directory.resolve(strict=True) == directory and
                    stat.S_ISDIR(info.st_mode) and
                    not directory.is_symlink() and
                    info.st_uid == os.geteuid() and
                    stat.S_IMODE(info.st_mode) == 0o700,
                    "socket and evidence parents must be canonical current-user 0700 directories")
        require(arguments.expected_frontend_dev is not None and
                arguments.expected_frontend_dev > 0 and
                arguments.expected_frontend_ino is not None and
                arguments.expected_frontend_ino > 0,
                "frontend file identity is required")


def main(argv=None):
    arguments = parse_args(sys.argv[1:] if argv is None else argv)
    validate_args(arguments)
    core = load_core(arguments.core_module, arguments.core_module_sha256)
    if arguments.check_only:
        engine = core.SafeRelocalizationEngine(
            core.pose((0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5)),
            session_token=arguments.session_token)
        engine.set_authoritative_epoch(0, 0)
        require(engine.snapshot()["graph_mutation_owned_by_this_node"] is False,
                "pure engine observer boundary failed")
        print("SAFE_RELOC_IPC_ENGINE_CHECK_OK ros_network=false graph_mutation=false")
        return 0
    expected = identity(arguments.expected_frontend_exe)
    require(expected["device"] == arguments.expected_frontend_dev and
            expected["inode"] == arguments.expected_frontend_ino,
            "frontend executable changed before engine start")
    return Runtime(arguments, core).run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"SAFE_RELOC_IPC_ENGINE_REFUSED: {error}", file=sys.stderr)
        raise SystemExit(1)
