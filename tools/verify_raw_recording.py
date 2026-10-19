#!/usr/bin/env python3
"""Finalize a stopped SVO recording; never treat shutdown alone as completeness."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess


def sha(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('missing/non-regular recording artifact: '+path.name)
    h = hashlib.sha256()
    before = path.stat()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    after = path.stat()
    if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
        raise ValueError('recording artifact changed while hashing')
    return h.hexdigest()


def verify(root, verifier):
    root = root.resolve(strict=True)
    top = json.loads((root/'status.json').read_text())
    if top.get('schema') == 'wbrov.raw-segments.v1':
        if top.get('state') != 'closed' or top.get('clean_shutdown') is not True:
            raise ValueError('segmented recording is still open or did not close cleanly')
        count = top.get('sealed_segments')
        if type(count) is not int or count <= 0:
            raise ValueError('segmented recording is empty')
        top_hash = sha(root/'status.json')
        segments = []
        previous = 0
        largest_gap = 0
        for index in range(count):
            segment = root/f'segment-{index:08d}'
            if segment.is_symlink() or not segment.is_dir():
                raise ValueError('missing/non-regular recording segment')
            result = verify(segment, verifier)
            first, last = result['frames']['first_source_ns'], result['frames']['last_source_ns']
            if first <= previous:
                raise ValueError('recording segments overlap or regress in source time')
            if previous: largest_gap = max(largest_gap, first-previous)
            previous = last
            segments.append({'directory':segment.name, **result})
        if top_hash != sha(root/'status.json'):
            raise ValueError('segmented recording changed during verification')
        return {'schema':'wbrov.raw-segment-verification.v1', 'complete':True,
                'scope':'closed per-segment SVO indexes and boundary decode; rotation gaps explicitly measured',
                'sensor_capture_losslessness_proven':False,'largest_boundary_gap_ns':largest_gap,
                'segments':segments,'status_sha256':top_hash,
                'frames':{'frames':sum(s['frames']['frames'] for s in segments),
                          'first_source_ns':segments[0]['frames']['first_source_ns'],
                          'last_source_ns':previous}}
    required = ['stereo.svo2', 'status.json', 'capture.jsonl', 'init.json',
                'runtime.json', 'recording.json', 'water_calibration.yml', 'sdk_version.txt',
                'camera_imu_extrinsics.json']
    pins = {name: sha(root/name) for name in required}
    status = json.loads((root/'status.json').read_text())
    if status.get('state') != 'closed' or status.get('clean_shutdown') is not True:
        raise ValueError('recording was not cleanly closed; retained for repair')
    if status.get('write_errors') != 0 or status.get('grab_errors') != 0:
        raise ValueError('capture/recording errors occurred; retained as partial')
    if not isinstance(status.get('sdk_frames_ingested'), int) or status['sdk_frames_ingested'] <= 0:
        raise ValueError('no SDK-ingested frames')
    if status.get('unique_computed_stamps') != status['sdk_frames_ingested']:
        raise ValueError('SDK ingestion does not cover exactly the unique grabbed source frames')
    version = subprocess.run([str(verifier), '--sdk-version'], check=True, capture_output=True, text=True, timeout=10).stdout.strip()
    if version != (root/'sdk_version.txt').read_text().strip():
        raise ValueError('SDK version differs from recording')
    q = subprocess.run([str(verifier), str(root/'stereo.svo2'), '--index-only'],
                       check=True, capture_output=True, text=True, timeout=60)
    decoded = json.loads(q.stdout.strip().splitlines()[-1])
    if decoded['frames'] != status['sdk_frames_ingested']:
        raise ValueError('SVO frame count differs from SDK ingestion count')
    if not (0 < decoded['first_source_ns'] <= status['first_source_ns'] <=
            status['last_source_ns'] <= decoded['last_source_ns']):
        raise ValueError('SVO does not cover recorded source-time range')
    if pins != {name: sha(root/name) for name in required}:
        raise ValueError('recording artifacts changed during verification')
    return {'schema':'wbrov.raw-recording-verification.v1', 'complete':True,
            'scope':'SDK-ingested frames, closed SVO index and boundary decode; full-frame decode is a separate replay check',
            'sensor_capture_losslessness_proven':False, 'computed_depth_stored':False,
            'sdk_version':version, 'frames':decoded, 'status':status, 'sha256':pins,
            'sdk_instant_write_status_is_diagnostic_final_index_controls_coverage':True,
            'replay_requires_same_sdk_calibration_and_depth_configuration':True}


def check_replay_contract(root, verifier, calibration, init, runtime):
    """Validate planned inputs before an offline consumer opens the SVO.

    Consumers must load these exact saved SDK configurations, then select the
    SVO input. This is an input contract, not proof that reconstruction passed.
    """
    root = root.resolve(strict=True)
    saved = json.loads((root/'verification.json').read_text())
    if saved.get('complete') is not True:
        raise ValueError('recording completeness was not verified')
    for name, expected in saved['sha256'].items():
        if Path(name).name != name or sha(root/name) != expected:
            raise ValueError('recorded artifact changed: '+name)
    version = subprocess.run([str(verifier), '--sdk-version'], check=True,
                             capture_output=True, text=True, timeout=10).stdout.strip()
    if version != saved['sdk_version']:
        raise ValueError('replay SDK version differs from recording')
    for name, source in [('water_calibration.yml', calibration), ('init.json', init),
                         ('runtime.json', runtime)]:
        if sha(source) != saved['sha256'][name]:
            raise ValueError('replay input differs from recording: '+name)
    return {'schema':'wbrov.raw-replay-input-contract.v1', 'compatible_inputs_verified':True,
            'sdk_version':version, 'recording_sha256':saved['sha256']['stereo.svo2'],
            'reconstruction_executed':False,
            'required_consumer_order':'load verified init/runtime/calibration, then select this SVO input'}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--directory', required=True, type=Path)
    p.add_argument('--verifier', required=True, type=Path)
    p.add_argument('--replay-calibration', type=Path)
    p.add_argument('--replay-init', type=Path)
    p.add_argument('--replay-runtime', type=Path)
    args = p.parse_args()
    replay = [args.replay_calibration, args.replay_init, args.replay_runtime]
    if any(replay):
        if not all(replay):
            p.error('replay requires calibration, init and runtime files together')
        try:
            result = check_replay_contract(args.directory, args.verifier, *replay)
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
            print(json.dumps({'compatible_inputs_verified':False,'reason':str(error)}))
            return 1
        print(json.dumps(result))
        return 0
    try:
        result = verify(args.directory, args.verifier)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        result = {'schema':'wbrov.raw-recording-verification.v1','complete':False,'reason':str(error)}
    target = args.directory/'verification.json'
    with target.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
