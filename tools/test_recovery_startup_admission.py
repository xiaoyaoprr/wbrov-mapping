"""Run the launcher's actual admission validator without ROS or hardware."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapping', type=Path, required=True)
    args = parser.parse_args()
    source = args.mapping.read_text()
    function = source[source.index('safe_reloc_state_status() {'):source.index('# BEGIN GRAPH_DISPLAY_SESSION_MONITOR')]
    embedded = function.split("<<'CORNER_RECOVERY_STATE_PY'\n", 1)[1].split('\nCORNER_RECOVERY_STATE_PY', 1)[0]
    constants = {}
    for node in ast.parse(embedded).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('expected_nodes', 'expected_sources'):
            constants[node.targets[0].id] = ast.literal_eval(node.value)
    token = 'isolated-validator-test'
    token_sha = hashlib.sha256(token.encode()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='recovery-startup-') as directory:
        root = Path(directory)
        evidence = root / 'corner_recovery'
        evidence.mkdir()
        config = dict(fixture_mode=False, session_token=token, session_token_sha256=token_sha,
                      database_sha256='0'*64, body_from_camera=[0,0,1,0,-1,0,0,0,0,-1,0,0],
                      camera_frame='zed_left_camera_optical_frame', pending_max_age_s=30,
                      image_buffer_capacity=128, tag_side_m=.08, min_tag_side_px=8,
                      frame_ready_topic=None, nodes=constants['expected_nodes'], sources=constants['expected_sources'])

        def write(path, value, jsonl=False):
            path.write_text(json.dumps(value) + ('\n' if jsonl else ''))
            path.chmod(0o600)

        write(root / 'corner_recovery_config.json', config)
        write(evidence / 'ready.json', dict(pid=123, start_ticks='456'))
        write(evidence / 'events.jsonl', dict(event='tag_visualization', graph_update_authorized=False), True)
        write(evidence / 'graph_states.jsonl', dict(reset_epoch=0, graph_revision=4, pose_count=1, actual_message_gid='a'*48), True)
        result = dict(state='closed', fixture_mode=False, image_buffer_capacity=128,
                      buffer_dropped=dict(rgb=0,camera_info=0), ready_frames=0, graph_states=81,
                      source_compromised=False, graph_state_rejections=0, counts=dict(rejected=0),
                      graph_epochs=[0], committed_epochs=[], bridge_proposals=0, bridge_terminals={})
        write(evidence / 'RESULT.json', result)
        write(evidence / 'cleanup.json', dict(primary_exception=None,cleanup_errors=[],bridge_complete=True))
        setup = ('python3_runtime=/usr/bin/python3\nrun_dir=' + repr(str(root)) + '\n'
                 'safe_reloc_gid_frontend_pid=123\ndeclare -A owned_start_ticks=([123]=456)\n'
                 'safe_relocalization_session_token_sha256=' + token_sha + '\n'
                 'safe_relocalization_database_sha256=' + '0'*64 + '\n')

        def check(name, phase, expected):
            completed = subprocess.run(['/bin/bash', '-c', setup+function+'\nsafe_reloc_state_status '+phase], capture_output=True, text=True)
            assert completed.returncode == expected, (name,completed.returncode,completed.stderr)
            print('PASS:', name)

        check('stationary graph admits display without a new pixel join', 'admitted', 0)
        check('the same evidence does not claim full recovery readiness', 'live', 2)
        check('zero recovery frames can close a stationary main map normally', 'terminal', 0)
        write(evidence / 'events.jsonl', dict(event='frame_ready'), True)
        check('full frame remains distinguishable', 'live', 0)
        write(evidence / 'events.jsonl', dict(event='sources_invalidated'), True)
        check('invalid source cannot admit display', 'admitted', 1)
        write(evidence / 'events.jsonl', dict(event='tag_visualization'), True)
        write(evidence / 'ready.json', dict(pid=124,start_ticks='456'))
        check('wrong process identity is rejected', 'admitted', 1)
        write(evidence / 'ready.json', dict(pid=123,start_ticks='456'))
        write(evidence / 'graph_states.jsonl', dict(reset_epoch=0,graph_revision=0,pose_count=0,actual_message_gid='a'*48), True)
        check('no checked graph remains pending', 'admitted', 2)
        result['ready_frames']=-1
        write(evidence / 'RESULT.json', result)
        check('malformed terminal count remains rejected', 'terminal', 1)
        result['ready_frames']=0;result['source_compromised']=True
        write(evidence / 'RESULT.json', result)
        check('terminal compromised source remains rejected', 'terminal', 1)

    block = source.split('# BEGIN RECOVERY_STARTUP_ADMISSION\n',1)[1].split('# END RECOVERY_STARTUP_ADMISSION',1)[0]
    # Shorten only the test budget; make checks slow to catch sleep-count loops.
    block = block.replace('+ 30 ))', '+ 2 ))')
    setup = 'set -u\n' + '\n'.join(name+'=123' for name in ('bridge_pid','tf_pid','apriltag_pid','graph_display_pid','submap_display_pid','odom_pid','slam_pid','apriltag_keyframe_binder_pid','safe_reloc_gid_frontend_pid','launcher_lock_broker_pid')) + '\n'
    setup += 'owned_process_alive() { sleep .03; return 0; }\nsafe_reloc_state_status() { return 2; }\n'
    start = time.monotonic()
    completed = subprocess.run(['/bin/bash','-c',setup+block], capture_output=True,text=True,timeout=5)
    elapsed = time.monotonic()-start
    assert completed.returncode==1 and elapsed<3.5, (elapsed,completed)
    print('PASS: real elapsed-time deadline includes slow checks, %.2fs' % elapsed)


if __name__ == '__main__':
    main()
