#!/usr/bin/env python3
"""Exercise actual launcher EXIT reporting without starting robot processes."""
from pathlib import Path
import argparse,json,subprocess,tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--outer',type=Path,required=True)
parser.add_argument('--mapping',type=Path,required=True)
args=parser.parse_args()
outer=args.outer.read_text(encoding='utf-8')
functions=outer[outer.index('show_mapping_failure_log() {'):outer.index('\nrequire_interactive() {')]
mapping=args.mapping.read_text(encoding='utf-8')
block=mapping[mapping.index('if [[ "$initial_frontend_stable" != true ]]; then'):mapping.index('\ncheck_reset_quarantine_runtime >"$run_dir/reset_runtime_odom_preexec_receipt.txt"')]
with tempfile.TemporaryDirectory(prefix='wbrov-reporting-') as tmp:
    root=Path(tmp)
    (root/'mapping.log').write_text('CAMERA STREAM FAILED TO START\n',encoding='utf-8')
    fixture='run_dir="$1"\nfailure_reason=""\nmapping_exit_status=1\ncleanup() { return 0; }\n'+functions+'\nfalse\non_exit\n'
    p=subprocess.run(['/usr/bin/bash','-c',fixture,'reporting-fixture',str(root)],capture_output=True,text=True,timeout=10)
    assert p.returncode==1 and 'CAMERA STREAM FAILED TO START' in p.stderr,p
    assert '未记录显式拒绝原因' not in (root/'exit_summary.txt').read_text(encoding='utf-8')
    for name in ('zed_rgbd.log','static_tf.log','apriltag.log','graph_display.log','submap_display.log'):
        (root/name).write_text('camera open fixture\n',encoding='utf-8')
    fixture='run_dir="$1"\ninitial_frontend_stable=false\nbridge_pid=123\ncritical_failure_reason=""\nowned_process_alive() { return 1; }\ntrap \'printf "%s\\n" "$critical_failure_reason" >&2\' EXIT\n'+block
    p=subprocess.run(['/usr/bin/bash','-c',fixture,'frontend-fixture',str(root)],capture_output=True,text=True,timeout=10)
    assert p.returncode==1 and 'ZED相机进程退出或身份失效' in p.stderr,p
print(json.dumps({'status':'passed','actual_exit_handler':True,'camera_failure_reason':True,'hardware_started':False}))
