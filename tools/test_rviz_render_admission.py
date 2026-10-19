"""Exercise the launcher's real rendering-admission function without hardware."""
import argparse
from pathlib import Path
import subprocess
import tempfile

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mapping',type=Path,required=True)
    args=parser.parse_args()
    source=args.mapping.read_text()
    start=source.index('wait_for_rviz_render_startup() {')
    function=source[start:source.index('\nstop_pid() {',start)]
    cases=(('ready',True,True,True,0),('black',False,True,True,1),
           ('identity_lost',True,False,True,1),('sibling_lost',True,True,False,1))
    for name,ready,owned,children,expected in cases:
        with tempfile.TemporaryDirectory(prefix='rviz-admission-') as directory:
            root=Path(directory)
            (root/'rviz.log').write_text('[WBROV_RVIZ_RENDER_READY] attempt=1\n' if ready else '[WBROV_RVIZ_RENDER_FAILED] attempt=2 retry=0\n')
            script='\n'.join(('set -eu',function,'run_dir="$1"','rviz_pid=123',
                'verify_critical_children_alive() { return '+('0' if children else '1')+'; }',
                'owned_process_alive() { return '+('0' if owned else '1')+'; }',
                'sleep() { :; }','wait_for_rviz_render_startup'))
            result=subprocess.run(['bash','-c',script,'render-admission',str(root)],capture_output=True,text=True)
            assert result.returncode==expected,(name,result.returncode,result.stdout,result.stderr)
    print('PASS: rendering readiness, persistent black window, identity loss, sibling loss')

if __name__=='__main__':main()
