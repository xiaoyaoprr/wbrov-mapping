#!/usr/bin/env python3
"""Native regression for the launcher's copied display bundle; no hardware.

Run on Jetson with --launcher and --runtime. Optionally --before-launcher
proves the original packaging fault is reproduced. ROS nodes use a separate
localhost-only domain and are created/destroyed within a bounded smoke test.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launcher', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--before-launcher', type=Path)
    args = parser.parse_args()
    runtime = args.runtime.resolve()
    env = dict(HOME=str(Path.home()), USER=os.environ.get('USER', 'exocean'),
               PATH='/usr/bin:/bin', LANG='C.UTF-8', PYTHONNOUSERSITE='1',
               PYTHONDONTWRITEBYTECODE='1', ROS_DOMAIN_ID='197', ROS_LOCALHOST_ONLY='1',
               AMENT_PREFIX_PATH=str(runtime)+':/opt/ros/humble',
               PYTHONPATH=':'.join(str(runtime/p) for p in
                   ('local/lib/python3.10/dist-packages', 'lib/python3.10/site-packages'))+
                   ':/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages',
               LD_LIBRARY_PATH=str(runtime/'lib')+':/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu')
    steps = []

    def run(name, command, expected=0, missing_module=False):
        result = subprocess.run(command, env=env, stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=30)
        print(result.stdout, end='')
        if result.returncode != expected or (missing_module and
                "No module named 'submap_display_geometry'" not in result.stderr):
            raise AssertionError(name+': '+result.stdout+result.stderr)
        steps.append(dict(name=name, returncode=result.returncode,
                          expected_returncode=expected, passed=True))

    def materialize(launcher, destination):
        source = launcher.read_text()
        function = source[source.index('prepare_visualization_runtime() {'):
                          source.index('\ncheck_camera_profile_dependencies() {')]
        variables = {'python3_runtime': '/usr/bin/python3.10'}
        files = {'graph_display_script': 'rtabmap_graph_display.py',
                 'graph_display_module': 'graph_display_geometry.py',
                 'submap_display_module': 'submap_display_geometry.py',
                 'test_rviz': 'rtabmap_dense_graph_optimized.rviz'}
        for key, name in files.items():
            variables[key] = str(runtime/'libexec'/name)
            match = re.search(r'^readonly '+key+r"_sha256='([0-9a-f]{64})'$", source, re.M)
            if match:
                variables[key+'_sha256'] = match[1]
        harness = 'set -euo pipefail\n'+''.join(k+'='+shlex.quote(v)+'\n' for k,v in variables.items())
        harness += function+'\nprepare_visualization_runtime '+shlex.quote(str(destination))+'\n'
        run('copy_'+launcher.parent.name, ['/usr/bin/bash', '-c', harness])
        return destination/'visualization_runtime'

    with tempfile.TemporaryDirectory(prefix='wbrov-display-regression-') as tmp:
        tmp = Path(tmp)
        if args.before_launcher:
            old = tmp/'before'; old.mkdir(mode=0o700)
            bundle = materialize(args.before_launcher, old)
            run('original_copy_reproduces_missing_module',
                ['/usr/bin/python3.10', '-B', str(bundle/'rtabmap_graph_display.py'), '--check-only'],
                expected=1, missing_module=True)
        fixed = tmp/'fixed'; fixed.mkdir(mode=0o700)
        bundle = materialize(args.launcher, fixed)
        for name in ('rtabmap_graph_display.py', 'graph_display_geometry.py',
                     'submap_display_geometry.py', 'rtabmap_dense_graph_optimized.rviz'):
            assert (bundle/name).read_bytes() == (runtime/'libexec'/name).read_bytes()
        command = ['/usr/bin/python3.10', '-B', str(bundle/'rtabmap_graph_display.py'), '--check-only']
        run('copied_bundle_imports_and_constructs_messages', command)
        helper = bundle/'submap_display_geometry.py'
        payload = helper.read_bytes(); helper.unlink()
        run('missing_helper_rejected', command, expected=1, missing_module=True)
        helper.write_bytes(payload); helper.chmod(0o600)
        smoke = '''import importlib.util,json,sys,time
from pathlib import Path
sys.path.insert(0,sys.argv[1])
import rtabmap_graph_display as graph
assert Path(graph.submap_geometry.__file__).parent == Path(sys.argv[1])
sys.path.insert(0,str(Path(sys.argv[2]).parent))
spec=importlib.util.spec_from_file_location('local_submap',sys.argv[2])
submap=importlib.util.module_from_spec(spec);spec.loader.exec_module(submap)
import rclpy
from rclpy.executors import SingleThreadedExecutor
rclpy.init(); nodes=[]; executor=SingleThreadedExecutor()
try:
    nodes=[graph.GraphDisplayNode(),submap.SubmapDisplayNode()]
    for node in nodes: executor.add_node(node)
    deadline=time.monotonic()+2
    while time.monotonic()<deadline: executor.spin_once(timeout_sec=.05)
finally:
    executor.shutdown()
    for node in nodes: node.destroy_node()
    rclpy.shutdown()
print(json.dumps(dict(status='PASS',actual_nodes_created=len(nodes),timers_spun=True,
                     clean_shutdown=True,hardware_started=False,ros_domain=197)))
'''
        run('actual_display_nodes_start_spin_shutdown', ['/usr/bin/python3.10', '-B', '-c', smoke,
            str(bundle), str(runtime/'libexec/rtabmap_submap_display.py')])
    print(json.dumps(dict(status='PASS', launcher_sha256=hashlib.sha256(args.launcher.read_bytes()).hexdigest(),
                         steps=steps, hardware_started=False, rviz_gui_started=False,
                         field_validated=False), sort_keys=True))


if __name__ == '__main__':
    main()
