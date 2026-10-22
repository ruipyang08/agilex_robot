"""Exercise the relocated application with an observation-only YAML recipe."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def main(prefix):
    assert not Path('/workspaces/ranger_arm/agilex_ws').exists()
    assert not (prefix / 'share/agx_workbench_workflow').exists()
    executable = prefix / 'lib/agx_robot_workflow/run_robot_workflow'
    recipe = prefix / 'share/agx_task_workflow/examples/record_value/workflow.yaml'
    master = subprocess.Popen(['roscore', '-p', '11311'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 15
        while True:
            try:
                with socket.create_connection(('127.0.0.1', 11311), timeout=.3):
                    break
            except OSError:
                assert master.poll() is None, 'isolated ROS master exited'
                if time.monotonic() >= deadline:
                    raise RuntimeError('isolated ROS master did not start')
                time.sleep(.1)
        for check in (True, False):
            summary = Path('/tmp/robot-workflow-' + str(check) + '.json')
            command = [str(executable), '_config:=' + str(recipe),
                       '_summary:=' + str(summary), '_check_only:=' + str(check).lower()]
            result = subprocess.run(command, capture_output=True, text=True, timeout=25)
            assert result.returncode == 0, result.stdout + result.stderr
            report = json.loads(summary.read_text())
            assert report['valid'] is True, report
            assert report['validation_only'] == check, report
            assert report['completion'] == ('Validated' if check else 'Succeeded'), report
            assert report['grasp_dispatched'] is False, report
            assert [task['type'] for task in report['tasks']] == ['record_value'], report
            assert report['tasks'][0]['dispatched'] == (not check), report
            print(json.dumps(report), flush=True)
    finally:
        if master.poll() is None:
            os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait()


if __name__ == '__main__':
    main(Path(sys.argv[1]))
