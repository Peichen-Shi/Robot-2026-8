#!/usr/bin/env python3
"""Recreate the Lab4 simulator/bridge pair; never reuse a stale ROS registration.

Only startup processes are managed here. Each new simulation gets a fresh
bridge and a live-feedback check. The simulator binary is never modified.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xmlrpc.client

WS = Path(__file__).resolve().parents[1]
SIM = Path.home() / 'vnav-builds/lab4/lab4.x86_64'
ASSETS = WS / 'scripts'
# catkin may execute generated wrappers under ros_ws/devel/.private.
# Match both source paths and wrappers so orphan UDP listeners cannot survive.
BRIDGE_ROOT = str(WS) + '/'


def processes():
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            stat = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z':
                continue
            argv = [v.decode(errors='replace') for v in (entry / 'cmdline').read_bytes().split(b'\0') if v]
            yield int(entry.name), argv, stat[19]
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue


def stop_matching(predicate, label):
    selected = {pid: birth for pid, argv, birth in processes()
                if pid != os.getpid() and predicate(argv)}
    if not selected:
        return
    print('Stopping ' + label + ': ' + ', '.join(map(str, selected)), flush=True)
    for sig, grace in [(signal.SIGTERM, 5), (signal.SIGKILL, 2)]:
        live = {pid: birth for pid, argv, birth in processes() if selected.get(pid) == birth}
        for pid in live:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        end = time.monotonic() + grace
        while time.monotonic() < end:
            remaining = [pid for pid, argv, birth in processes() if live.get(pid) == birth]
            if not remaining:
                return
            time.sleep(.1)
    raise RuntimeError('Could not stop ' + label)


def is_launch(argv, package, launch):
    return any(Path(a).name == 'roslaunch' for a in argv) and package in argv and launch in argv


def stop_flight():
    def flight(argv):
        binaries = {'controller_fast', 'controller_node', 'trajectory_fast',
                    'trajectory_generation_node', 'traj_vertices_publisher', 'simple_traj_planner'}
        local = any(a.startswith(str(WS) + '/') and Path(a).name in binaries for a in argv)
        monitor = any(a.startswith(str(WS / 'scripts') + '/') and
                      Path(a).name in {'monitor_fast.py', 'monitor_run.py', 'visualize_race.py', 'lab4_gate_timer.py'} for a in argv)
        orchestrator = any(a.startswith(str(WS) + '/') and Path(a).name == 'speed_trials.py' for a in argv)
        return local or monitor or orchestrator or is_launch(argv, 'planner_pkg', 'traj_gen.launch') or \
            is_launch(argv, 'trajectory_generation', 'traj_following.launch')
    stop_matching(flight, 'old Lab4 flight nodes')
    try:
        from tesse.env import Env
        from tesse.msgs import PropSpeeds
        Env(simulation_ip='127.0.0.1', own_ip='127.0.0.1').send(PropSpeeds(0, 0, 0, 0))
    except Exception as error:
        print('Motor stop unavailable:', error, flush=True)


def stop_pair():
    stop_matching(lambda argv: is_launch(argv, 'tesse_ros_bridge', 'tesse_quadrotor_bridge.launch') or
                  any(a.startswith(BRIDGE_ROOT) and Path(a).name in
                      {'tesse_ros_node.py', 'quadrotor_control_interface.py'} for a in argv),
                  'old simulator bridge')
    stop_matching(lambda argv: str(SIM) in argv, 'old Lab4 simulator')


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1
        return connection


def master_ready():
    try:
        with xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311'),
                                       transport=TimeoutTransport()) as master:
            return master.getPid('/lab4_startup')[0] == 1
    except (OSError, xmlrpc.client.Error):
        return False


def spawn(argv, logfile, session):
    with open(session / logfile, 'ab', buffering=0) as log:
        return subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)


def ensure_master(session):
    if master_ready():
        return
    spawn(['roscore', '-p', '11311'], 'roscore.log', session)
    end = time.monotonic() + 15
    while time.monotonic() < end:
        if master_ready():
            return
        time.sleep(.2)
    raise RuntimeError('ROS master did not start; see roscore.log')


def wait_metadata(simulator, session):
    # Run the request in a bounded child: TESSE can otherwise block on a broken
    # TCP reply while ROS simulation time is stopped.
    probe = '''import time
from tesse.env import Env
from tesse.msgs import MetadataRequest
env = Env(simulation_ip="127.0.0.1", own_ip="127.0.0.1")
deadline = time.monotonic() + 18
while time.monotonic() < deadline:
    try:
        reply = env.request(MetadataRequest(), timeout=1)
        if reply is not None and reply.metadata:
            print("Simulator metadata interface ready.", flush=True)
            raise SystemExit(0)
    except (OSError, ValueError):
        pass
    time.sleep(.2)
raise SystemExit(1)
'''
    try:
        check = subprocess.run([sys.executable, '-c', probe], timeout=23, check=False)
    except subprocess.TimeoutExpired:
        raise RuntimeError('Simulator metadata interface timed out; see unity.log')
    if simulator.poll() is not None or check.returncode:
        raise RuntimeError('Simulator failed to become ready; see simulator.log and unity.log')


def prepare(session):
    stop_flight()
    # A bridge may answer XML-RPC although its UDP listener is no longer alive.
    # Stop the old pair first, then start the simulator before its new bridge.
    for attempt in (1, 2):
        print('Preparing simulator and bridge, attempt %d/2.' % attempt, flush=True)
        stop_pair()
        try:
            ensure_master(session)
            simulator_command = [str(SIM), '-screen-fullscreen', '0', '-screen-width', '640',
                                 '-screen-height', '448', '-screen-quality', 'Fastest',
                                 '-job-worker-count', '1', '-logFile', str(session / 'unity.log')]
            if os.environ.get('LAB4_SIM_HEADLESS', '0') == '1':
                simulator_command += ['-batchmode', '-nographics']
            simulator = spawn(simulator_command, 'simulator.log', session)
            wait_metadata(simulator, session)
            bridge = spawn(['roslaunch', 'tesse_ros_bridge', 'tesse_quadrotor_bridge.launch',
                            'sim_ip:=127.0.0.1', 'self_ip:=127.0.0.1',
                            'publish_collisions:=true', 'publish_metadata:=true', 'clock_from_metadata_udp:=true', 'persistent_metadata_listener:=true'], 'bridge.log', session)
            check = subprocess.run([sys.executable, str(ASSETS / 'wait_feedback.py'),
                                    '--timeout', '20', '--output', str(session / 'feedback_ready.json')],
                                   timeout=28, check=False)
            if check.returncode or simulator.poll() is not None or bridge.poll() is not None:
                raise RuntimeError('State or simulation clock is not advancing')
            (session / 'startup.json').write_text(json.dumps({
                'attempt': attempt, 'simulator_pid': simulator.pid, 'bridge_pid': bridge.pid,
                'headless': os.environ.get('LAB4_SIM_HEADLESS', '0') == '1',
                'simulator_command': simulator_command,
                'live_state_and_clock': True}, indent=2), encoding='utf-8')
            print('State feedback and /clock ready. Fresh simulator/bridge connected.', flush=True)
            return
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
            print('Startup attempt failed:', error, flush=True)
    stop_pair()
    raise RuntimeError('Lab4 startup failed after 2 attempts. Logs: ' + str(session))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('session', nargs='?')
    parser.add_argument('--stop-flight', action='store_true')
    args = parser.parse_args()
    if args.stop_flight:
        stop_flight()
    else:
        if not args.session:
            parser.error('session is required')
        session = Path(args.session).resolve()
        if session.parent != (WS / 'results').resolve():
            parser.error('session must be directly inside ' + str(WS / 'results'))
        prepare(session)
