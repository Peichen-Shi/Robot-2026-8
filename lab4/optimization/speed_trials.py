#!/usr/bin/env python3
"""Record independent real TESSE trials with a persistent controller/bridge.

Simulator, bridge and one controller must already run. Repositioning occurs
only before recording. No actual state is synthesized or changed in a trial.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time
import threading
import csv
import sys
ANALYSIS_SCRIPTS=os.environ.get('LAB4_ANALYSIS_SCRIPTS',os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
sys.path.insert(0, ANALYSIS_SCRIPTS)
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Twist, Pose, PoseArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from trajectory_generation.msg import TrajectoryStatus
from tesse_ros_bridge.srv import RepositionRequestService
from tesse_ros_bridge.msg import CollisionStats


class WallReferenceTimer:
    """Publish a stationary hold independently of ROS clock delivery jitter.

    This never advances mission time or moves a trajectory. The normal state,
    reference and motor watchdogs remain enabled with their original limits.
    """
    def __init__(self, callback):
        self.stop_event = threading.Event()
        def run():
            while not self.stop_event.wait(0.02) and not rospy.is_shutdown():
                callback(None)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def shutdown(self):
        self.stop_event.set()


class Trials:
    def __init__(self, root, subset_csv=None, subset_first=9, options=None):
        self.options = options
        self.root, self.state, self.status = root, None, None
        self.pref_enabled = True
        self.hold_time_from_start = rospy.Duration(0)
        self.target = [.4,8.,2.,-math.pi/2]
        if options:
            self.target[2] = options.start_height
        self.start_target = list(self.target)
        self.last_mission_reference = None
        self.last_collision = None
        self.prepared_gates = None
        self.gates_released = False
        # Do not latch the active input: a later repeated trial must not receive
        # the previous trial's gates before the new preflight handoff.
        self.gate_release = rospy.Publisher('/active_race_gates', PoseArray, queue_size=1, latch=False)
        rospy.Subscriber('/desired_traj_vertices', PoseArray, self.on_prepared_gates, queue_size=1)
        self.expected_bag = None
        self.recording_opened = False
        rospy.Subscriber('/begin_write', String, self.on_recording_opened, queue_size=1)
        self.subset_csv, self.subset_first = subset_csv, subset_first
        if subset_csv:
            with open(subset_csv) as f:
                gate=list(csv.DictReader(f))[subset_first]
            yaw=float(gate['yaw_raw_rad'])
            self.target=[float(gate['x'])-2*math.cos(yaw),
                         float(gate['y'])-2*math.sin(yaw),float(gate['z']),yaw]
        self.pref = rospy.Publisher('/desired_state', MultiDOFJointTrajectoryPoint, queue_size=1)
        self.pref_stamped = rospy.Publisher('/desired_state_stamped', MultiDOFJointTrajectory, queue_size=1)
        rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, self.on_reference, queue_size=1)
        rospy.Subscriber('/current_state', Odometry, self.on_state, queue_size=1)
        rospy.Subscriber('/trajectory_status', TrajectoryStatus, self.on_status, queue_size=1)
        rospy.Subscriber('/tesse/collision', CollisionStats, self.on_collision,queue_size=10)
        self.timer = WallReferenceTimer(self.publish_pref)
        rospy.on_shutdown(self.timer.shutdown)

    def on_state(self, m):
        self.state = m

    def on_recording_opened(self,m):
        if self.expected_bag and os.path.abspath(m.data)==self.expected_bag:
            self.recording_opened=True

    def on_prepared_gates(self,m):
        self.prepared_gates=m
        if self.gates_released:
            self.gate_release.publish(m)

    def on_reference(self,m):
        if not self.pref_enabled and m.transforms:
            self.last_mission_reference=m

    def on_status(self, m):
        self.status = m

    def on_collision(self,m):
        if m.is_collision and not self.pref_enabled:
            self.last_collision=m.object_name

    def publish_pref(self, _):
        if not self.pref_enabled:
            return
        m, tr = MultiDOFJointTrajectoryPoint(), Transform()
        tr.translation.x, tr.translation.y, tr.translation.z = self.target[:3]
        tr.rotation.z, tr.rotation.w = math.sin(self.target[3]/2), math.cos(self.target[3]/2)
        m.transforms, m.velocities, m.accelerations = [tr], [Twist()], [Twist()]
        # Keep the completed flight's time during final hover. Publishing zero
        # here would tell RViz that a new flight has begun and erase its path.
        m.time_from_start = self.hold_time_from_start
        self.pref.publish(m)
        stamped=MultiDOFJointTrajectory()
        stamped.header.stamp=rospy.Time.now()
        stamped.header.frame_id='world'
        stamped.joint_names=['base_link_gt']
        stamped.points=[m]
        self.pref_stamped.publish(stamped)

    def spawn(self, command, output):
        return subprocess.Popen(command, stdout=open(output, 'w'), stderr=subprocess.STDOUT,
                                start_new_session=True)

    def stop(self, p):
        if p is not None and p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)

    def prepare(self):
        self.target=list(self.start_target)
        self.hold_time_from_start = rospy.Duration(0)
        self.pref_enabled = True
        rospy.wait_for_service('/tesse/reposition_request', timeout=10)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.target[0], self.target[2], self.target[1]
        pose.orientation.y, pose.orientation.w = -math.sin(self.target[3]/2), math.cos(self.target[3]/2)
        if not rospy.ServiceProxy('/tesse/reposition_request', RepositionRequestService)(pose).success:
            raise RuntimeError('pretrial reposition rejected')
        deadline, stable_since = time.monotonic()+90., None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.state:
                p, v, w = self.state.pose.pose.position, self.state.twist.twist.linear, self.state.twist.twist.angular
                error = math.sqrt((p.x-self.target[0])**2+(p.y-self.target[1])**2+(p.z-self.target[2])**2)
                speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                omega = math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z)
                stable = error < .12 and speed < .10 and omega < .10
                if stable:
                    stable_since = stable_since or rospy.get_time()
                    if rospy.get_time()-stable_since >= (self.options.settle_seconds if self.options else 10.):
                        return
                else:
                    stable_since = None
            time.sleep(.05)
        raise RuntimeError('initial conditions did not settle; no race started')

    def run(self, index):
        run_id = ('terminal_regression_%02d' if self.subset_csv else 'stable_run_%02d') % index
        if self.options:
            run_id=self.options.run_prefix+'_%02d'%index
        directory = os.path.join(self.root, run_id)
        os.makedirs(directory, exist_ok=False)
        self.prepare()
        self.status = None
        self.last_collision = None
        self.prepared_gates = None
        self.gates_released = False
        meta = dict(run_id=run_id, source='new real TESSE closed-loop experiment',
                    gains=[rospy.get_param('/controller_node/'+k) for k in ('kx','kv','kr','komega')],
                    time_scale=self.options.time_scale if self.options else 3., exit_distance_m=1.,
                    post_hold_s=self.options.post_hold, preflight='separate from mission; never reposition during bag')
        meta['collision_telemetry_enabled']=rospy.get_param('/tesse/tesse_ros_bridge/publish_collisions',False)
        meta['simulator_headless']=os.environ.get('LAB4_SIM_HEADLESS','0')=='1'
        meta['clock_rate_modified']=False
        meta['race_exit_observation_s']=self.options.exit_observation
        meta['long_hold_completed']=self.options.exit_observation<=0
        initial = self.state.pose.pose.position
        meta['actual_initial_position_m'] = [initial.x, initial.y, initial.z]
        meta['target_duration_s']=self.options.target_duration if self.options else 0.
        meta['terminal_segment_s']=self.options.terminal_segment if self.options else 0.
        meta['start_mode']='prepared near-ground start' if initial.z < 1.0 else 'prepared airborne start'
        meta['timing']='report TRACKING-to-last-gate and mission-to-last-gate separately; preflight not included'
        subprocess.check_call(['rosparam', 'dump', directory+'/parameters.yaml'])
        processes = []
        try:
            topics = ['/clock','/current_state','/desired_state','/desired_traj_vertices',
                      '/active_race_gates','/desired_state_stamped','/trajectory_status',
                      '/rotor_speed_cmds','/controller_diagnostics',
                      '/tesse/applied_rotor_speed_cmds','/tesse/rotor_watchdog_event','/tesse/collision']
            self.expected_bag=os.path.abspath(directory+'/'+run_id+'.bag')
            self.recording_opened=False
            recorder=self.spawn(['rosbag','record','--publish','--tcpnodelay',
                                 '-O',self.expected_bag]+topics,directory+'/record.log')
            processes.append(recorder)
            deadline=time.monotonic()+30.
            while time.monotonic()<deadline:
                if recorder.poll() is not None:raise RuntimeError('recorder failed before mission')
                if self.recording_opened and self.pref_stamped.get_num_connections()>=2:
                    break
                time.sleep(.02)
            else:raise RuntimeError('recording readiness was not confirmed; no mission released')
            time.sleep(.5)
            print(run_id+' recorder ready; preflight remains sole reference',flush=True)
            planner = self.spawn(['rosrun', 'trajectory_generation', 'trajectory_generation_node',
                                  '/desired_traj_vertices:=/active_race_gates',
                                  '_time_scale:='+str(self.options.time_scale if self.options else 3.),
                                  '_target_duration:='+str(self.options.target_duration if self.options else 0.),
                                  '_terminal_segment_time:='+str(self.options.terminal_segment if self.options else 0.),
                                  '_post_hold_duration:='+str(self.options.post_hold),
                                  '_ready_hold_duration:='+str(self.options.ready_hold if self.options else 5.),
                                  '_publish_frequency:=100.0',
                                  '_run_id:='+run_id], directory+'/planner.log')
            processes.append(planner)
            time.sleep(2.)  # preflight remains the sole reference source
            # Warm the official gate process on a staging topic while the sole
            # preflight reference stays active. Release the unmodified message
            # synchronously at handoff, avoiding rosrun startup in a 0.5 s gap.
            gate_command = ['rosrun','planner_pkg','traj_vertices_publisher',
                            '_simulator_data_directory:='+os.path.expanduser('~/vnav-builds/lab4/lab4_Data'),
                            '_gates_output_csv:='+directory+'/gates.csv']
            gates = self.spawn(gate_command, directory+'/gates.log')
            processes.append(gates)
            self.prepared_gates=rospy.wait_for_message('/desired_traj_vertices',PoseArray,timeout=20.)
            deadline=time.monotonic()+5.
            while self.gate_release.get_num_connections()<2 and time.monotonic()<deadline:
                time.sleep(.02)
            if self.gate_release.get_num_connections()<2:
                raise RuntimeError('planner did not connect to staged gate release')
            if self.status and self.status.run_id==run_id:
                raise RuntimeError('planner ran before authorized gate release; no valid mission')
            meta['gate_handoff']='planner isolated on nonlatched /active_race_gates; original official gate topic unmodified; bag open verified before planning'
            meta['recording_open_confirmed_before_mission']=self.recording_opened
            subprocess.check_call(['rosparam','dump',directory+'/parameters.yaml'])
            self.pref_enabled = False
            self.gates_released = True
            self.gate_release.publish(self.prepared_gates)
            deadline, finished_at = time.monotonic()+900., None
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if planner.poll() is not None or recorder.poll() is not None:
                    raise RuntimeError('planner or recorder exited early')
                if gates.poll() is not None:
                    raise RuntimeError('official gate publisher exited early')
                if self.status and self.status.run_id == run_id:
                    if self.last_collision:
                        raise RuntimeError('simulator collision: '+self.last_collision)
                    if self.status.fault:
                        raise RuntimeError('planner fault: '+self.status.reason)
                    # Racing qualification does not claim a 60-second hover.
                    # Keep the original 75 s generator setting, but end this
                    # recording after reference end + a declared exit window.
                    if (self.options.exit_observation>0 and self.status.reference_complete and
                        self.status.phase in ('POST_RACE_HOLD','FINISHED') and
                        self.status.trajectory_elapsed_s >= self.status.planned_duration_s+self.options.exit_observation):
                        break
                    if self.status.phase == 'FINISHED':
                        finished_at = finished_at or rospy.get_time()
                        if rospy.get_time()-finished_at >= 5.:
                            break
                    if self.status.phase == 'TRACKING' and self.state:
                        p=self.state.pose.pose.position
                        if not (-55<p.x<8 and -18<p.y<24 and .1<p.z<12):
                            raise RuntimeError('physical position safety envelope exceeded')
                time.sleep(.05)
            else:
                raise RuntimeError('trial timed out or interrupted')
            meta['recording_outcome'] = 'reference endpoint observed; physical racing acceptance requires analyzer'
        except Exception as error:
            meta['recording_outcome'] = 'failed: '+str(error)
        finally:
            for process in processes:
                self.stop(process)
            # Continue the exact final reference AFTER the recorder has stopped;
            # do not drop a successfully parked drone while offline analysis runs.
            if self.last_mission_reference:
                self.hold_time_from_start = self.last_mission_reference.time_from_start
                tr=self.last_mission_reference.transforms[0]
                q=tr.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                self.target=[tr.translation.x,tr.translation.y,tr.translation.z,yaw]
                self.pref_enabled=True
            with open(directory+'/run_metadata.json','w') as f:
                json.dump(meta,f,indent=2)
        with open(directory+'/analysis_console.log', 'w') as analysis_log:
            subprocess.check_call(['python3', os.path.join(ANALYSIS_SCRIPTS,'analyze_lab4_bag.py'),
                                   directory+'/'+run_id+'.bag','--output-dir',directory+'/analysis',
                                   '--gate-index-offset',str(self.subset_first if self.subset_csv else 0)],
                                  stdout=analysis_log, stderr=subprocess.STDOUT)
        with open(directory+'/analysis/metrics.json') as f:
            metrics = json.load(f)
        accepted = metrics['actual_stable_full_route']
        if self.subset_csv:
            reasons=metrics['completion_verification']['failure_reasons']
            accepted=(metrics['passed_gates']==12-self.subset_first and
                      reasons==['incomplete sequential gate crossings'])
            with open(directory+'/partial_regression_acceptance.json','w') as f:
                json.dump(dict(partial_regression_passed=accepted,
                               official_full_route=False, original_gate_ids=meta['original_gate_ids']),f,indent=2)
        elif self.options and self.options.target_duration > 0:
            # A long terminal hover must not dilute poor racing precision.
            # Require actual gate crossing AND settled finish, not reference T.
            from check_fast_result import evaluate
            speed_result = evaluate(metrics,self.options.acceptance_limit)
            with open(directory+'/analysis/fast_acceptance.json','w') as f:
                json.dump(speed_result,f,indent=2)
            accepted = accepted and speed_result['passed']
        if self.options.race_scoring:
            from race_acceptance import evaluate_run
            assessment=evaluate_run(directory)
            with open(directory+'/race_acceptance.json','w') as f:
                json.dump(assessment,f,indent=2)
            accepted=assessment['passed']
        if not accepted:
            raise RuntimeError(run_id+' failed acceptance; stop repeated trials for diagnosis')
        if self.options.race_scoring:
            print('%s ACCEPTED | 12/12 gates | %.3f s | minimum clearance %.3f m' %
                  (run_id, assessment['first_gate_to_last_gate_ms']/1000.0,
                   assessment['minimum_clearance_m']), flush=True)
        else:
            print(run_id+' ACCEPTED', flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output-root', required=True)
    p.add_argument('--post-hold',type=float,default=75.)
    p.add_argument('--race-scoring',action='store_true',help='First-to-last gate task; keep old strict checker output separately')
    p.add_argument('--exit-observation',type=float,default=5.,help='Recorded seconds after reference end; 0 retains original long-hold workflow')
    p.add_argument('--count', type=int, default=3)
    p.add_argument('--first-index',type=int,default=1)
    p.add_argument('--subset-csv')
    p.add_argument('--subset-first',type=int,default=9)
    p.add_argument('--target-duration',type=float,default=14.)
    p.add_argument('--time-scale',type=float,default=1.)
    p.add_argument('--ready-hold',type=float,default=.5)
    p.add_argument('--start-height',type=float,default=.7)
    p.add_argument('--settle-seconds',type=float,default=3.)
    p.add_argument('--run-prefix',default='fast_run')
    p.add_argument('--terminal-segment',type=float,default=1.8)
    p.add_argument('--keep-final-hold',action='store_true')
    p.add_argument('--acceptance-limit',type=float,default=15.)
    args = p.parse_args()
    if args.subset_csv:
        p.error('scene recorder wrapper supports the full original course only')
    rospy.init_node('lab4_trial_orchestrator')
    trial = Trials(args.output_root,args.subset_csv,args.subset_first,args)
    for index in range(args.first_index, args.first_index+args.count):
        trial.run(index)
    if args.keep_final_hold:
        print('All trials complete. Keeping the exact terminal setpoint active until Ctrl-C.',flush=True)
        rospy.spin()
