"""Read actual diagnostic bag messages without altering the simulator."""
import json
import sys
from pathlib import Path
import numpy as np
import rosbag

directory = Path(sys.argv[1])
meta = json.loads((directory/'run_metadata.json').read_text())
rows = []
with rosbag.Bag(str(directory/(meta['run_id']+'.bag'))) as bag:
    for _, m, _ in bag.read_messages(topics=['/controller_diagnostics']):
        if m.task_phase_name != 'TRACKING':
            continue
        vec = lambda v: np.array([v.x, v.y, v.z])
        rows.append([m.header.stamp.to_sec(), np.linalg.norm(vec(m.position_error)),
                     np.linalg.norm(vec(m.velocity)), np.linalg.norm(vec(m.body_angular_velocity)),
                     m.thrust_command, m.thrust_unconstrained,
                     float(m.force_limited), float(m.moment_limited),
                     float(m.control_fault), *vec(m.moment_unconstrained)])
if rows:
    a = np.array(rows)
    print(json.dumps(dict(run=meta['run_id'],samples=len(rows),
         position_error_peak_m=float(a[:,1].max()),position_error_p95_m=float(np.percentile(a[:,1],95)),
         speed_peak_mps=float(a[:,2].max()),body_rate_peak_rad_s=float(a[:,3].max()),
         thrust_peak_N=float(a[:,4].max()),requested_thrust_peak_N=float(a[:,5].max()),
         force_limited_fraction=float(a[:,6].mean()),moment_limited_fraction=float(a[:,7].mean()),
         faults=int(a[:,8].sum()),requested_moment_abs_peak_Nm=np.abs(a[:,9:12]).max(axis=0).tolist()),indent=2))
