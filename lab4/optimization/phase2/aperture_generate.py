from pathlib import Path
import csv,json,math,yaml,numpy as np
root=Path.home()/'lab4_sub7_20261025'
folder=root/'optimization/phase2'
gates=list(csv.DictReader((root/'optimization/official_gates.csv').open()))
p=np.array([[float(g[k]) for k in ('x','y','z')] for g in gates])
yaw=np.array([float(g['yaw_raw_rad']) for g in gates])
tangent=np.stack([-np.sin(yaw),np.cos(yaw),np.zeros(len(gates))],axis=1)
start=np.array([.4,8.,2.67])
def lateral_offsets(vertical,limit):
 selected=p+np.array([0.,0.,1.])*vertical[:,None]
 lateral=np.zeros(len(gates))
 for _ in range(800):
  current=selected+tangent*lateral[:,None]
  delta=np.diff(np.vstack([start,current]),axis=0)
  unit=delta/np.linalg.norm(delta,axis=1)[:,None]
  gradient=unit.copy();gradient[:-1]-=unit[1:]
  gradient=np.sum(gradient*tangent,axis=1)+.04*lateral
  lateral=np.clip(lateral-.35*gradient,-limit,limit)
 return lateral.tolist()
stock=yaml.safe_load((folder/'stock_prediction.yaml').read_text())
profiles={
 'ap_stock':dict(stock),
 'ap_torque':dict(stock,maximum_thrust=75.,maximum_moment_x=12.,maximum_moment_y=12.,maximum_moment_z=7.),
 'ap_balanced':dict(stock,kx=18.,kv=8.5,kr=28.,komega=4.4,maximum_thrust=80.,maximum_moment_x=14.,maximum_moment_y=14.,maximum_moment_z=8.),
 'ap_no_prediction':dict(stock,reference_prediction=False,maximum_thrust=75.,maximum_moment_x=12.,maximum_moment_y=12.,maximum_moment_z=7.),
}
for name,profile in profiles.items():(folder/(name+'.yaml')).write_text(yaml.safe_dump(profile,sort_keys=False))
manifest=[]
for target,shift,lat,profile in [
 (6.4,.55,0.,'ap_stock'),(6.1,.55,0.,'ap_torque'),
 (6.1,.55,.35,'ap_torque'),(5.95,.55,.35,'ap_balanced'),
 (6.1,.55,.35,'ap_no_prediction'),(6.1,.65,.35,'ap_stock')]:
 vertical=np.clip(2.8-p[:,2],-shift,shift)
 lateral=lateral_offsets(vertical,lat)
 schedule=yaml.safe_load((folder/('uniform_'+str(target)+'.yaml')).read_text())
 schedule.update(gate_lateral_offsets=lateral,gate_vertical_offsets=vertical.tolist())
 name='ap_T%s_z%s_y%s'%(target,shift,lat)
 (folder/(name+'.yaml')).write_text(yaml.safe_dump(schedule,sort_keys=False))
 manifest.append(dict(prefix='aperture_'+name+'_'+profile,schedule=name,profile=profile,total=sum(schedule['segment_durations'])))
(folder/'aperture_batch.json').write_text(json.dumps(manifest,indent=2))
with (folder/'aperture_batch.tsv').open('w') as f:
 for t in manifest:f.write('\t'.join(map(str,[t['prefix'],t['schedule'],t['profile'],t['total']]))+'\n')
print('Generated aperture targets for original physical gates')
