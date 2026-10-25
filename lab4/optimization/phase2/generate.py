from pathlib import Path
import yaml,json
root=Path.home()/'lab4_sub7_20261025'
folder=root/'optimization/phase2';folder.mkdir(exist_ok=True)
baseline=yaml.safe_load((folder/'baseline_695_controller.yaml').read_text())
times=yaml.safe_load((folder/'baseline_695_timing.yaml').read_text())['segment_durations']
profiles={
 'stock':dict(baseline),
 'response32':dict(baseline,kr=32.,komega=4.8,maximum_thrust=80.,maximum_moment_x=16.,maximum_moment_y=16.,maximum_moment_z=10.,feedforward_filter_tau=.015),
 'response40':dict(baseline,kx=22.,kv=10.,kr=40.,komega=5.5,maximum_thrust=90.,maximum_moment_x=20.,maximum_moment_y=20.,maximum_moment_z=12.,feedforward_filter_tau=.015),
}
for name,profile in profiles.items():(folder/(name+'.yaml')).write_text(yaml.safe_dump(profile,sort_keys=False))
for target in [6.4,6.1,5.95]:
 for protected in [0,3]:
  selected=list(times)
  indices=list(range(1+protected,12))
  remaining=target-sum(selected[1:1+protected])
  factor=remaining/sum(selected[i] for i in indices)
  for i in indices:selected[i]*=factor
  name=('uniform' if protected==0 else 'protect3')+'_'+str(target)
  (folder/(name+'.yaml')).write_text(yaml.safe_dump({'segment_durations':selected},sort_keys=False))
manifest=[]
for schedule,profile in [
 ('protect3_6.4','stock'),('uniform_6.4','response32'),
 ('protect3_6.4','response32'),('uniform_6.4','response40'),
 ('uniform_6.1','response32'),('protect3_6.1','response40')]:
 durations=yaml.safe_load((folder/(schedule+'.yaml')).read_text())['segment_durations']
 manifest.append(dict(prefix='phase2_'+schedule+'_'+profile,schedule=schedule,profile=profile,total=sum(durations)))
(folder/'batch.json').write_text(json.dumps(manifest,indent=2))
for trial in manifest:print(trial['prefix'],trial['total'])
