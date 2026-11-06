from pathlib import Path
import csv,json,math,hashlib
base=Path(__file__).parent;root=base.parent if base.name=='scripts' else base/'submission';checks=[]
for dataset in ['pair','dataset1','dataset2']:
 for method in ['SIFT','SURF','ORB','FAST_BRIEF']+([] if dataset=='pair' else ['LK']):
  folder=root/'results'/dataset
  summary=next(csv.DictReader((folder/f'{method}_summary.csv').open()))
  rows=list(csv.DictReader((folder/f'{method}_frames.csv').open()))
  assert len(rows)==int(summary['pairs'])>0,(dataset,method,'pair count')
  expected=1 if dataset=='pair' else None
  if expected:assert len(rows)==expected
  for row in rows:
   vals=[float(row[k]) for k in ['keypoints1','keypoints2','matches','good','inliers','inlier_ratio','ms']]
   assert all(math.isfinite(v) and v>=0 for v in vals)
   assert vals[4]<=vals[3]<=vals[2] and vals[5]<=1
   rate=vals[4]/vals[3] if vals[3] else 0
   assert abs(rate-vals[5])<1e-6
  for k in ['keypoints1','keypoints2','matches','good','inliers','inlier_ratio','ms']:
   mean=sum(float(row[k]) for row in rows)/len(rows)
   assert abs(mean-float(summary[k]))<max(.002,abs(mean)*1e-5),(dataset,method,k,mean,summary[k])
  checks.append(dict(dataset=dataset,method=method,pairs=len(rows),valid=True))
 for expected_file in ([] if dataset=='pair' else ['LK_tracks.png','Farneback.png']):assert (root/'results'/dataset/expected_file).exists()
assert len({x['pairs'] for x in checks if x['dataset']=='dataset1'})==1
assert len({x['pairs'] for x in checks if x['dataset']=='dataset2'})==1
assert next(x['pairs'] for x in checks if x['dataset']=='dataset1')==1583
assert next(x['pairs'] for x in checks if x['dataset']=='dataset2')==1385
(root/'validation.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
from PIL import Image,ImageStat
files=[]
for f in sorted((root/'screenshots').glob('*.png')):
 stats=ImageStat.Stat(Image.open(f).convert('RGB'));assert max(stats.stddev)>10,('blank screenshot',f.name)
 files.append({'filename':f.name,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()})
with (root/'screenshots'/'截图编号索引.csv').open('w',newline='',encoding='utf-8-sig') as f:
 w=csv.DictWriter(f,fieldnames=['filename','sha256']);w.writeheader();w.writerows(files)
assert {int(x['filename'].split('_')[0]) for x in files}==set(range(1,18))
print(json.dumps(checks,indent=2));print('VERIFIED_SCREENSHOTS',len(files))
