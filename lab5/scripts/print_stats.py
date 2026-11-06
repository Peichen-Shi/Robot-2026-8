import csv,sys
from pathlib import Path
root=Path.home()/'lab5_complete';d=sys.argv[1]
print('LAB5 FULL SEQUENCE STATISTICS:',d)
print('C++ OpenCV 4.2 NONFREE | consecutive frames | all frame pairs')
info=root/'logs'/(d+'_info.txt')
if info.exists():
 for line in info.read_text().splitlines():
  if line.startswith(('path:','duration:','start:','end:','size:','messages:')):print(line)
log=root/'logs'/(d+'_SIFT.log')
if log.exists(): print(log.read_text().splitlines()[0])
print('\nmethod        pairs    kp1      kp2      matches   good     inliers   ratio%   ms')
for name in ['SIFT','SURF','ORB','FAST_BRIEF','LK']:
 row=next(csv.DictReader((root/'results'/d/(name+'_summary.csv')).open()))
 vals=[float(row[k]) for k in ['keypoints1','keypoints2','matches','good','inliers']]
 print(f'{name:13s} {int(row["pairs"]):5d} '+''.join(f'{x:9.2f}' for x in vals)+f'{float(row["inlier_ratio"])*100:9.2f}{float(row["ms"]):9.2f}')
print('\nInlier ratio = per-pair inliers/good, then arithmetic mean.')
print('LK good = valid status and in-bounds tracks; no ratio filter.')
print('All raw measurements are in results/'+d+'/*_frames.csv')
