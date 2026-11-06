import urllib.request,concurrent.futures
from pathlib import Path
p=Path.home()/'lab5_complete/data'
base='https://www.dropbox.com/scl/fo/pvme4alm9cjckwloll2tu/AHCpSmA00hS3WSF4HDpXxIg?rlkey=xpqg2fiwuyq5tejd7kxrnfe0k&dl=1&preview=lab5%2F'
def f(name):
 with urllib.request.urlopen(base+name,timeout=120) as r,(p/(name+'.part')).open('wb') as dst:
  n=0;total=r.headers.get('Content-Length');print(name,total,flush=True)
  while True:
   data=r.read(4*1024*1024)
   if not data:break
   dst.write(data);n+=len(data)
   if n%(64*1024*1024)==0:print(name,n,flush=True)
 (p/(name+'.part')).rename(p/name);print(name,'READY',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:list(ex.map(f,['30fps_424x240_2018-10-01-18-35-06.bag','vnav-lab5-smooth-trajectory.bag']))
