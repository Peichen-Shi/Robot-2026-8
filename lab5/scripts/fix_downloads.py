import concurrent.futures,urllib.request
from pathlib import Path
p=Path.home()/'lab5_complete/sources/build/downloads/xfeatures2d';p.mkdir(parents=True,exist_ok=True)
jobs=[('34e4206aef44d50e6bbcd0ab06354b52e7466d26',f'boostdesc_{n}.i') for n in ['bgm','bgm_bi','bgm_hd','binboost_064','binboost_128','binboost_256','lbgm']]+[('fccf7cd6a4b12079f73bbfb21745f9babcd4eb1d',f'vgg_generated_{n}.i') for n in [48,64,80,120]]
def get(x):
 commit,name=x
 for attempt in range(3):
  try:
   data=urllib.request.urlopen(f'https://raw.githubusercontent.com/opencv/opencv_3rdparty/{commit}/{name}',timeout=30).read();(p/name).write_bytes(data);return name,len(data)
  except Exception as e:print(e,flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:print(list(pool.map(get,jobs)),flush=True)
