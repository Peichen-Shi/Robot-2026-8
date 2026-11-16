#!/usr/bin/env python3
"""Repair the ROS2 lowercase CameraInfo names after rosbags 0.9 conversion.
Wire field layout is identical; only ROS1 definition and MD5 change.
The input ROS2 dataset is never modified.
"""
import mmap,sys
path=sys.argv[1]
replacements=[(b'b5aff04250fbf270b71bef9d82c2132e',b'c9a58c1b0b154e0e6da7578cb991d214'),
 (b'float64[] d\nfloat64[9] k\nfloat64[9] r\nfloat64[12] p',b'float64[] D\nfloat64[9] K\nfloat64[9] R\nfloat64[12] P')]
with open(path,'r+b') as f:
 with mmap.mmap(f.fileno(),0) as m:
  for before,after in replacements:
   assert len(before)==len(after)
   start=0;count=0
   while True:
    pos=m.find(before,start)
    if pos<0:break
    m[pos:pos+len(before)]=after;start=pos+len(before);count+=1
   print('Repaired',count,'occurrences of',before[:32])
  m.flush()
