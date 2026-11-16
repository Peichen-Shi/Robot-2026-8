#!/usr/bin/env python3
"""Show one-step, ground-truth-anchored pose estimates as short RViz paths."""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
rospy.init_node('lab6_paths')
publishers={}; paths={}
def callback(pose,name):
    path=paths[name];path.header=pose.header;path.poses.append(pose);path.poses=path.poses[-1000:]
    publishers[name].publish(path)
for name in ['5pt','8pt','2pt','arun','gt']:
    paths[name]=Path()
    publishers[name]=rospy.Publisher('/lab6/path_'+name,Path,queue_size=1,latch=True)
    rospy.Subscriber('/gt_camera_pose' if name=='gt' else '/lab6/pose_'+name,PoseStamped,callback,callback_args=name,queue_size=10)
rospy.spin()
