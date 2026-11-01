#!/usr/bin/env python3
"""Real closed-loop isolation references; never synthesizes actual state."""
import math
import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


class Reference:
    def __init__(self):
        self.initial = None
        self.start = None
        self.mode = rospy.get_param('~mode', 'yaw_sweep')
        self.pub = rospy.Publisher('/desired_state', MultiDOFJointTrajectoryPoint, queue_size=1)
        self.sub = rospy.Subscriber('/current_state', Odometry, self.state, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.02), self.tick)

    def state(self, m):
        if self.initial is None:
            self.initial = m.pose.pose
            self.initial.position.x = rospy.get_param('~target_x', self.initial.position.x)
            self.initial.position.y = rospy.get_param('~target_y', self.initial.position.y)
            self.initial.position.z = rospy.get_param('~target_z', self.initial.position.z)
            q = self.initial.orientation
            self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.yaw = rospy.get_param('~target_yaw', self.yaw)
            self.start = rospy.Time.now()

    def tick(self, _):
        if self.initial is None:
            return
        t = (rospy.Time.now()-self.start).to_sec()
        if t < 0:
            rospy.signal_shutdown('clock rollback invalidates diagnostic run')
            return
        yaw, rate, dx, vx, ax = self.yaw, 0., 0., 0., 0.
        if self.mode == 'yaw_sweep':
            # Continuous unwrapped 0 -> 170 -> 190 -> 360 degrees: crosses pi.
            knots = [(10., 35., 0., 170.), (45., 60., 170., 190.),
                     (70., 95., 190., 360.)]
            offset = 0.
            for begin, end, first, last in knots:
                if t >= end:
                    offset = math.radians(last)
                elif t >= begin:
                    u = (t-begin)/(end-begin)
                    s = 10*u**3-15*u**4+6*u**5
                    ds = (30*u**2-60*u**3+30*u**4)/(end-begin)
                    offset = math.radians(first+(last-first)*s)
                    rate = math.radians(last-first)*ds
                    break
            yaw += offset
        elif self.mode == 'fixed_yaw_position' and 10. <= t <= 90.:
            u = (t-10.)/80.
            dx = .5*(1-math.cos(2*math.pi*u))
            vx = math.pi/80*math.sin(2*math.pi*u)
            ax = 2*math.pi**2/80**2*math.cos(2*math.pi*u)
        m = MultiDOFJointTrajectoryPoint()
        tr = Transform()
        tr.translation.x = self.initial.position.x+dx
        tr.translation.y = self.initial.position.y
        tr.translation.z = self.initial.position.z
        tr.rotation.z, tr.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        v, a = Twist(), Twist()
        v.linear.x, v.angular.z, a.linear.x = vx, rate, ax
        m.transforms, m.velocities, m.accelerations = [tr], [v], [a]
        m.time_from_start = rospy.Duration(t)
        self.pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('lab4_diagnostic_reference')
    reference = Reference()
    rospy.spin()
