from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


class RoboMasterEPSim:

    def __init__(self):
        print("连接 CoppeliaSim...")

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.FL = self.sim.getObject(
            "/RoboMaster/front_left_wheel_joint"
        )
        self.FR = self.sim.getObject(
            "/RoboMaster/front_right_wheel_joint"
        )
        self.RL = self.sim.getObject(
            "/RoboMaster/rear_left_wheel_joint"
        )
        self.RR = self.sim.getObject(
            "/RoboMaster/rear_right_wheel_joint"
        )

        print("RoboMaster EP 连接成功")

    def set_wheels(self, fl, fr, rl, rr):
        self.sim.setJointTargetVelocity(self.FL, fl)
        self.sim.setJointTargetVelocity(self.FR, fr)
        self.sim.setJointTargetVelocity(self.RL, rl)
        self.sim.setJointTargetVelocity(self.RR, rr)

    def stop(self):
        self.set_wheels(0, 0, 0, 0)

    def forward(self, speed=2.0):
        self.set_wheels(
            speed,
            speed,
            speed,
            speed
        )

    def backward(self, speed=2.0):
        self.set_wheels(
            -speed,
            -speed,
            -speed,
            -speed
        )

    def left(self, speed=2.0):
        self.set_wheels(
            -speed,
            speed,
            speed,
            -speed
        )

    def right(self, speed=2.0):
        self.set_wheels(
            speed,
            -speed,
            -speed,
            speed
        )

    def rotate_left(self, speed=2.0):
        self.set_wheels(
            -speed,
            speed,
            -speed,
            speed
        )

    def rotate_right(self, speed=2.0):
        self.set_wheels(
            speed,
            -speed,
            speed,
            -speed
        )

    def move_forward(self, seconds=1.0, speed=2.0):
        self.forward(speed)
        time.sleep(seconds)
        self.stop()

    def move_backward(self, seconds=1.0, speed=2.0):
        self.backward(speed)
        time.sleep(seconds)
        self.stop()

    def move_left(self, seconds=1.0, speed=2.0):
        self.left(speed)
        time.sleep(seconds)
        self.stop()

    def move_right(self, seconds=1.0, speed=2.0):
        self.right(speed)
        time.sleep(seconds)
        self.stop()

    def rotate(self, seconds=1.0, speed=2.0):
        self.rotate_right(speed)
        time.sleep(seconds)
        self.stop()