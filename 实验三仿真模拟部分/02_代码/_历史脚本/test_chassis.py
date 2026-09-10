from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

client = RemoteAPIClient()
sim = client.require("sim")

print("连接 CoppeliaSim 成功")

FL = sim.getObject("/RoboMaster/front_left_wheel_joint")
FR = sim.getObject("/RoboMaster/front_right_wheel_joint")
RL = sim.getObject("/RoboMaster/rear_left_wheel_joint")
RR = sim.getObject("/RoboMaster/rear_right_wheel_joint")


def set_wheels(fl, fr, rl, rr):
    sim.setJointTargetVelocity(FL, fl)
    sim.setJointTargetVelocity(FR, fr)
    sim.setJointTargetVelocity(RL, rl)
    sim.setJointTargetVelocity(RR, rr)


def stop():
    set_wheels(0, 0, 0, 0)


speed = 2.0
duration = 2.0


tests = [
    ("测试A",  speed,  speed,  speed,  speed),
    ("测试B",  speed, -speed, -speed,  speed),
    ("测试C",  speed,  speed, -speed, -speed),
    ("测试D",  speed, -speed,  speed, -speed),
]


for name, fl, fr, rl, rr in tests:

    print()
    print("=" * 40)
    print(name)
    print(
        f"FL={fl}, FR={fr}, "
        f"RL={rl}, RR={rr}"
    )
    print("=" * 40)

    set_wheels(fl, fr, rl, rr)

    time.sleep(duration)

    stop()

    print(name, "结束")

    time.sleep(1)


print()
print("全部测试完成")