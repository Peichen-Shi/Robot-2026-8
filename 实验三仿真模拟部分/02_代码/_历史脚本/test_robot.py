from robot_sim import RoboMasterEPSim
import time


robot = RoboMasterEPSim()

print("前进")
robot.move_forward(1.0, 2.0)

time.sleep(0.5)

print("后退")
robot.move_backward(1.0, 2.0)

time.sleep(0.5)

print("左移")
robot.move_left(1.0, 2.0)

time.sleep(0.5)

print("右移")
robot.move_right(1.0, 2.0)

time.sleep(0.5)

print("测试完成")