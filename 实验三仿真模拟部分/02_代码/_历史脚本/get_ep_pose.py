from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.getObject("sim")

robot = sim.getObject("/RoboMaster")
box = sim.getObject("/Box")

robot_pos = sim.getObjectPosition(robot, -1)
robot_ori = sim.getObjectOrientation(robot, -1)

box_pos = sim.getObjectPosition(box, -1)

print()
print("========== EP ==========")
print("位置:")
print("X =", robot_pos[0])
print("Y =", robot_pos[1])
print("Z =", robot_pos[2])

print("姿态:")
print("A =", robot_ori[0])
print("B =", robot_ori[1])
print("G =", robot_ori[2])

print()
print("========== Box ==========")
print("X =", box_pos[0])
print("Y =", box_pos[1])
print("Z =", box_pos[2])

print()
print("读取完成")