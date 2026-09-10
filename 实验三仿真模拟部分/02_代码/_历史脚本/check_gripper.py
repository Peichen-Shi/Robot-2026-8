from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject("sim")

gripper = sim.getObject(
    "/RoboMaster/arm_base_link_respondable/gripper_link_respondable"
)

box = sim.getObject("/Box")

gripper_pos = sim.getObjectPosition(gripper, -1)
box_pos = sim.getObjectPosition(box, -1)

print("========== 当前末端位置 ==========")
print("Gripper:")
print("X =", gripper_pos[0])
print("Y =", gripper_pos[1])
print("Z =", gripper_pos[2])

print()
print("Box:")
print("X =", box_pos[0])
print("Y =", box_pos[1])
print("Z =", box_pos[2])

print()
print("高度差 Z =", gripper_pos[2] - box_pos[2])