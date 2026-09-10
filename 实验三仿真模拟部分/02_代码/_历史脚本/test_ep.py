from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("正在连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.require("sim")

print("连接成功！")
print("Simulation state:", sim.getSimulationState())

# 获取场景中的所有对象
objects = sim.getObjectsInTree(
    sim.handle_scene,
    sim.handle_all,
    0
)

print(f"\n场景中共有 {len(objects)} 个对象")
print("=" * 60)

for handle in objects:
    try:
        name = sim.getObjectAlias(handle, 1)
        print(f"{handle:5d}  {name}")
    except:
        pass

print("=" * 60)