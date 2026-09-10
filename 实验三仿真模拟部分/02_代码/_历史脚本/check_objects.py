from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject("sim")

print("连接成功")
print()
print("场景中的物体：")
print("==============================")

handles = sim.getObjectsInTree(
    sim.handle_scene,
    sim.handle_all,
    0
)

for h in handles:
    try:
        name = sim.getObjectAlias(h, -1)
        print(h, ":", name)
    except:
        pass

print("==============================")
print("完成")