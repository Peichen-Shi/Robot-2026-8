from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os


# ============================================================
# 保存当前 CoppeliaSim 场景
# ============================================================

print()
print("==============================================")
print("       保存当前 CoppeliaSim 仿真场景")
print("==============================================")
print()

# 连接 CoppeliaSim
print("连接 CoppeliaSim...")

client = RemoteAPIClient()
sim = client.getObject("sim")

print("连接成功")
print()


# ============================================================
# 保存路径
# ============================================================

save_path = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\scene_6grid.ttt"


# ============================================================
# 保存当前场景
# ============================================================

print("正在保存场景...")

try:

    sim.saveScene(save_path)

    print()
    print("==============================================")
    print("              保存成功！")
    print("==============================================")
    print()
    print("文件：")
    print(save_path)
    print()

    if os.path.exists(save_path):

        size = os.path.getsize(save_path)

        print(
            "文件大小：%.2f MB"
            % (size / 1024 / 1024)
        )

    print()
    print("当前建模状态已经保存。")

except Exception as e:

    print()
    print("保存失败！")
    print("错误：", e)


print()
input("按 Enter 退出...")