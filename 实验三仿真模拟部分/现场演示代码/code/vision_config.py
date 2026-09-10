# -*- coding: utf-8 -*-
"""
vision_config.py —— 视觉模块公共配置（相机 / 网格 / 类别 / 料盒 / 消息总线）
"""
import os

# ---------------- 仿真相机 ----------------
# 说明：相机放在较高处并使用小视场角（长焦/近似正交投影），
#       这样俯视时几乎看不到物体侧面，圆度才能稳定区分“棱柱(方)/圆柱(圆)”。
CAM_POS = (0.38, 0.0, 12.00)         # 相机位置（正上方俯视物体行）
CAM_TARGET = (0.38, 0.0, 0.00)       # 光轴朝向地面
CAM_UP = (0.0, 1.0, 0.0)             # 俯视时的“画面上方”= 世界 +Y
CAM_FOV = 12.0                       # 视场角（度）→ 覆盖约 2.5m
CAM_W, CAM_H = 640, 360              # 图像分辨率（小分辨率降低渲染负担）
CAM_NEAR, CAM_FAR = 0.05, 30.0       # 裁剪面（必须够远才能看到地面）
CAM_NAME = "vision_cam"

# ---------------- 网格（物体行） ----------------
GRID_X = 0.380                       # 物体行所在的 x
GRID_Y0 = -0.5                       # 位置 1 的 y
GRID_DY = 0.20                       # 相邻位置间距
GRID_N = 6
GRID_Y_TOL = 0.09                    # 判定属于某个网格的 y 容差
ROW_X_TOL = 0.09                     # 判定属于物体行的 x 容差
OBJ_TOP_Z = 0.22                     # 物体顶面高度（反投影校正用）

# ---------------- 类别 ----------------
CLASS_A, CLASS_B = "A", "B"
CLASS_NAME = {CLASS_A: "A类・蓝色棱柱", CLASS_B: "B类・红色圆柱"}
CLASS_BIN = {CLASS_A: "LeftBin", CLASS_B: "RightBin"}     # A→左料盒, B→右料盒
CLASS_COLOR = {CLASS_A: (0.10, 0.35, 0.90), CLASS_B: (0.90, 0.18, 0.18)}

# ---------------- 料盒与投放点 ----------------
BIN_Y = {"LeftBin": -0.90, "RightBin": 0.90}
SLOT_X = 0.380
SLOT_DY = 0.15
SLOT_Z = 0.13

# ---------------- HSV 颜色阈值（OpenCV: H 0~179） ----------------
HSV_BLUE = [((95, 90, 60), (135, 255, 255))]
HSV_RED = [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))]

MIN_AREA = 60                        # 最小连通域面积（像素，640x360 下物体约 15x15px）
CIRC_ROUND = 0.88                    # 圆度阈值：> 视为圆柱(圆)，< 视为棱柱(方)

# ---------------- 消息总线（模仿 发布/订阅） ----------------
BUS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision_bus")
FRAME_DIR = os.path.join(BUS_DIR, "frames")
TOPIC_LATEST = os.path.join(BUS_DIR, "latest.jpg")
TOPIC_ANNOTATED = os.path.join(BUS_DIR, "annotated_latest.jpg")
TOPIC_CAM_META = os.path.join(BUS_DIR, "camera_meta.json")
TOPIC_DETECTIONS = os.path.join(BUS_DIR, "detections.json")
CALIB_PATH = os.path.join(BUS_DIR, "calibration.json")
DESKTOP_ANNOTATED = r"C:\Users\39562\Desktop\实验三_视觉识别结果.jpg"


def ensure_bus():
    os.makedirs(FRAME_DIR, exist_ok=True)
    return BUS_DIR
