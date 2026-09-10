from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# ============================================================
# RoboMaster EP
# 实验三：桌面六网格自动分类整理
#
# 最终布局：
#
# LeftBin  Grid1  Grid2  Grid3  Grid4  Grid5  Grid6  RightBin
#
# Y：
# -0.9     -0.5   -0.3   -0.1    0.1    0.3    0.5     0.9
#
# Grid1~3：蓝色四棱柱
# Grid4~6：红色圆柱
#
# 四棱柱底边 = 0.16 m
# 圆柱直径 = 0.16 m
# 物体高度 = 0.22 m
#
# RoboMaster 初始位置 = (0, 0, 0)
# ============================================================


print()
print("==============================================")
print("       RoboMaster EP 六网格场景初始化")
print("==============================================")
print()


# ============================================================
# 1. 连接 CoppeliaSim
# ============================================================

print("连接 CoppeliaSim...")

client = RemoteAPIClient()

sim = client.getObject("sim")

print("连接成功")
print()


# ============================================================
# 2. 获取 RoboMaster
# ============================================================

print("获取 RoboMaster...")

robot = sim.getObject("/RoboMaster")

print("RoboMaster 获取成功")
print()


# ============================================================
# 3. 设置小车初始位置
# ============================================================

sim.setObjectPosition(
    robot,
    -1,
    [0.0, 0.0, 0.0]
)

print("RoboMaster 初始位置：")
print("X = 0.000")
print("Y = 0.000")
print("Z = 0.000")
print()


# ============================================================
# 4. 获取原来的 Box
# ============================================================

print("获取原来的 Box...")

try:

    box = sim.getObject("/Box")

    print("Box 获取成功")

except Exception:

    box = None

    print("没有找到 /Box")


print()


# ============================================================
# 5. 六个物体尺寸
# ============================================================

# 所有物体统一 X
OBJECT_X = 0.38

# 物体高度
OBJECT_HEIGHT = 0.22

# 物体中心高度
# 底部 Z=0
OBJECT_Z = OBJECT_HEIGHT / 2


# ------------------------------------------------------------
# 统一尺寸：
#
# 四棱柱底边 = 0.16m
# 圆柱直径 = 0.16m
# ------------------------------------------------------------

OBJECT_DIAMETER = 0.12

# 四棱柱
BOX_SIZE = OBJECT_DIAMETER / 2

# 圆柱
CYLINDER_RADIUS = OBJECT_DIAMETER / 2


# ============================================================
# 6. 删除之前创建的 Grid 和料盒
# ============================================================

print("删除旧实验物体...")

old_objects = [

    "/Grid1",
    "/Grid2",
    "/Grid3",
    "/Grid4",
    "/Grid5",
    "/Grid6",

    "/LeftBin",
    "/RightBin",

    "/LeftBin_bottom",
    "/LeftBin_front",
    "/LeftBin_back",
    "/LeftBin_left",
    "/LeftBin_right",

    "/RightBin_bottom",
    "/RightBin_front",
    "/RightBin_back",
    "/RightBin_left",
    "/RightBin_right"

]


for name in old_objects:

    try:

        obj = sim.getObject(name)

        sim.removeObject(obj)

        print("删除:", name)

    except Exception:

        pass


print()


# ============================================================
# 7. 创建蓝色四棱柱
# ============================================================

def create_box(name, x, y, z):

    obj = sim.createPrimitiveShape(

        sim.primitiveshape_cuboid,

        [
            BOX_SIZE,
            BOX_SIZE,
            OBJECT_HEIGHT
        ],

        0

    )


    # 设置名称
    sim.setObjectAlias(
        obj,
        name,
        1
    )


    # 设置位置
    sim.setObjectPosition(

        obj,
        -1,

        [
            x,
            y,
            z
        ]

    )


    # --------------------------------------------------------
    # 设置蓝色
    #
    # RGB：
    # R = 0
    # G = 0
    # B = 1
    # --------------------------------------------------------

    sim.setShapeColor(

        obj,

        "",

        sim.colorcomponent_ambient_diffuse,

        [
            0.0,
            0.0,
            1.0
        ]

    )


    return obj


# ============================================================
# 8. 创建红色圆柱
# ============================================================

def create_cylinder(name, x, y, z):

    obj = sim.createPrimitiveShape(

        sim.primitiveshape_cylinder,

        [
            CYLINDER_RADIUS,
            CYLINDER_RADIUS,
            OBJECT_HEIGHT
        ],

        0

    )


    # 设置名称
    sim.setObjectAlias(
        obj,
        name,
        1
    )


    # 设置位置
    sim.setObjectPosition(

        obj,
        -1,

        [
            x,
            y,
            z
        ]

    )


    # --------------------------------------------------------
    # 设置红色
    #
    # RGB：
    # R = 1
    # G = 0
    # B = 0
    # --------------------------------------------------------

    sim.setShapeColor(

        obj,

        "",

        sim.colorcomponent_ambient_diffuse,

        [
            1.0,
            0.0,
            0.0
        ]

    )


    return obj


# ============================================================
# 9. 创建六个物体
# ============================================================

print("==============================================")
print("              创建六个 Grid")
print("==============================================")
print()


grid_data = [

    # 名称      Y       形状          类别

    ("Grid1",  -0.5,   "BOX",        "A"),

    ("Grid2",  -0.3,   "BOX",        "A"),

    ("Grid3",  -0.1,   "BOX",        "A"),

    ("Grid4",   0.1,   "CYLINDER",   "B"),

    ("Grid5",   0.3,   "CYLINDER",   "B"),

    ("Grid6",   0.5,   "CYLINDER",   "B"),

]


for name, y, shape_type, category in grid_data:

    print("----------------------------------------------")

    print(name)

    print(
        "X = %.3f m"
        % OBJECT_X
    )

    print(
        "Y = %.3f m"
        % y
    )

    print(
        "Z = %.3f m"
        % OBJECT_Z
    )

    print(
        "高度 = %.3f m"
        % OBJECT_HEIGHT
    )

    print(
        "尺寸 = %.3f m"
        % OBJECT_DIAMETER
    )

    print(
        "类别 = %s"
        % category
    )


    # --------------------------------------------------------
    # Grid1~3：蓝色四棱柱
    # --------------------------------------------------------

    if shape_type == "BOX":

        create_box(

            name,

            OBJECT_X,

            y,

            OBJECT_Z

        )

        print("形状 = 四棱柱")

        print("颜色 = 蓝色")

        print(
            "底边 = %.2f × %.2f m"
            % (
                BOX_SIZE,
                BOX_SIZE
            )
        )


    # --------------------------------------------------------
    # Grid4~6：红色圆柱
    # --------------------------------------------------------

    else:

        create_cylinder(

            name,

            OBJECT_X,

            y,

            OBJECT_Z

        )

        print("形状 = 圆柱")

        print("颜色 = 红色")

        print(
            "直径 = %.2f m"
            % OBJECT_DIAMETER
        )

        print(
            "半径 = %.2f m"
            % CYLINDER_RADIUS
        )


    print()


# ============================================================
# 10. 创建空心料盒
# ============================================================

def create_bin(name, x, y):

    # ========================================================
    # 料盒尺寸
    # ========================================================

    WIDTH = 0.45

    DEPTH = 0.45

    HEIGHT = 0.08

    WALL = 0.02

    BOTTOM = 0.02


    # ========================================================
    # 底板
    #
    # 底面 Z = 0
    # 不悬空
    # ========================================================

    bottom = sim.createPrimitiveShape(

        sim.primitiveshape_cuboid,

        [
            WIDTH,
            DEPTH,
            BOTTOM
        ],

        0

    )


    sim.setObjectAlias(

        bottom,

        name + "_bottom",

        1

    )


    sim.setObjectPosition(

        bottom,

        -1,

        [
            x,
            y,
            BOTTOM / 2
        ]

    )


    # ========================================================
    # 前墙
    # ========================================================

    front = sim.createPrimitiveShape(

        sim.primitiveshape_cuboid,

        [
            WIDTH,
            WALL,
            HEIGHT
        ],

        0

    )


    sim.setObjectAlias(

        front,

        name + "_front",

        1

    )


    sim.setObjectPosition(

        front,

        -1,

        [
            x,
            y - DEPTH / 2 + WALL / 2,
            HEIGHT / 2
        ]

    )


    # ========================================================
    # 后墙
    # ========================================================

    back = sim.createPrimitiveShape(

        sim.primitiveshape_cuboid,

        [
            WIDTH,
            WALL,
            HEIGHT
        ],

        0

    )


    sim.setObjectAlias(

        back,

        name + "_back",

        1

    )


    sim.setObjectPosition(

        back,

        -1,

        [
            x,
            y + DEPTH / 2 - WALL / 2,
            HEIGHT / 2
        ]

    )


    # ========================================================
    # 左墙
    # ========================================================

    left = sim.createPrimitiveShape(

        sim.primitiveshape_cuboid,

        [
            WALL,
            DEPTH - 2 * WALL,
            HEIGHT
        ],

        0

    )


    sim.setObjectAlias(

        left,

        name + "_left",

        1

    )


    sim.setObjectPosition(

        left,

        -1,

        [
            x - WIDTH / 2 + WALL / 2,
            y,
            HEIGHT / 2
        ]

    )


    # ========================================================
    # 右墙
    # ========================================================

    right = sim.createPrimitiveShape(

        sim.primitiveshape_cuboid,

        [
            WALL,
            DEPTH - 2 * WALL,
            HEIGHT
        ],

        0

    )


    sim.setObjectAlias(

        right,

        name + "_right",

        1

    )


    sim.setObjectPosition(

        right,

        -1,

        [
            x + WIDTH / 2 - WALL / 2,
            y,
            HEIGHT / 2
        ]

    )


# ============================================================
# 11. 左料盒
# ============================================================

print("==============================================")
print("                创建左料盒")
print("==============================================")
print()


create_bin(

    "LeftBin",

    OBJECT_X,

    -0.9

)


print("LeftBin")

print("X = 0.380 m")

print("Y = -0.900 m")

print("底部 Z = 0.000 m")

print("底面积 = 0.45 × 0.45 m")

print("高度 = 0.08 m")

print("状态 = 放置在地面")

print()


# ============================================================
# 12. 右料盒
# ============================================================

print("==============================================")
print("                创建右料盒")
print("==============================================")
print()


create_bin(

    "RightBin",

    OBJECT_X,

    0.9

)


print("RightBin")

print("X = 0.380 m")

print("Y = 0.900 m")

print("底部 Z = 0.000 m")

print("底面积 = 0.45 × 0.45 m")

print("高度 = 0.08 m")

print("状态 = 放置在地面")

print()


# ============================================================
# 13. 删除原来的 Box
# ============================================================

print("删除原来的 /Box...")

if box is not None:

    try:

        sim.removeObject(box)

        print("/Box 已删除")

    except Exception as e:

        print("删除 /Box 失败：", e)

else:

    print("没有找到 /Box，无需删除")


print()


# ============================================================
# 14. 输出最终布局
# ============================================================

print("==============================================")
print("                 最终布局")
print("==============================================")
print()


print(
    "LeftBin   Grid1   Grid2   Grid3   Grid4   Grid5   Grid6   RightBin"
)

print()


print(
    " -0.9     -0.5    -0.3    -0.1     0.1     0.3     0.5     0.9"
)

print()


print("Grid1 → 蓝色四棱柱 → A类")

print("Grid2 → 蓝色四棱柱 → A类")

print("Grid3 → 蓝色四棱柱 → A类")

print()

print("Grid4 → 红色圆柱 → B类")

print("Grid5 → 红色圆柱 → B类")

print("Grid6 → 红色圆柱 → B类")

print()


# ============================================================
# 15. 尺寸信息
# ============================================================

print("==============================================")
print("                 尺寸参数")
print("==============================================")
print()

print("四棱柱底边：0.16 × 0.16 m")

print("圆柱直径：0.16 m")

print("圆柱半径：0.08 m")

print("所有物体高度：0.22 m")

print("物体中心高度：Z = 0.11 m")

print()


# ============================================================
# 16. 料盒信息
# ============================================================

print("==============================================")
print("                 料盒参数")
print("==============================================")
print()

print("左料盒：Y = -0.90 m")

print("右料盒：Y = 0.90 m")

print("X = 0.38 m")

print("底面积：0.45 × 0.45 m")

print("高度：0.08 m")

print("底部：Z = 0")

print("顶部：开放")

print("状态：贴地")

print()


# ============================================================
# 17. RoboMaster 信息
# ============================================================

print("==============================================")
print("              RoboMaster 初始位置")
print("==============================================")
print()

print("X = 0.000 m")

print("Y = 0.000 m")

print("Z = 0.000 m")

print()


# ============================================================
# 18. 分类规则
# ============================================================

print("==============================================")
print("                 分类规则")
print("==============================================")
print()

print("蓝色四棱柱 → A类 → 左料盒")

print("红色圆柱   → B类 → 右料盒")

print()


# ============================================================
# 19. 完成
# ============================================================

print("==============================================")
print("            六网格场景创建完成")
print("==============================================")
print()

input("按 Enter 退出...")