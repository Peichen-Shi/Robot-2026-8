# -*- coding: utf-8 -*-
"""
make_layout.py —— 生成当前场景布局图（清晰大方 + 备注完整）
输出：项目目录、桌面、交付包的 实验三_布局图.png
内容：主图俯视布局（含地面网格/类别/料盒/投放点/参考线/直角走位）+ 右侧信息卡 + 侧视剖面 + 底部流程条
坐标均为场景实测值。
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow, FancyBboxPatch
import matplotlib.font_manager as fm
import os, shutil

for cand in ["Microsoft YaHei", "SimHei", "DengXian", "SimSun"]:
    try:
        fm.findfont(fm.FontProperties(family=cand), fallback_to_default=False)
        plt.rcParams["font.family"] = cand
        break
    except Exception:
        continue
plt.rcParams["axes.unicode_minus"] = False

# ================= 场景参数（实测） =================
OBJ_X = 0.380
OBJ_Y = [-0.5, -0.3, -0.1, 0.1, 0.3, 0.5]
OBJ_D, OBJ_H = 0.06, 0.22                  # 物体顶面尺寸 / 高度
CLASS_A_POS = [1, 3, 5]                    # A 类：蓝色四棱柱 → 左料盒
CLASS_B_POS = [2, 4, 6]                    # B 类：红色圆柱   → 右料盒
GRID_X0, GRID_X1 = 0.280, 0.480            # 地面黑色网格范围
GRID_YS = [-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6]
BIN_CX, BIN_W, BIN_D, BIN_CY = 0.50, 0.43, 0.45, 0.90
WALL, WALL_H, BOTTOM = 0.02, 0.08, 0.02
SLOT_X, SLOT_DY, SLOT_Z = 0.380, 0.15, 0.13
ROBOT_START = (0.038, 0.038)
ROBOT_L, ROBOT_W = 0.32, 0.24
X_SAFE, X_GRASP, X_BIN = 0.015, 0.207, 0.048
ROBOT_HALF_LEN, BIN_WALL_OUTER = 0.190, 0.275

C_BLUE, C_RED = "#2E6FD9", "#D93B3B"
C_BIN_F, C_BIN_E = "#F1F1F1", "#9AA0A6"
C_ROBOT_F, C_ROBOT_E = "#DCE9FF", "#2F4B7C"
C_SAFE, C_WALL, C_GRASP, C_SLOT, C_GRID = "#2E9E4F", "#C0392B", "#8E44AD", "#D98324", "#1B1B1B"

fig = plt.figure(figsize=(16.5, 11.2), dpi=150)
fig.patch.set_facecolor("white")
gs = fig.add_gridspec(3, 2, width_ratios=[1.62, 1.0], height_ratios=[1.0, 0.72, 0.20],
                      left=0.04, right=0.975, top=0.885, bottom=0.035, wspace=0.16, hspace=0.30)

fig.suptitle("实验三 · 桌面物体自动分类整理 —— 场景布局图", fontsize=21, fontweight="bold",
             color="#1B2A41", y=0.965)
fig.text(0.5, 0.912, "RoboMaster EP × CoppeliaSim 4.10　|　单位：米　|　坐标取自场景 second_imitation.ttt 实测",
         ha="center", fontsize=11.5, color="#7A7A7A")

# ==================== 主图：俯视布局 ====================
ax = fig.add_subplot(gs[0:2, 0])
ax.set_xlim(-0.32, 0.95); ax.set_ylim(-1.36, 1.36)
ax.set_aspect("equal"); ax.set_facecolor("#FCFCFD")
ax.set_xticks([0, 0.2, 0.4, 0.6, 0.8]); ax.set_yticks([-1.0, -0.5, 0, 0.5, 1.0])
ax.tick_params(labelsize=11, colors="#5A5A5A", length=3)
for s in ax.spines.values():
    s.set_color("#DDDDDD")

# 地面黑色正方形网格（与场景一致：黑色格线）
for i, y in enumerate(GRID_YS):
    ax.plot([GRID_X0, GRID_X1], [y, y], color=C_GRID, lw=2.6, solid_capstyle="butt", zorder=0.6)
for x in (GRID_X0, GRID_X1):
    ax.plot([x, x], [GRID_YS[0], GRID_YS[-1]], color=C_GRID, lw=2.6,
            solid_capstyle="butt", zorder=0.6)
ax.text((GRID_X0 + GRID_X1) / 2, GRID_YS[-1] + 0.035,
        "地面黑色正方形网格（6 格，每格 0.20）", ha="center", fontsize=9.5, color="#333333")

# 料盒
for cy, tag, subs in ((-BIN_CY, "左料盒  LeftBin", "A 类"), (BIN_CY, "右料盒  RightBin", "B 类")):
    ax.add_patch(FancyBboxPatch((BIN_CX - BIN_W/2 + WALL/2, cy - BIN_D/2 + WALL/2),
                                BIN_W - WALL, BIN_D - WALL,
                                boxstyle="round,pad=0,rounding_size=0.012",
                                facecolor=C_BIN_F, edgecolor="none", zorder=1))
    for (x0, y0, w, h) in [(BIN_CX - BIN_W/2, cy - BIN_D/2, BIN_W, WALL),
                           (BIN_CX - BIN_W/2, cy + BIN_D/2 - WALL, BIN_W, WALL),
                           (BIN_CX - BIN_W/2, cy - BIN_D/2, WALL, BIN_D),
                           (BIN_CX + BIN_W/2 - WALL, cy - BIN_D/2, WALL, BIN_D)]:
        ax.add_patch(Rectangle((x0, y0), w, h, facecolor=C_BIN_E, edgecolor="white", lw=0.6, zorder=3))
    ax.text(BIN_CX, cy - BIN_D/2 - 0.085, "%s（放 %s 物体）" % (tag, subs),
            ha="center", va="top", fontsize=12.5, color="#3A3A3A")

# 投放点（标号＝放第几个物体）
SLOTS = {1: (BIN_CY * -1 - SLOT_DY), 3: (BIN_CY * -1), 5: (BIN_CY * -1 + SLOT_DY),
         2: (BIN_CY - SLOT_DY), 4: BIN_CY, 6: (BIN_CY + SLOT_DY)}
for num, sy in SLOTS.items():
    ax.add_patch(Rectangle((SLOT_X - OBJ_D, sy - OBJ_D), OBJ_D * 2, OBJ_D * 2,
                           facecolor="white", edgecolor=C_SLOT, ls=(0, (4, 3)), lw=1.7, zorder=4))
    ax.text(SLOT_X, sy, str(num), ha="center", va="center", fontsize=13,
            color=C_SLOT, fontweight="bold", zorder=5)
ax.text(BIN_CX + BIN_W/2 + 0.03, BIN_CY, "投放点\nx = 0.380\n间距 0.15\nz = 0.13",
        fontsize=10, color="#9C5F00", va="center")
ax.text(BIN_CX + BIN_W/2 + 0.03, -BIN_CY, "投放点\nx = 0.380\n间距 0.15\nz = 0.13",
        fontsize=10, color="#9C5F00", va="center")

# 物体（放大显示，用真实颜色/形状）
for i, y in enumerate(OBJ_Y):
    n = i + 1
    isA = n in CLASS_A_POS
    col = C_BLUE if isA else C_RED
    if isA:
        ax.add_patch(Rectangle((OBJ_X - 0.055, y - 0.055), 0.11, 0.11,
                               facecolor=col, edgecolor="white", lw=1.4, zorder=6))
    else:
        ax.add_patch(Circle((OBJ_X, y), 0.055, facecolor=col, edgecolor="white", lw=1.4, zorder=6))
    ax.text(OBJ_X, y, str(n), ha="center", va="center", color="white",
            fontsize=11.5, fontweight="bold", zorder=7)
    ax.text(OBJ_X - 0.115, y, "%s类" % ("A" if isA else "B"), ha="right", va="center",
            fontsize=10, color=col)

# 小车起点 + 朝向
ax.add_patch(FancyBboxPatch((ROBOT_START[0] - ROBOT_L/2, ROBOT_START[1] - ROBOT_W/2),
                            ROBOT_L, ROBOT_W, boxstyle="round,pad=0.006,rounding_size=0.03",
                            facecolor=C_ROBOT_F, edgecolor=C_ROBOT_E, lw=1.8, zorder=7))
ax.add_patch(FancyArrow(ROBOT_START[0] + ROBOT_L/2, ROBOT_START[1], 0.15, 0,
                        width=0.010, head_width=0.045, color=C_ROBOT_E, zorder=8))
ax.text(ROBOT_START[0] - 0.025, ROBOT_START[1] - 0.20, "小车起点", ha="center",
        fontsize=11.5, color=C_ROBOT_E)

# 参考线（竖向标签，互不拥挤）
for x, c, lab, ylab in [(X_SAFE, C_SAFE, "安全通道", 1.32), (X_GRASP, C_GRASP, "抓取位", 1.32),
                        (BIN_WALL_OUTER, C_WALL, "料盒外壁", 1.32)]:
    ax.axvline(x, color=c, ls="--", lw=1.5, alpha=0.85, zorder=2)
    ax.text(x, ylab, lab, rotation=90, ha="center", va="top", fontsize=10.5, color=c)
for x, c, v in [(X_SAFE, C_SAFE, "0.015"), (X_GRASP, C_GRASP, "≈0.207"), (BIN_WALL_OUTER, C_WALL, "0.275")]:
    ax.text(x, -1.33, "x=%s" % v, ha="center", va="bottom", fontsize=9.5, color=c)

# 直角走位示意（一个循环）
ax.add_patch(FancyArrow(0.207 - 0.02, -0.16, -0.145, 0, width=0.008, head_width=0.03,
                        color="#E67E22", zorder=9))
ax.text(0.115, -0.135, "① 直线后退", color="#E67E22", fontsize=9.5, ha="center")
ax.add_patch(FancyArrow(0.045, -0.22, 0, -0.32, width=0.008, head_width=0.03,
                        color="#E67E22", zorder=9))
ax.text(0.075, -0.40, "② 平移\n到料盒前", color="#E67E22", fontsize=9.5)

ax.set_title("俯视布局（数字＝物体编号与投放顺序；物体符号已放大便于查看）",
             fontsize=15, color="#1B2A41", pad=14)

# ==================== 右上：信息卡 ====================
ax_i = fig.add_subplot(gs[0, 1]); ax_i.axis("off")
ax_i.add_patch(FancyBboxPatch((0.01, 0.01), 0.98, 0.98,
                              boxstyle="round,pad=0.01,rounding_size=0.03",
                              facecolor="#F7F9FC", edgecolor="#DCE3ED", lw=1.5,
                              transform=ax_i.transAxes))
ax_i.text(0.05, 0.955, "关键参数与规则", fontsize=15, color="#1B2A41",
          fontweight="bold", transform=ax_i.transAxes)
ROWS = [
    ("物体", "6 个，x = 0.380，y = −0.5 ~ +0.5，间距 0.20", C_BLUE),
    ("类别规则", "A 类＝蓝色四棱柱（位置 1/3/5）→ 左料盒", C_BLUE),
    ("", "B 类＝红色圆柱（位置 2/4/6）→ 右料盒", C_RED),
    ("地面网格", "黑色方格 0.20×0.20，x 0.28~0.48，y −0.60~+0.60", C_GRID),
    ("", "最外侧格边距料盒内壁 7.5cm（不与料盒重合）", C_GRID),
    ("料盒", "中心 (0.50, ±0.90)，内腔 0.43×0.45，壁厚 0.02 / 壁高 0.08", C_BIN_E),
    ("投放点", "x = 0.380，间距 0.15，z = 0.13（左箱 1/3/5，右箱 2/4/6）", C_SLOT),
    ("安全通道", "x = 0.015（所有横向平移在此进行）", C_SAFE),
    ("抓取位", "车体 x ≈ 0.207（夹爪对准物体中心）", C_GRASP),
    ("料盒停靠", "车体 x≈0.048，车头前缘 0.239 < 箱壁外面 0.275（余量 3.6cm）", C_WALL),
    ("走位规则", "只走轴向直线（直角走位）：抓取→后退→平移→前移→放置，不斜穿", "#E67E22"),
    ("识别方式", "俯视相机（12m 长焦）识别类别/检测框/置信度 → 定网格 → 选料盒", "#1F6FEB"),
    ("夹爪相机", "装在夹爪上；视频左上角全程显示，抓取前右上角叠加识别结果", "#1F6FEB"),
]
y0 = 0.885
for i, (k, v, c) in enumerate(ROWS):
    yy = y0 - i * 0.0695
    ax_i.add_patch(Circle((0.062, yy + 0.012), 0.0125, transform=ax_i.transAxes,
                          facecolor=c, edgecolor="none"))
    if k:
        ax_i.text(0.095, yy + 0.014, k, fontsize=12, color="#22324A",
                  fontweight="bold", transform=ax_i.transAxes)
        ax_i.text(0.235, yy + 0.014, v, fontsize=10.8, color="#4C5C70", transform=ax_i.transAxes)
    else:
        ax_i.text(0.235, yy + 0.014, v, fontsize=10.8, color="#4C5C70", transform=ax_i.transAxes)

# ==================== 右下：侧视剖面 ====================
ax2 = fig.add_subplot(gs[1, 1])
ax2.set_xlim(-0.30, 0.88); ax2.set_ylim(-0.03, 0.52)
ax2.set_aspect("equal"); ax2.set_facecolor("#FCFCFD")
ax2.set_yticks([0, 0.1, 0.2]); ax2.set_xticks([0, 0.2, 0.4, 0.6, 0.8])
ax2.tick_params(labelsize=10, colors="#5A5A5A", length=3)
for s in ax2.spines.values():
    s.set_color("#DDDDDD")
ax2.axhline(0, color="#CFCFCF", lw=3, solid_capstyle="butt")
ax2.plot([GRID_X0, GRID_X1], [0.002, 0.002], color=C_GRID, lw=3)          # 地面网格线
ax2.add_patch(Rectangle((OBJ_X - OBJ_D/2, 0), OBJ_D, OBJ_H, facecolor=C_BLUE,
                        edgecolor="white", lw=1.0))
ax2.text(OBJ_X, OBJ_H + 0.02, "物体 0.22 高", fontsize=10.5, ha="center", color="#2E6FD9")
ax2.add_patch(Rectangle((BIN_CX - BIN_W/2, 0), BIN_W, BOTTOM, facecolor=C_BIN_E))
ax2.add_patch(Rectangle((BIN_CX - BIN_W/2, 0), WALL, WALL_H, facecolor=C_BIN_E))
ax2.add_patch(Rectangle((BIN_CX + BIN_W/2 - WALL, 0), WALL, WALL_H, facecolor=C_BIN_E))
ax2.plot([BIN_CX - BIN_W/2, BIN_CX - BIN_W/2], [0, 0.30], color=C_WALL, ls="-.", lw=1.2)
ax2.text(BIN_CX - BIN_W/2 - 0.008, 0.305, "料盒外壁 x=0.275", color=C_WALL, fontsize=9, ha="center")
ax2.add_patch(Rectangle((SLOT_X - OBJ_D/2, BOTTOM), OBJ_D, OBJ_H,
                        facecolor="#BFD3F5", edgecolor="#7C9BD6", lw=1.0, ls="--"))
ax2.text(SLOT_X + 0.05, BOTTOM + OBJ_H/2, "箱内 z=0.13", fontsize=9.5, color="#5C7BB8")
ax2.text(BIN_CX + 0.02, 0.105, "壁高 0.08", fontsize=9.5, color="#6A6A6A")
ax2.add_patch(FancyBboxPatch((X_BIN - ROBOT_L/2, 0), ROBOT_L, 0.12,
                             boxstyle="round,pad=0.004,rounding_size=0.02",
                             facecolor=C_ROBOT_F, edgecolor=C_ROBOT_E, lw=1.5))
ax2.text(X_BIN, 0.145, "小车停靠 x≈0.048", ha="center", fontsize=9.5, color=C_ROBOT_E)
ax2.add_patch(FancyArrow(X_BIN + ROBOT_L/2 - 0.03, 0.085, 0.17, 0.11,
                         width=0.006, head_width=0.03, color=C_ROBOT_E))
ax2.text(0.16, 0.215, "机械臂前伸送料（约 0.35）", fontsize=9.5, color=C_ROBOT_E)
ax2.annotate("", xy=(BIN_WALL_OUTER, 0.30), xytext=(X_BIN + ROBOT_HALF_LEN, 0.30),
             arrowprops=dict(arrowstyle="<->", color=C_WALL, lw=1.3))
ax2.text((BIN_WALL_OUTER + X_BIN + ROBOT_HALF_LEN)/2, 0.315, "余量 3.6cm", ha="center",
         fontsize=9.5, color=C_WALL)
ax2.set_title("侧视剖面（高度与停靠关系）", fontsize=13, color="#1B2A41", pad=10)

# ==================== 底部：动作流程条 ====================
ax3 = fig.add_subplot(gs[2, :]); ax3.axis("off")
steps = ["①平移到物体行", "②前移对准", "③夹爪闭合至接触", "④抬升", "⑤直线后退",
         "⑥平移到料盒前", "⑦前移+手臂伸入", "⑧轻放到水平线", "⑨夹爪张开", "⑩收臂", "11.返回原位"]
x = 0.012
for i, s in enumerate(steps):
    w = 0.081
    ax3.add_patch(FancyBboxPatch((x, 0.22), w, 0.56,
                                 boxstyle="round,pad=0.004,rounding_size=0.05",
                                 facecolor="#EAF2FF" if i % 2 == 0 else "#F4F8FF",
                                 edgecolor="#B9CDEA", lw=1.0, transform=ax3.transAxes))
    ax3.text(x + w/2, 0.50, s, ha="center", va="center", fontsize=10.2, color="#22324A",
             transform=ax3.transAxes)
    x += w + 0.0065
ax3.text(0.012, 0.06, "动作流程（每个物体一个循环，共 6 次；全程只走轴向直线）",
         fontsize=11, color="#5A6A80", transform=ax3.transAxes)

out_proj = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\实验三_布局图.png"
out_desk = r"C:\Users\39562\Desktop\实验三_布局图.png"
out_pkg = r"C:\Users\39562\Desktop\实验三_交付包_视觉分类整理\01_文档\实验三_布局图.png"
fig.savefig(out_proj, bbox_inches="tight", facecolor="white")
plt.close(fig)
msg = []
for dst in (out_desk, out_pkg):
    try:
        shutil.copyfile(out_proj, dst)
        msg.append("已复制→" + os.path.basename(os.path.dirname(dst)) or dst)
    except Exception as e:
        msg.append("复制失败(%s): %s" % (os.path.dirname(dst), e))
from PIL import Image
im = Image.open(out_proj)
print("saved:", out_proj)
print("size=%dx%d  %.0f KB" % (im.size[0], im.size[1], os.path.getsize(out_proj)/1024))
for m in msg:
    print(" ", m)
