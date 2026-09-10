# -*- coding: utf-8 -*-
"""
organize_videos.py —— 整理演示视频：统一命名（演示内容_视角）、集中到桌面文件夹、生成清单
命名规则：实验三_<演示内容>_<视角>.mp4   视角取：正面 / 俯视 / 侧面
"""
import cv2
import os
import shutil

DESK = r"C:\Users\39562\Desktop"
OUTDIR = os.path.join(DESK, "实验三_演示视频")
os.makedirs(OUTDIR, exist_ok=True)

VISION_CONTENT = ("视觉分类抓取与放置（完整视觉流程）：【只用夹爪相机判断】逐个位置开到抓取位 → 夹爪相机看正前方有没有物体 → "
                  "有物体则按类别（A类蓝色棱柱→左料盒，B类红色圆柱→右料盒）抓取、搬运并放到箱内水平线；"
                  "看不到物体则跳过、不抓取，直接前往下一个位置。"
                  "视频中【左上角＝夹爪摄像头画面（全程实时）】，"
                  "【右上角＝夹爪摄像头识别结果（判断/抓取时出现，标注 A类物体（蓝色棱柱）/B类物体（红色圆柱）+ 置信度）】")
VISION_COMMON_NOTE = ("六轴机械臂 + 夹爪：闭合到“手指刚接触物体”即停（不穿模）；放置时机械臂前伸送料，落点误差 ≤6mm；"
                      "全程直角走位（只走轴向直线）")

# (候选源路径列表, 最终文件名, 视角, 演示内容, 备注)
PLAN = [
    ([os.path.join(DESK, "实验三_视觉分类演示_正面.mp4")],
     "实验三_视觉分类演示_正面.mp4", "正面", VISION_CONTENT,
     "6 个物体全流程（位置4 当时还有物体）；含左上夹爪摄像头全程显示 + 右上抓取前识别结果"),
    ([os.path.join(DESK, "实验三_视觉分类演示_俯视.mp4")],
     "实验三_视觉分类演示_俯视.mp4", "俯视", VISION_CONTENT,
     "较早录制：6 个物体全程分类（位置4 当时还有物体）；俯视机位可看清“方形”直角走位轨迹、地面黑色网格与箱内物品排成同一条水平线"),
    ([os.path.join(DESK, "实验三_视觉分类演示_侧面.mp4")],
     "实验三_视觉分类演示_侧面.mp4", "侧面", VISION_CONTENT,
     "6 个物体全流程（位置4 当时还有物体）；侧面机位可同框看到整排物体、左右料盒与机械臂前伸送料动作"),
    ([os.path.join(OUTDIR, "实验三_视觉分类演示_正面_位置4无物体_自动跳过.mp4")],
     "实验三_视觉分类演示_正面_位置4无物体_自动跳过.mp4", "正面",
     "【中文版 · 漏检跳过演示（只用夹爪相机判断）】场景中位置4 无物体；小车仍开到位置4 的抓取位，"
     "夹爪相机看正前方——看不到物体就【跳过、不抓取】，直接前往位置5；其余 5 个正常按类别分类放置",
     "中文版（单独文件，未覆盖原视频）：位置4 自动跳过；落点误差 ≤5mm"),
    ([os.path.join(OUTDIR, "实验三_视觉分类演示_正面_位置4无物体_自动跳过_EN.mp4")],
     "实验三_视觉分类演示_正面_位置4无物体_自动跳过_EN.mp4", "正面",
     "【英文版 · 漏检跳过演示】画面文字全部为英文：右上角显示 Object（类别）、Confidence（置信度）、"
     "Grasp: YES（执行抓取）/ NO（无物体跳过）；位置4 无物体 → 跳过不抓取，其余 5 个正常分类放置",
     "★英文版：右上角显示 类别 + 置信度 + 是否夹取；落点误差 ≤4mm"),
    ([os.path.join(DESK, "实验三_演示_正面.mp4"), os.path.join(OUTDIR, "实验三_直角走位演示_正面.mp4")],
     "实验三_直角走位演示_正面.mp4", "正面",
     "不含视觉模块的对照版本：直角走位抓取放置（类别规则固定 位置1/3/5→左料盒，2/4/6→右料盒）",
     "对照版本（无视觉决策、无画中画、无左上相机）"),
    ([os.path.join(DESK, "实验三_演示_俯视.mp4"), os.path.join(OUTDIR, "实验三_直角走位演示_俯视.mp4")],
     "实验三_直角走位演示_俯视.mp4", "俯视",
     "同“直角走位演示_正面”，换俯视机位",
     "对照版本（无视觉决策、无画中画、无左上相机）"),
    ([os.path.join(DESK, "实验三_分类整理演示.mp4"),
      os.path.join(OUTDIR, "实验三_旧版_斜线走位演示_侧视.mp4"),
      os.path.join(OUTDIR, "实验三_旧版_斜线走位演示_侧面.mp4")],
     "实验三_旧版_斜线走位演示_侧面.mp4", "侧面",
     "早期版本：斜线走位、车体会靠近料盒（已被直角走位版取代）",
     "旧版留档（对比用）；最新方案请看“视觉分类演示”三个视角"),
]

rows = []
for srcs, final, view, content, note in PLAN:
    dst = os.path.join(OUTDIR, final)
    for src in srcs:
        if os.path.exists(src) and os.path.abspath(src) != os.path.abspath(dst):
            shutil.move(src, dst)
            break
    if os.path.exists(dst):
        rows.append((final, view, content, note, dst))
    else:
        print("!! 缺少:", final)

# 清理旧的命名（已被新名字取代）
for old in ["实验三_视觉分类演示_侧视.mp4", "实验三_视觉分类演示.mp4", "实验三_演示_正面.mp4", "实验三_演示_俯视.mp4"]:
    p = os.path.join(OUTDIR, old)
    if os.path.exists(p):
        os.remove(p)
        print("已移除旧命名:", old)

def info(p):
    cap = cv2.VideoCapture(p)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 1
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return n / fps, n, w, h, os.path.getsize(p) / 1024 / 1024

L = ["# 实验三 · 演示视频清单", "",
     "> 目录：`桌面\\实验三_演示视频\\`　|　命名规则：`实验三_<演示内容>_<视角>.mp4`（视角：**正面 / 俯视 / 侧面**）",
     "> **视觉分类演示共 3 个视角**（正面、俯视、侧面），演示内容完全相同，只是机位不同；",
     "> 另有 2 个不含视觉模块的对照视频，以及 1 个旧版留档视频。", ""]
for name, view, content, note, path in rows:
    dur, n, w, h, mb = info(path)
    L += ["## %s" % name, "",
          "| 项目 | 说明 |", "|---|---|",
          "| **演示内容** | %s |" % content,
          "| **拍摄视角** | **%s** |" % view,
          "| **分辨率 / 时长** | %dx%d / %.1f 秒（%d 帧） |" % (w, h, dur, n),
          "| **文件大小** | %.1f MB |" % mb,
          "| **备注** | %s |" % note, ""]

L += ["---", "",
      "## 画面上的信息怎么看",
      "- **标题栏**：演示内容 + 拍摄视角（例如“演示：视觉分类抓取与放置 ｜ 视角：俯视视角”）。",
      "- **左上角**：**夹爪摄像头画面（全程实时）** —— 相机装在夹爪上，画面随机械臂一起运动。",
      "- **右上角**：**夹爪摄像头 + 识别结果**（仅在即将抓取时出现），检测框旁标注 **A类物体（蓝色棱柱）** 或 "
      "**B类物体（红色圆柱）** 及置信度；抓取成功后自动关闭。",
      "- **步骤栏**：带编号的步骤说明（① … ⑭）与进度（物体 n/6）。",
      "- **地面**：物体 1~6 下方绘有黑色正方形网格（格心即物体位置，最外侧边缘距料盒 7.5cm，不与料盒重合）。",
      ""]

catalog = os.path.join(OUTDIR, "演示视频清单.md")
with open(catalog, "w", encoding="utf-8") as f:
    f.write("\n".join(L))
print("清单已生成:", catalog)
for name, view, content, note, path in rows:
    dur, n, w, h, mb = info(path)
    print("  %-38s 视角=%-4s %5.1fs %5.1fMB" % (name, view, dur, mb))
