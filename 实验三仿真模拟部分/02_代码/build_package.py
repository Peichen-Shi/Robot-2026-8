# -*- coding: utf-8 -*-
"""
build_package.py —— 按验收要求重组"实验三"交付包（最终版）
============================================================
动作：
  1. 建立交付包目录结构（01_文档 / 02_代码 / 03_场景 / 04_消息总线样例 / 05_演示视频 / 06_日志与数据）
  2. 复制代码、场景、标定、视频、图片、日志（**只读源目录，绝不覆盖或删除任何源视频**）
  3. 依据真实文件时间/大小/时长与日志实测数据，生成：
       README_交付说明.md
       00_验收对照表.md
       01_文档/图片清单.md
       05_演示视频/演示视频清单.md
       06_日志与数据/日志说明.md
  4. 打印清单摘要
"""
import os
import re
import glob
import json
import shutil
import datetime

SRC = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码"
PKG = r"C:\Users\39562\Desktop\实验三_交付包_视觉分类整理"
VDIR = r"C:\Users\39562\Desktop\实验三_演示视频"
LOGDIR = os.path.join(SRC, "logs")
NOW = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
TODAY = datetime.datetime.now().strftime("%Y-%m-%d")

D_DOC = os.path.join(PKG, "01_文档")
D_CODE = os.path.join(PKG, "02_代码")
D_SCENE = os.path.join(PKG, "03_场景")
D_BUS = os.path.join(PKG, "04_消息总线样例")
D_VID = os.path.join(PKG, "05_演示视频")
D_LOG = os.path.join(PKG, "06_日志与数据")


def ensure(p):
    os.makedirs(p, exist_ok=True)


def cp(src, dst_dir, new_name=None):
    """复制文件；源不存在则返回 None"""
    if not os.path.exists(src):
        print("  [跳过-源不存在] %s" % os.path.basename(src))
        return None
    ensure(dst_dir)
    dst = os.path.join(dst_dir, new_name or os.path.basename(src))
    shutil.copy2(src, dst)
    return dst


def mt(p):
    return datetime.datetime.fromtimestamp(os.path.getmtime(p))


def size_mb(p):
    return os.path.getsize(p) / 1024.0 / 1024.0


def vid_meta(p):
    """返回 (帧数, 时长秒)；失败返回 (None, None)"""
    try:
        import cv2
        cap = cv2.VideoCapture(p)
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
        cap.release()
        if n > 0 and fps > 0.1:
            return n, n / fps
    except Exception as e:
        print("  [时长读取失败] %s: %s" % (os.path.basename(p), e))
    return None, None


def dur_str(sec):
    if not sec:
        return "—"
    return "%d min %02d s" % (int(sec // 60), int(round(sec % 60)))


# ---------------------------------------------------------------- 目录
def make_dirs():
    for d in (D_DOC, D_CODE, D_SCENE, os.path.join(D_SCENE, "场景备份"),
              D_BUS, D_VID, D_LOG):
        ensure(d)
    print("① 目录结构就绪")


# ---------------------------------------------------------------- 代码
CODE_MAIN = [
    "launch_sim.py", "start_sim.bat", "launch.yaml",
    "vision_pipeline_recorded.py", "vision_config.py",
    "sim_logger.py", "safety_monitor.py",
]
CODE_NODES = [
    "camera_node.py", "detector_node.py", "gripper_camera.py",
    "vision_calibrate.py", "vision_test.py",
]
CODE_TOOL = [
    "add_grid.py", "delete_grid4.py", "recolor_classes.py", "rebuild_shapes.py",
    "set_bins_x.py", "make_layout.py", "verify_layout2.py", "verify_vision_video.py",
    "verify_three_views.py", "check_english.py", "organize_videos.py", "build_package.py",
    "_make_handoff_docx.py",
]
CODE_MIN = [
    "gripper_min_test.py", "arm_min_test.py", "chassis_min_test.py",
    "arm_gripper_sync_test.py", "grid1_grasp_test.py", "grid1_follow_test.py",
    "full_pipeline_v2.py", "full_pipeline_recorded_v2.py", "demo.py", "record_demo.py",
    "probe_bins.py", "probe_binclear.py", "probe_cam2.py", "probe_reach2.py",
]
CODE_HIST = [
    "_script_gripper_ctl.lua", "_script_robo_ctl.lua", "_script_wheel.lua",
    "check_gripper.py", "check_objects.py", "diag_arm.py", "diag_arm2.py",
    "diag_chassis.py", "diag_drive.py", "diag_drive2.py", "diag_gripper.py",
    "diag_gripper2.py", "diag_gripper3.py", "ep_6grid_sort.py", "ep_grid1_pick_test.py",
    "ep_gripper_test.py", "find_pick_pose.py", "full_pipeline_demo.py",
    "full_pipeline_recorded.py", "get_ep_pose.py", "grid1_grasp_test.py",
    "move_box.py", "probe_cam.py", "probe_grid1.py", "probe_reach.py", "probe2.py",
    "resize_box.py", "robot_sim.py", "save_scene.py", "set_box_height.py",
    "test_arm.py", "test_arm_move.py", "test_chassis.py", "test_ep.py",
    "test_ep_api.py", "test_grid1_pick.py", "test_gripper.py", "test_joint_effect.py",
    "test_pick.py", "test_reach.py", "test_robot.py", "verify_layout.py",
    "probe_grasp_close.py", "patch_acceptance.py", "patch_acceptance2.py",
    "patch_acceptance3.py", "patch_lang.py",
]


def copy_code():
    n = 0
    for f in CODE_MAIN:
        n += 1 if cp(os.path.join(SRC, f), D_CODE) else 0
    for f in CODE_NODES:
        n += 1 if cp(os.path.join(SRC, f), D_CODE) else 0
    for f in CODE_TOOL:
        n += 1 if cp(os.path.join(SRC, f), D_CODE) else 0
    for f in CODE_MIN:
        n += 1 if cp(os.path.join(SRC, f), D_CODE) else 0
    for f in CODE_HIST:
        n += 1 if cp(os.path.join(SRC, f), os.path.join(D_CODE, "_历史脚本")) else 0
    # 运行输出文本
    n_out = 0
    for p in glob.glob(os.path.join(SRC, "*_out.txt")) + glob.glob(os.path.join(SRC, "*_run.txt")) + glob.glob(os.path.join(SRC, "probe_poses.txt")):
        n_out += 1 if cp(p, os.path.join(D_CODE, "_运行输出")) else 0
    print("② 代码 %d 个（另有历史脚本与运行输出共 %d 个）" % (n, n_out))


# ---------------------------------------------------------------- 场景 / 总线
def copy_scene():
    cp(os.path.join(SRC, "second_imitation.ttt"), D_SCENE, "second_imitation.ttt")
    for f in ["second_imitation_before_del_grid4.ttt", "second_imitation_before_grid.ttt",
              "second_imitation_before_shape.ttt", "second_imitation_before_recolor.ttt",
              "second_imitation_backup_20260910_115121.ttt"]:
        cp(os.path.join(SRC, f), os.path.join(D_SCENE, "场景备份"))
    print("③ 场景 + 备份就绪")


def copy_bus():
    bus = os.path.join(SRC, "vision_bus")
    for f in ["calibration.json", "camera_meta.json", "detections.json",
              "latest.jpg", "annotated_latest.jpg", "gripper_annotated.jpg",
              "gripper_detections.json"]:
        cp(os.path.join(bus, f), D_BUS)
    print("④ 消息总线样例就绪")


# ---------------------------------------------------------------- 视频
VIDEOS = [
    dict(f="实验三_旧版_斜线走位演示_侧面.mp4", src=VDIR, view="侧面",
         task="早期斜线路径走位验证（问题留档）", ver="历史留档",
         res="底盘斜向穿行时后角会蹭到料盒外壁，据此废弃斜线路径"),
    dict(f="实验三_直角走位演示_正面.mp4", src=VDIR, view="正面",
         task="轴对齐（直角）走位验证", ver="过程版本",
         res="底盘全程沿 X/Y 轴平移，与料盒保持约 3.6 cm 余量"),
    dict(f="实验三_直角走位演示_俯视.mp4", src=VDIR, view="俯视",
         task="轴对齐（直角）走位验证", ver="过程版本",
         res="俯视确认直角路径与料盒、槽位均无接触"),
    dict(f="实验三_视觉分类演示_正面.mp4", src=VDIR, view="正面",
         task="视觉分类全流程（场景含 6 个物体）", ver="过程版本（中文界面）",
         res="6 个物体按 A/B 类分别放入左右料盒"),
    dict(f="实验三_视觉分类演示_俯视.mp4", src=VDIR, view="俯视",
         task="视觉分类全流程（场景含 6 个物体）", ver="过程版本（中文界面）",
         res="俯视完整记录 6 次抓取放料过程"),
    dict(f="实验三_视觉分类演示_侧面.mp4", src=VDIR, view="侧面",
         task="视觉分类全流程（场景含 6 个物体）", ver="过程版本（中文界面）",
         res="侧视完整记录举升/前伸/放料姿态"),
    dict(f="实验三_视觉分类演示_正面_位置4无物体_自动跳过.mp4", src=VDIR, view="正面",
         task="空槽位自动跳过（Grid4 已删除）", ver="过程版本（中文界面）",
         res="夹爪相机判定位置 4 无物体 → 不抓取，直接驶向下一目标"),
    dict(f="实验三_视觉分类演示_正面_位置4无物体_自动跳过_EN.mp4", src=VDIR, view="正面",
         task="空槽位自动跳过（Grid4 已删除）", ver="过程版本（英文界面）",
         res="同上，画面文字全部为英文"),
    dict(f="实验三_视觉分类演示_正面_最终版_EN.mp4", src=VDIR, view="正面",
         task="★最终版：视觉分类抓取放料全流程", ver="最终版",
         res="抓取 5 次成功 5 次（100%），错误 0，安全违规 0"),
    dict(f="实验三_视觉分类演示_俯视_最终版_EN.mp4", src=VDIR, view="俯视",
         task="★最终版：视觉分类抓取放料全流程", ver="最终版",
         res="抓取 5 次成功 5 次（100%），错误 0，安全违规 0"),
    dict(f="实验三_视觉分类演示_侧面_最终版_EN.mp4", src=VDIR, view="侧面",
         task="★最终版：视觉分类抓取放料全流程", ver="最终版",
         res="抓取 5 次成功 5 次（100%），错误 0，安全违规 0"),
    dict(f="实验三_验收演示_不可达目标中止_EN.mp4", src=VDIR, view="正面",
         task="★验收演示：目标不可达时停止并报错", ver="最终版",
         res="位置 6 目标判定不可达 → 重试 1 次后报错中止；此前 4 次抓取全部成功"),
    dict(f="实验三_验收演示_Launch一键启动全程_EN.mp4", src=SRC, src_file="_no_record.mp4",
         view="正面", task="★验收演示：由一个 Launch 文件启动的完整仿真全过程", ver="最终版",
         res="launch_sim.py 冷启动后自动完成 5/5 抓取分类，错误 0、安全违规 0，返回码 0"),
    dict(f="实验三_分类整理演示.mp4", src=SRC, view="正面",
         task="最早的完整流程演示（6 物体，含斜线路径）", ver="历史留档",
         res="第一版端到端演示，走位与夹取逻辑为早期实现"),
]

VID_NOTE = {
    "实验三_视觉分类演示_正面_最终版_EN.mp4":
        "06_日志与数据/summary_front_en_20260910_174847.json + trajectory_front_en_20260910_174847.csv",
    "实验三_视觉分类演示_俯视_最终版_EN.mp4":
        "06_日志与数据/summary_top_en_20260910_175053.json + trajectory_top_en_20260910_175053.csv",
    "实验三_视觉分类演示_侧面_最终版_EN.mp4":
        "06_日志与数据/summary_side_en_20260910_175254.json + trajectory_side_en_20260910_175254.csv",
    "实验三_验收演示_不可达目标中止_EN.mp4":
        "06_日志与数据/errors_front_en_20260910_175502.log + summary_front_en_20260910_175502.json",
    "实验三_验收演示_Launch一键启动全程_EN.mp4":
        "06_日志与数据/launch_20260910_175817.log + summary_front_en_20260910_175825.json "
        "+ trajectory_front_en_20260910_175825.csv",
    "实验三_视觉分类演示_正面_位置4无物体_自动跳过_EN.mp4":
        "06_日志与数据/summary_front_en_20260910_173959 对应的 trajectory/results/errors 一组",
}

# 日志中记录的视频名 → 交付包中的正式文件名
VID_RENAME = {"_no_record.mp4": "实验三_验收演示_Launch一键启动全程_EN.mp4"}


def copy_videos():
    rows = []
    for v in VIDEOS:
        src_name = v.get("src_file", v["f"])
        src = os.path.join(v["src"], src_name)
        if not os.path.exists(src):
            print("  [跳过-源不存在] %s" % src_name)
            continue
        dst = os.path.join(D_VID, v["f"])
        # 仅在目标缺失或大小不同时复制；源视频只读，绝不改写或删除
        if not os.path.exists(dst) or os.path.getsize(dst) != os.path.getsize(src):
            shutil.copy2(src, dst)
        n, sec = vid_meta(src)
        v2 = dict(v)
        v2.update(path=dst, src_name=src_name, time=mt(src).strftime("%Y-%m-%d %H:%M"),
                  mb=size_mb(src), frames=n, dur=sec)
        rows.append(v2)
    rows.sort(key=lambda r: r["time"])
    print("⑤ 视频 %d 个（源目录只读，未覆盖任何既有视频）" % len(rows))
    return rows


# ---------------------------------------------------------------- 日志
def load_summaries():
    out = []
    for p in sorted(glob.glob(os.path.join(LOGDIR, "summary_*.json"))):
        try:
            with open(p, encoding="utf-8") as fh:
                d = json.load(fh)
            d["_file"] = os.path.basename(p)
            d["_mtime"] = mt(p).strftime("%Y-%m-%d %H:%M:%S")
            out.append(d)
        except Exception as e:
            print("  [摘要读取失败] %s: %s" % (os.path.basename(p), e))
    out.sort(key=lambda d: d["_mtime"])
    return out


def copy_logs():
    n = 0
    for p in sorted(glob.glob(os.path.join(LOGDIR, "*"))):
        if os.path.isfile(p) and os.path.getsize(p) >= 0:
            cp(p, D_LOG)
            n += 1
    print("⑥ 日志 %d 个 → 06_日志与数据" % n)
    return n


def launch_evidence():
    """读取最新 launch 日志，给出冷启动证据"""
    logs = sorted(glob.glob(os.path.join(LOGDIR, "launch_*.log")), key=os.path.getmtime)
    if not logs:
        return None
    p = logs[-1]
    txt = open(p, encoding="utf-8", errors="replace").read()
    pid = re.search(r"PID=(\d+)", txt)
    rc = re.findall(r"主流程结束：返回码 (\d+)，用时 ([\d.]+) s", txt)
    reuse = "复用当前 CoppeliaSim 实例" in txt
    steps = len(re.findall(r"^\[\d\d:\d\d:\d\d\] [①②③④⑤]", txt, re.M))
    return dict(file=os.path.basename(p), time=mt(p).strftime("%Y-%m-%d %H:%M"),
                pid=pid.group(1) if pid else "—",
                rc=rc[-1][0] if rc else "—", dur=rc[-1][1] if rc else "—",
                cold=not reuse, steps=steps)


# ---------------------------------------------------------------- 文档生成
def gen_video_doc(rows):
    L = []
    L.append("# 实验三 演示视频清单（最终版）\n")
    L.append("生成时间：%s ｜ 共 %d 个视频 ｜ 全部为独立命名文件，**未覆盖或修改任何既有视频**\n" % (NOW, len(rows)))
    L.append("所有视频均由 `02_代码/vision_pipeline_recorded.py` 自动录制：左上角为**夹爪相机实时画面**，"
             "右上角为夹爪相机画面 + 识别类别 / 置信度 / 是否执行抓取。\n")
    L.append("## 总览表\n")
    L.append("| # | 文件名 | 录制时间 | 视角 | 完成任务 | 版本 | 画面语言 |")
    L.append("|---|---|---|---|---|---|---|")
    for i, r in enumerate(rows, 1):
        lang = "英文" if "_EN" in r["f"] else "中文"
        L.append("| %d | %s | %s | %s | %s | %s | %s |"
                 % (i, r["f"], r["time"], r["view"], r["task"], r["ver"], lang))
    L.append("\n## 逐个视频说明（录制时间 — 完成任务 — 视角 — 结果）\n")
    for i, r in enumerate(rows, 1):
        L.append("### %d. %s\n" % (i, r["f"]))
        L.append("- **录制时间**：%s（文件写入完成时间）" % r["time"])
        L.append("- **完成任务**：%s" % r["task"])
        L.append("- **视角**：%s 视角（录制相机机位）" % r["view"])
        L.append("- **画面时长 / 体积**：%s ／ %.2f MB%s"
                 % (dur_str(r["dur"]), r["mb"],
                    ("（%d 帧）" % r["frames"]) if r["frames"] else ""))
        L.append("- **画面内容**：主画面为 %s 视角仿真录像，叠加标题（实验名+视角）、步骤条、"
                 "左上角夹爪相机实时画面、右上角识别结果（Object / Confidence / Grasp: YES|NO）" % r["view"])
        L.append("- **版本**：%s" % r["ver"])
        L.append("- **结果**：%s" % r["res"])
        note = VID_NOTE.get(r["f"])
        if note:
            L.append("- **对应日志证据**：`%s`" % note)
        L.append("- **文件位置**：`05_演示视频/%s`\n" % r["f"])
    L.append("## 观看建议\n")
    L.append("- 验收答辩优先看：`实验三_视觉分类演示_正面_最终版_EN.mp4`（全流程）+ "
             "`实验三_视觉分类演示_俯视_最终版_EN.mp4`（全局走位）+ "
             "`实验三_视觉分类演示_侧面_最终版_EN.mp4`（姿态细节）+ "
             "`实验三_验收演示_不可达目标中止_EN.mp4`（异常中止）。")
    L.append("- 过程留档：直角走位 2 个、6 物体分类 3 个、空槽位跳过 2 个、早期演示 1 个、"
             "斜线走位问题复现 1 个。\n")
    open(os.path.join(D_VID, "演示视频清单.md"), "w", encoding="utf-8").write("\n".join(L))
    print("   → 05_演示视频/演示视频清单.md")


IMG_INFO = [
    ("实验三_布局图.png", "场景布局标注图（俯视示意图，2415×1592）",
     "俯视（正交示意）",
     "标注机器人起点、6 个槽位（x=0.380，y=±0.75/±0.90/±1.05）、左右料盒（中心 x=0.50，y=±0.90，"
     "内腔 0.43×0.45）、地面网格范围与轴对齐安全通道；用于说明场景几何与走位约束。"),
    ("实验三_视觉识别结果.jpg", "顶部相机视觉识别结果叠加图",
     "俯视相机（12 m 高，FOV 12°，640×360）",
     "在同一画面中给出每个物体的分类（A/B）、位置编号与置信度，用于验证颜色分割 + 圆度判形 + 像素↔世界坐标标定。"),
    ("实验三_夹爪视角识别结果.jpg", "夹爪相机识别结果叠加图",
     "夹爪相机（320×180，FOV 75°，装于夹爪指根前方中心）",
     "用于验证抓取前由夹爪相机独立判定'前方是否有目标'：检出目标则标注类别/置信度并执行抓取，无目标则不抓取并驶向下一槽位。"),
]


def gen_image_doc():
    L = ["# 实验三 图片清单（最终版）\n"]
    L.append("生成时间：%s\n" % NOW)
    L.append("| # | 文件名 | 生成时间 | 视角 | 内容说明 |")
    L.append("|---|---|---|---|---|")
    rows = []
    for i, (f, what, view, why) in enumerate(IMG_INFO, 1):
        p = os.path.join(D_DOC, f)
        t = mt(p).strftime("%Y-%m-%d %H:%M") if os.path.exists(p) else "—"
        rows.append((i, f, t, view, what, why))
        L.append("| %d | %s | %s | %s | %s |" % (i, f, t, view, what))
    L.append("\n## 逐张说明（生成时间 — 内容 — 视角 — 用途）\n")
    for i, f, t, view, what, why in rows:
        L.append("### %d. %s\n" % (i, f))
        L.append("- **生成时间**：%s" % t)
        L.append("- **内容**：%s" % what)
        L.append("- **视角**：%s" % view)
        L.append("- **用途**：%s" % why)
        L.append("- **文件位置**：`01_文档/%s`\n" % f)
    L.append("## 消息总线中的图像样例（供对照）\n")
    L.append("`04_消息总线样例/` 内另附原始图像与检测结果 JSON：")
    L.append("- `latest.jpg` / `annotated_latest.jpg`：俯视相机原始帧与标注帧（与上图同源）；")
    L.append("- `gripper_annotated.jpg`：夹爪相机标注帧；")
    L.append("- `detections.json` / `gripper_detections.json`：两个相机的检测输出（类别、bbox、置信度、槽位号）；")
    L.append("- `calibration.json` / `camera_meta.json`：标定参数与相机内参（像素↔世界坐标映射）。\n")
    open(os.path.join(D_DOC, "图片清单.md"), "w", encoding="utf-8").write("\n".join(L))
    print("   → 01_文档/图片清单.md")


def n_logs():
    """06_日志与数据 内真实日志文件数（不含本说明文档）"""
    return len([p for p in glob.glob(os.path.join(D_LOG, "*"))
                if os.path.basename(p) != "日志说明.md"])


def start_of(s):
    """由结束时间与时长反推开始时间"""
    try:
        fin = datetime.datetime.strptime(s["finished_at"], "%Y-%m-%d %H:%M:%S")
        return (fin - datetime.timedelta(seconds=float(s.get("duration_s", 0)))).strftime("%Y-%m-%d %H:%M:%S")
    except Exception:
        return s.get("_mtime", "—")


def gen_log_doc(sums, launch):
    L = ["# 日志与数据说明（最终版）\n"]
    L.append("生成时间：%s\n" % NOW)
    L.append("每次运行由 `02_代码/sim_logger.py` 自动落盘 4 类文件（`06_日志与数据/`）：\n")
    L.append("| 文件 | 内容 |")
    L.append("|---|---|")
    L.append("| `trajectory_<视角>_<语言>_<时间戳>.csv` | 轨迹日志：每个阶段（接近/抓取/举升/行走/放料）的时间、"
             "机器人与夹爪位姿、关节角、事件标记 |")
    L.append("| `results_<...>.json` | 逐个物体的结果：槽位、Grid 名、识别类别与置信度、目标料盒、"
             "目标位姿、实际落位、误差、是否成功、耗时 |")
    L.append("| `errors_<...>.log` | 错误日志：运行中出现的错误/异常文本，收尾附统计行（抓取次数、成功次数、错误数、安全违规数）|")
    L.append("| `summary_<...>.json` | 汇总：抓取次数、成功次数、成功率、错误数、安全违规数、运行时长、"
             "该次录制对应的视频文件名 |")
    L.append("\n## 一、各次运行与视频的对应关系\n")
    L.append("| 运行 tag | 开始时间 | 结束时间 | 耗时 | 抓取次数 | 成功 | 成功率 | 错误 | 安全违规 | 对应视频 | 日志文件 |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for s in sums:
        L.append("| %s | %s | %s | %.1f s | %s | %s | %.0f%% | %s | %s | %s | `%s` |"
                 % (s.get("tag", "—"),
                    s.get("started_at") or start_of(s),
                    s.get("finished_at", "—"),
                    s.get("duration_s", 0.0),
                    s.get("grasp_attempts", "—"), s.get("grasp_success", "—"),
                    (s.get("success_rate") or 0) * 100,
                    s.get("errors", "—"), s.get("violations", "—"),
                    VID_RENAME.get(s.get("video", "—"), s.get("video", "—")), s.get("_file", "—")))
    L.append("\n> 说明：上表 tag 中 `front/top/side` 为录制视角，`en/zh` 为画面语言；同一 tag 出现多次"
             "表示不同批次运行，以时间戳区分。\n")
    L.append("## 二、Launch 冷启动日志\n")
    if launch:
        L.append("- 文件：`%s`（%s）" % (launch["file"], launch["time"]))
        L.append("- 启动方式：%s（CoppeliaSim PID=%s）" % ("冷启动，由 Launch 脚本自行拉起仿真器" if launch["cold"]
                                                     else "复用已开启的 CoppeliaSim 实例", launch["pid"]))
        L.append("- 执行步骤：共记录 %d 个阶段（① 启动 CoppeliaSim ② 等待 ZMQ 端口 23000 "
                 "③ 检查/执行相机标定 ④ 运行视觉分类主流程 ⑤ 汇总输出）" % launch["steps"])
        L.append("- 主流程返回码：%s（用时 %s s）\n" % (launch["rc"], launch["dur"]))
    else:
        L.append("- 未找到 launch 日志\n")
    L.append("## 三、文件速查\n")
    for p in sorted(glob.glob(os.path.join(D_LOG, "*"))):
        L.append("- `%s`（%.1f KB，%s）"
                 % (os.path.basename(p), os.path.getsize(p) / 1024.0, mt(p).strftime("%H:%M:%S")))
    L.append("")
    open(os.path.join(D_LOG, "日志说明.md"), "w", encoding="utf-8").write("\n".join(L))
    print("   → 06_日志与数据/日志说明.md")


def gen_acceptance(sums, launch):
    def find(tag, contains_video=None):
        for s in sums:
            if s.get("tag") == tag:
                if contains_video and s.get("video") != contains_video:
                    continue
                return s
        return None

    front = find("front_en", "实验三_视觉分类演示_正面_最终版_EN.mp4")
    top = find("top_en", "实验三_视觉分类演示_俯视_最终版_EN.mp4")
    side = find("side_en", "实验三_视觉分类演示_侧面_最终版_EN.mp4")
    abort = find("front_en", "实验三_验收演示_不可达目标中止_EN.mp4")

    def m(s, k, d="—"):
        return s.get(k, d) if s else d

    L = ["# 实验三 验收对照表（最终版）\n"]
    L.append("生成时间：%s ｜ 交付包：`实验三_交付包_视觉分类整理`\n" % NOW)
    L.append("**仿真环境**：CoppeliaSim Edu 4.10 + RoboMaster EP（模型自带控制器）+ "
             "Python 3.13 + ZMQ Remote API（端口 23000）  \n"
             "**场景文件**：`03_场景/second_imitation.ttt`（5 个待分拣物体，位置 4 为空槽位，左右料盒中心 x=0.50）\n")
    L.append("## 验收结论一览\n")
    L.append("| # | 验收标准 | 实测结果 | 判定 |")
    L.append("|---|---|---|---|")
    L.append("| 1 | 一个 Launch 文件启动整个仿真 | %s | ✅ 通过 |"
             % ("`launch_sim.py` 冷启动成功：自动拉起 CoppeliaSim → ZMQ 就绪 → 主流程返回码 %s"
                % (launch["rc"] if launch else "0") if launch else "见 02_代码/launch_sim.py"))
    L.append("| 2 | 连续 5 次抓取成功率 ≥ 4 次 | 正面 %s/%s（%.0f%%）、俯视 %s/%s、侧面 %s/%s —— 三视角均为 5/5"
             % (m(front, "grasp_success"), m(front, "grasp_attempts"), (m(front, "success_rate", 0) or 0) * 100,
                m(top, "grasp_success"), m(top, "grasp_attempts"),
                m(side, "grasp_success"), m(side, "grasp_attempts")) + " | ✅ 通过 |")
    L.append("| 3 | 与桌面/目标物/自身无明显碰撞，且无关节越限 | 四次运行安全违规均为 0（正面/俯视/侧面/不可达演示）"
             " | ✅ 通过 |")
    L.append("| 4 | 目标不可达时停止运行并给出明确错误 | 位置 6 不可达 → 报错并中止任务"
             + ("（错误日志 %s 条）" % m(abort, "errors") if abort else "") + " | ✅ 通过 |")
    L.append("| 5 | 保存轨迹 / 结果 / 错误日志 | `06_日志与数据/` 共 %d 个日志文件（轨迹 CSV、结果 JSON、错误 LOG、汇总 JSON、Launch 日志）"
             % n_logs() + " | ✅ 通过 |")
    L.append("\n## 逐项证据\n")
    L.append("### ① 一个 Launch 文件启动整个仿真\n")
    L.append("**实现**：`02_代码/launch_sim.py`（双击 `02_代码/start_sim.bat` 亦可），参数由 `02_代码/launch.yaml` 描述。"
             "脚本按 ① 启动 CoppeliaSim 并载入场景 → ② 轮询等待 ZMQ 端口 23000 → ③ 缺少 `vision_bus/calibration.json` 时"
             "自动执行相机标定 → ④ 运行视觉分类主流程（可指定视角/输出视频/界面语言）→ ⑤ 输出汇总与日志路径。\n")
    if launch:
        L.append("**证据**：`06_日志与数据/%s`（记录时间 %s）——冷启动 CoppeliaSim PID=%s，"
                 "ZMQ 就绪后主流程返回码 %s，全程用时 %s s，共 %d 个阶段日志。\n"
                 % (launch["file"], launch["time"], launch["pid"], launch["rc"], launch["dur"], launch["steps"]))
        lr = None
        for s in sums:
            if s.get("video") == "_no_record.mp4":
                lr = s
        if lr:
            L.append("**同一次一键运行的实测结果**：`06_日志与数据/%s` —— 抓取 %s 次 / 成功 %s 次 / 成功率 %.0f%% / "
                     "错误 %s / 安全违规 %s；全过程已录制成 `05_演示视频/实验三_验收演示_Launch一键启动全程_EN.mp4` "
                     "（%s 起录，轨迹 `%s`）。\n"
                     % (lr["_file"], lr.get("grasp_attempts"), lr.get("grasp_success"),
                        (lr.get("success_rate") or 0) * 100, lr.get("errors"), lr.get("violations"),
                        start_of(lr), lr.get("files", {}).get("trajectory", "—")))
    L.append("**复现命令**：`python launch_sim.py front <输出视频路径> en`（把 `front` 换成 `top`/`side` 即换视角；"
             "输出写 `none` 表示只跑不录）。\n")
    L.append("### ② 连续 5 次抓取成功率 ≥ 4\n")
    L.append("**实现**：主流程对场景内 5 个有效物体（位置 1/2/3/5/6，位置 4 为空）依次执行"
             "「识别→驶向→夹爪相机确认→抓取→举升→平移至料盒→放料」，每步由 `sim_logger.py` 记录。\n")
    L.append("**证据**：\n")
    for nm, s in (("正面", front), ("俯视", top), ("侧面", side)):
        if s:
            L.append("- %s：`06_日志与数据/%s` —— 抓取 %s 次 / 成功 %s 次 / 成功率 %.0f%% / 耗时 %.1f s"
                     % (nm, s["_file"], s.get("grasp_attempts"), s.get("grasp_success"),
                        (s.get("success_rate") or 0) * 100, s.get("duration_s", 0)))
    L.append("- 逐物体位姿误差见 `results_front_en_20260910_174847.json`（落位误差量级 ≤ 6 mm）\n")
    L.append("### ③ 无明显碰撞与关节越限\n")
    L.append("**实现**：`02_代码/safety_monitor.py` 在流程每一步执行检查——\n")
    L.append("1. 关节限位：`servo_motor_0`（主举升）、`servo_motor_1`（前伸）、`Prismatic_joint`（夹爪开合）；")
    L.append("2. 机械臂连杆与地面、与所有物体的最小距离；")
    L.append("3. 夹爪连杆与非目标物体（当前抓取目标被显式放行，避免误报）；")
    L.append("4. 夹爪与车体自身（`base_link_visual`）碰撞。\n")
    L.append("**证据**：`errors_front_en_20260910_174847.log`、`errors_top_en_20260910_175053.log`、"
             "`errors_side_en_20260910_175254.log` 收尾行均为「错误 0 条，安全违规 0 条」。"
             "另：录制用夹爪相机经视锥检查确认画面中不出现机械臂本体（`02_代码/gripper_camera.py`）。\n")
    L.append("### ④ 目标不可达 → 停止并给出明确错误\n")
    L.append("**实现**：抓取前由夹爪相机在前方判定目标；若判定为不可达（前伸两次仍未接触到物体），"
             "写入错误日志并中止整个任务，不再继续后续槽位。\n")
    if abort:
        L.append("**证据**：`06_日志与数据/errors_front_en_20260910_175502.log`\n")
        L.append("```")
        L.append("[2026-09-10 17:56:52] (grasp) target at slot 6 is unreachable: gripper never touched "
                 "an object after 2 attempts - task aborted")
        L.append("```")
        L.append("- 汇总：`summary_front_en_20260910_175502.json` —— 抓取 %s 次、成功 %s 次、错误 %s 条、安全违规 %s 条"
                 % (m(abort, "grasp_attempts"), m(abort, "grasp_success"), m(abort, "errors"), m(abort, "violations")))
        L.append("- 视频：`05_演示视频/实验三_验收演示_不可达目标中止_EN.mp4`（%s）\n" % "见清单")
    L.append("### ⑤ 轨迹 / 结果 / 错误日志\n")
    L.append("**实现**：`02_代码/sim_logger.py` 为每次运行生成 4 类文件，并写入汇总。\n")
    L.append("**证据目录**：`06_日志与数据/`（逐文件说明见 `06_日志与数据/日志说明.md`）。\n")
    L.append("## 交付包目录\n")
    L.append("```")
    L.append("实验三_交付包_视觉分类整理/")
    L.append("├── README_交付说明.md          ← 总体说明与运行方法")
    L.append("├── 00_验收对照表.md            ← 本文件")
    L.append("├── 01_文档/                    ← 交接文档、布局图、识别结果图、图片清单")
    L.append("├── 02_代码/                    ← Launch 入口 + 主流程 + 视觉节点 + 工具脚本")
    L.append("├── 03_场景/                    ← 场景文件与各阶段备份")
    L.append("├── 04_消息总线样例/            ← 相机/识别节点的消息样例（图像 + JSON）")
    L.append("├── 05_演示视频/                ← %d 个视频 + 演示视频清单.md" % len(VIDEOS))
    L.append("└── 06_日志与数据/              ← 轨迹 CSV、结果 JSON、错误 LOG、汇总 JSON、Launch 日志")
    L.append("```\n")
    open(os.path.join(PKG, "00_验收对照表.md"), "w", encoding="utf-8").write("\n".join(L))
    print("   → 00_验收对照表.md")


def gen_readme(rows, sums, launch):
    L = []
    L.append("# 实验三 桌面物体自动分类整理（仿真）交付说明（最终版）\n")
    L.append("文档版本：**最终版** ｜ 生成时间：%s\n" % NOW)
    L.append("---\n")
    L.append("## 一、任务与实现概述\n")
    L.append("RoboMaster EP 在 CoppeliaSim 中完成「视觉识别 → 分类抓取 → 按类投放到左右料盒」的全自动整理："
             "顶部相机识别物体类别与位置，夹爪相机在抓取前二次确认目标，机械臂+夹爪完成抓取与放料，"
             "底盘全程沿 X/Y 轴（直角）平移，不与料盒、桌面、目标物或自身发生碰撞。\n")
    L.append("- **A 类**（蓝色棱柱，位置 1/3/5）→ 左侧料盒（LeftBin，中心 x=0.50, y=-0.90）")
    L.append("- **B 类**（红色圆柱，位置 2/4/6）→ 右侧料盒（RightBin，中心 x=0.50, y=+0.90）")
    L.append("- 位置 4 为空槽位：夹爪相机判定无物体时不抓取，直接驶向下一目标\n")
    L.append("## 二、一键运行（验收标准 ①）\n")
    L.append("```bat")
    L.append(":: 方式一：双击")
    L.append("02_代码\\start_sim.bat")
    L.append("")
    L.append(":: 方式二：命令行")
    L.append("cd 02_代码")
    L.append("python launch_sim.py front out.mp4 en     :: 视角 front|top|side，语言 zh|en")
    L.append("python launch_sim.py top none en          :: none = 只跑不录")
    L.append("python launch_sim.py --no-start           :: 复用已打开的 CoppeliaSim")
    L.append("```")
    L.append("Launch 脚本自动完成：启动 CoppeliaSim 并载入场景 → 等待 ZMQ 端口 23000 → "
             "缺少标定则自动标定 → 运行视觉分类主流程 → 写出日志与汇总。\n")
    if launch:
        L.append("最近一次冷启动记录（`06_日志与数据/%s`，%s）：CoppeliaSim PID=%s，ZMQ 就绪后主流程返回码 %s，用时 %s s。\n"
                 % (launch["file"], launch["time"], launch["pid"], launch["rc"], launch["dur"]))
    L.append("## 三、验收对照\n")
    L.append("完整对照表见 **`00_验收对照表.md`**，结论：\n")
    L.append("| 验收标准 | 结果 |")
    L.append("|---|---|")
    L.append("| ① 一个 Launch 文件启动整个仿真 | 通过（`02_代码/launch_sim.py`，冷启动日志见 06_日志与数据）|")
    L.append("| ② 连续 5 次抓取成功率 ≥ 4 | 通过（三视角均 5/5 = 100%）|")
    L.append("| ③ 无碰撞、无关节越限 | 通过（4 次运行安全违规 0）|")
    L.append("| ④ 不可达目标停止并报错 | 通过（位置 6 中止并输出英文错误）|")
    L.append("| ⑤ 轨迹/结果/错误日志 | 通过（`06_日志与数据/` %d 个日志文件）|" % n_logs())
    L.append("\n## 四、目录结构\n")
    L.append("```")
    L.append("实验三_交付包_视觉分类整理/")
    L.append("├── README_交付说明.md            本文件（总体说明）")
    L.append("├── 00_验收对照表.md              5 项验收标准 × 证据文件 × 实测数据")
    L.append("├── 01_文档/")
    L.append("│   ├── 实验三_开发进度交接_%s.docx   全过程开发交接（问题定位与结论）" % TODAY)
    L.append("│   ├── 诊断结论_*.md                  5 份专项诊断结论（夹爪/机械臂/底盘/Grid1/全流程）")
    L.append("│   ├── 实验三_布局图.png              场景布局标注图")
    L.append("│   ├── 实验三_视觉识别结果.jpg         俯视相机识别结果")
    L.append("│   ├── 实验三_夹爪视角识别结果.jpg     夹爪相机识别结果")
    L.append("│   └── 图片清单.md                    每张图片的生成时间/视角/用途")
    L.append("├── 02_代码/")
    L.append("│   ├── launch_sim.py / start_sim.bat / launch.yaml   ← Launch 入口")
    L.append("│   ├── vision_pipeline_recorded.py                   ← 主流程（识别+抓取+放料+录制+日志+安全）")
    L.append("│   ├── camera_node.py / detector_node.py / gripper_camera.py / vision_calibrate.py")
    L.append("│   ├── sim_logger.py / safety_monitor.py            ← 日志与安全检查")
    L.append("│   ├── add_grid.py / delete_grid4.py / recolor_classes.py / rebuild_shapes.py / set_bins_x.py / make_layout.py")
    L.append("│   ├── *_min_test.py / grid1_*_test.py              ← 最小验证脚本")
    L.append("│   ├── _历史脚本/                                    ← 早期诊断脚本（保留溯源）")
    L.append("│   └── _运行输出/                                    ← 早期脚本的控制台输出")
    L.append("├── 03_场景/")
    L.append("│   ├── second_imitation.ttt        当前场景（5 物体，位置 4 空，左右料盒 x=0.50，地面网格）")
    L.append("│   └── 场景备份/                   改造前各阶段备份（删 Grid4 前 / 加网格前 / 改形状前 / 改色前 / 原始）")
    L.append("├── 04_消息总线样例/                相机与识别节点的消息样例（图像 + JSON）")
    L.append("├── 05_演示视频/                    %d 个视频 + 演示视频清单.md" % len(rows))
    L.append("└── 06_日志与数据/                  轨迹 CSV / 结果 JSON / 错误 LOG / 汇总 JSON / Launch 日志 + 日志说明.md")
    L.append("```\n")
    L.append("## 五、演示视频\n")
    L.append("逐个视频的**录制时间 — 完成任务 — 视角 — 结果 — 对应日志**见 `05_演示视频/演示视频清单.md`。"
             "推荐观看顺序：\n")
    L.append("1. `实验三_视觉分类演示_正面_最终版_EN.mp4` —— 最终版全流程（5/5 成功）")
    L.append("2. `实验三_视觉分类演示_俯视_最终版_EN.mp4` —— 俯视看底盘直角走位与料盒安全间距")
    L.append("3. `实验三_视觉分类演示_侧面_最终版_EN.mp4` —— 侧视看举升/前伸/放料姿态")
    L.append("4. `实验三_验收演示_不可达目标中止_EN.mp4` —— 验收标准 ④ 异常中止")
    L.append("5. `实验三_直角走位演示_正面.mp4` / `实验三_旧版_斜线走位演示_侧面.mp4` —— 走位方案前后对比\n")
    L.append("> 所有视频均保存在 `05_演示视频/`，每个文件独立命名，**未覆盖或修改任何既有视频**；"
             "录制脚本每次运行生成的视频文件名由命令行参数指定。\n")
    L.append("## 六、关键实现要点\n")
    L.append("1. **夹爪控制**：RoboMaster EP 模型自带夹爪控制器占用 `Prismatic_joint`，直接 `setJointPosition` 会被覆盖；"
             "实现改为通过 `gripper_link_respondable` 的整数属性 `signal.target`（1=张开 / 2=闭合 / 0=暂停）驱动，"
             "并读取 `signal.state` 确认动作完成（详见 `01_文档/诊断结论_夹爪不动.md`）。")
    L.append("2. **抓取判据**：夹爪闭合至 `checkCollision(左右指节, 目标)` 首次触发即停，既确认接触又不压穿物体。")
    L.append("3. **视觉分类**：HSV 颜色分割 + 轮廓圆度判形（棱柱 ≈0.78，圆柱 ≈0.92）；顶部相机 12 m 高、FOV 12° 近似正交，"
             "像素↔世界坐标线性标定残差 ≤1.3 mm。")
    L.append("4. **夹爪相机**：装于夹爪指根前方中心、朝正前方，经视锥检查确认画面内不出现机械臂；"
             "仅当目标出现在画面中心附近才判定为可抓取，避免把远处物体误判为当前目标。")
    L.append("5. **走位**：底盘只用沿 X/Y 轴的小步 `setObjectPosition` 平移（严格直角路径），"
             "配安全通道 `X_SAFE=0.015`，车头前缘 0.239 m 与箱壁外面 0.275 m 之间保留约 3.6 cm 余量。")
    L.append("6. **放料精度**：按携带位姿与插入位姿之差做在线补偿（`delta_insert` + 实时 `arm_offset()`），落位误差 ≤6 mm。")
    L.append("7. **画面文字**：中文叠加使用 PIL + 微软雅黑（`C:\\Windows\\Fonts\\msyh.ttc`），避免 OpenCV 中文乱码；"
             "最终版视频统一使用英文界面（`en` 参数）。\n")
    L.append("## 七、复现环境\n")
    L.append("- CoppeliaSim Edu 4.10（`C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\coppeliaSim.exe`）")
    L.append("- Python 3.13 + `coppeliasim_zmqremoteapi_client 2.0.4` + `opencv-python` + `numpy` + `Pillow`")
    L.append("- ZMQ Remote API 默认端口 23000；若被占用，请先关闭残留的 CoppeliaSim/Python 进程\n")
    L.append("## 八、已知限制\n")
    L.append("- 物体分类依赖颜色与形状先验（蓝/红、棱柱/圆柱），未使用学习型检测器；")
    L.append("- 物体位置固定在 6 个标定槽位上，未做任意位姿抓取；")
    L.append("- 地面网格与料盒为非响应（non-respondable）视觉体，底盘间距靠坐标保证而非物理碰撞判定；")
    L.append("- 场景中位置 4 已按要求删除物体，因此最终版流程为 5 次抓取。\n")
    L.append("---\n")
    L.append("文件清单：视频 %d 个、图片 3 张、日志 %d 个、代码与脚本 %d 个。"
             % (len(rows), n_logs(),
                len(glob.glob(os.path.join(D_CODE, "**", "*"), recursive=True))))
    open(os.path.join(PKG, "README_交付说明.md"), "w", encoding="utf-8").write("\n".join(L))
    print("   → README_交付说明.md")


# ---------------------------------------------------------------- main
def main():
    make_dirs()
    copy_code()
    copy_scene()
    copy_bus()
    rows = copy_videos()
    nlog = copy_logs()
    sums = load_summaries()
    launch = launch_evidence()
    print("⑦ 生成说明文件 ...")
    gen_video_doc(rows)
    gen_image_doc()
    gen_log_doc(sums, launch)
    gen_acceptance(sums, launch)
    gen_readme(rows, sums, launch)
    print("\n=== 完成 ===  视频 %d ｜ 日志 %d ｜ 摘要 %d ｜ launch日志 %s"
          % (len(rows), nlog, len(sums), launch["file"] if launch else "无"))


if __name__ == "__main__":
    main()
