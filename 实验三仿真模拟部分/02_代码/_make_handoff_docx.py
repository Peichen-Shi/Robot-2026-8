# -*- coding: utf-8 -*-
"""生成《实验三 开发进度交接 2026-09-10.docx》到项目目录"""
from docx import Document
from docx.shared import Pt, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
import os

OUT = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\实验三_开发进度交接_2026-09-10.docx"
LAYOUT_IMG = r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码\实验三_布局图.png"

doc = Document()

# 基础样式
style = doc.styles["Normal"]
style.font.name = "微软雅黑"
style.font.size = Pt(10.5)
for lvl in range(1, 4):
    h = doc.styles["Heading %d" % lvl]
    h.font.name = "微软雅黑"
    h.font.color.rgb = RGBColor(0x1F, 0x3B, 0x73)

def H(level, text):
    doc.add_heading(text, level=level)

def P(text, bold=False, color=None):
    p = doc.add_paragraph()
    r = p.add_run(text)
    r.bold = bold
    if color:
        r.font.color.rgb = color
    return p

def B(text):
    return P(text, bold=True)

def L(text):
    doc.add_paragraph(text, style="List Bullet")

def table(headers, rows):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Light Grid Accent 1"
    for i, h in enumerate(headers):
        t.rows[0].cells[i].text = h
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = str(v)

# ============================================================
H(0, "实验三：桌面物体自动分类整理 —— 开发进度交接文档")
P("交接日期：2026-09-10（已更新：完整分类流程跑通）　|　场景：second_imitation.ttt　|　由谁接手：明日/其他 AI", color=RGBColor(0x66, 0x66, 0x66))

H(1, "0. 给新接手者的一句话交接提示")
P("本实验为 CoppeliaSim 中的 RoboMaster EP 桌面物体自动分类整理。最核心的经验一句话："
  "场景里所有执行件（夹爪/机械臂/小车）都带“模型原厂控制器/脚本”，不要试图直接硬写关节位置；"
  "要么走它自带接口（夹爪 signal 信号），要么用验证过的方式（机械臂 setJointTargetPosition；小车平移=小步传送）。"
  "截至 2026-09-10，交接清单 6 步已全部完成：夹爪开合、机械臂升降、协同、Grid1“跟随式假抓取搬运”、小车移动、"
  "以及完整分类流程（full_pipeline_demo.py：6 个物体全部按颜色放入左右分类箱，误差 0.0000 m）。"
  "下一步属于“优化与成果输出”（见第 6 节），主线功能已可用。")

H(1, "0.1 场景布局图（当前实测坐标）")
if os.path.exists(LAYOUT_IMG):
    doc.add_picture(LAYOUT_IMG, width=Inches(6.3))
    P("上图说明：主图为俯视布局（含地面黑色正方形网格、A/B 类别、左右料盒与投放点编号、三条参考线、直角走位示意）；"
      "右上为关键参数与规则信息卡；右下为侧视剖面（高度与停靠余量）；底部为动作流程条（①…⑪）。关键坐标 —— "
      "物体行 x=0.380（y=−0.5~+0.5，间距 0.20）；类别规则 A 类＝蓝色四棱柱（位置 1/3/5）→ 左料盒、B 类＝红色圆柱（位置 2/4/6）→ 右料盒；"
      "地面网格 0.20×0.20（x 0.28~0.48，y −0.60~+0.60，最外侧距料盒内壁 7.5cm）；"
      "料盒中心 (0.50, ±0.90)、内腔 0.43×0.45、壁厚 0.02、壁高 0.08；投放点 x=0.380、间距 0.15、z=0.13；"
      "安全通道 x=0.015；抓取位车体 x≈0.207；料盒停靠车体 x≈0.048（车头前缘 0.239 < 箱壁外面 0.275，余量 3.6cm）。"
      "（桌面同名的《实验三_布局图.png》可直接插入报告）", color=RGBColor(0x55, 0x55, 0x55))
else:
    P("（未找到布局图 PNG，可运行 make_layout.py 生成）", color=RGBColor(0x99, 0x33, 0x33))

H(2, "3.7 视觉分类模块（仿真相机 → 识别节点 → 决策 → 抓取）")
L("【类别规则】位置 1/3/5 = A 类物体（蓝色四棱柱）→ 左料盒；位置 2/4/6 = B 类物体（红色圆柱）→ 右料盒。")
L("     场景已按此规则重建并保存：1/3/5 为蓝色棱柱、2/4/6 为红色圆柱（recolor_classes.py 改色、rebuild_shapes.py 换形状）。")
L("【相机节点】camera_node.py：在场景中放置俯视相机 vision_cam（高度 12m + 12° 小视场角，近似正交投影，"
  "这样俯视几乎看不到物体侧面，圆度才能稳定区分方/圆），发布 latest.jpg 与 camera_meta.json。")
L("【相机标定】vision_calibrate.py：以 6 个已知网格位置为靶标，最小二乘拟合 像素↔世界 线性映射，"
  "写入 vision_bus/calibration.json；实测最大残差 ≤ 1mm（方向：画面上方 = 世界 +Y）。")
L("【识别节点】detector_node.py：HSV 颜色分割（蓝→A、红→B）+ 轮廓圆度判形状（棱柱≈0.78 / 圆柱≈0.92）；"
  "发布 detections.json（每个目标：类别 class、检测框 bbox、置信度 confidence、圆度、世界坐标、网格号、目标料盒）。")
L("【网格判定】检测框中心 →（标定映射）世界坐标 → 最近网格号 1~6（要求 |x-0.38|≤0.09、|y-网格中心|≤0.09）。")
L("【夹爪第一视角】gripper_camera.py：相机挂在夹爪手腕上（eye-in-hand），随臂运动朝夹爪前方看；"
  "小车到达抓取位、即将抓取时，在视频右上角显示画中画，识别节点对这张图做近距识别并发布"
  "gripper_detections.json（类别/检测框/置信度，近距下形状不作判据）；抓取成功后关闭画中画。")
L("【执行】vision_pipeline_recorded.py：按视觉结果选择料盒（A→LeftBin / B→RightBin），沿直角走位完成"
  "取物→搬运→放入 x=0.380 水平线→返回原位；实测 6/6 全部正确，末端误差 0.0000 m。")
L("【视频画面布局（录制时自动叠加）】标题栏＝演示内容+拍摄视角；**左上角＝夹爪摄像头画面（全程实时，相机装在夹爪上随臂运动）**；"
  "**右上角＝夹爪摄像头识别结果（即将抓取时出现，检测框旁标注 A类物体（蓝色棱柱）/B类物体（红色圆柱）+ 置信度，抓取成功后关闭）**；"
  "步骤栏＝带编号的步骤说明（①…⑭）与进度（物体 n/6）；地面＝物体 1~6 下方的黑色正方形网格。")
L("【抓取与放置细节】夹爪闭合到“手指刚接触物体”即停（checkCollision 判定，手指不穿过物体）；"
  "放置时由机械臂前伸把物体送进箱内并落到水平线 x=0.380（实测落点误差 ≤6mm），不再出现物体自己飘移的现象。")
L("【文字渲染】所有中文标注用 PIL + 微软雅黑绘制，不会出现“????”乱码。")
L("【漏检跳过（只用夹爪相机判断）】流程改为逐个位置判断：开到抓取位后用【夹爪相机】看正前方是否有物体；"
  "只有检测框中心接近画面中央的检测才算“当前位置有物体”（避免把相邻位置的物体误判为当前物体）；"
  "看不到物体则跳过、不抓取，直接前往下一个位置。演示：正面视频中位置4 的物体已移除（delete_grid4.py），"
  "系统自动跳过该位置，其余 5 个物体正常分类放置（落点误差 ≤6mm）。")
L("【消息总线】vision_bus/ 目录：latest.jpg、camera_meta.json、detections.json、annotated_latest.jpg、"
  "gripper_latest.jpg、gripper_camera_meta.json、gripper_detections.json、gripper_annotated.jpg、frames/。")

H(1, "1. 环境与启动")
table(["项目", "值"], [
    ["开发目录", r"C:\Users\39562\Desktop\实验三桌面物体自动分类整理_仿真开发代码"],
    ["CoppeliaSim", "Edu 4.10，位于 C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\coppeliaSim.exe"],
    ["仿真场景", "项目目录下 second_imitation.ttt（唯一在用场景；内含 RoboMaster + Grid1~6 + 左右分类箱零件）"],
    ["Python", "3.13（C:\\Users\\39562\\AppData\\Local\\Programs\\Python\\Python313）"],
    ["通信库", "coppeliasim_zmqremoteapi_client 2.0.4"],
    ["ZMQ 端口", "23000"],
    ["启动方法", "coppeliaSim.exe second_imitation.ttt（GUI）；等控制台出现 ZeroMQ 启动行后即可跑 python 脚本"],
    ["无关噪音", "启动时的 simWS.dll 加载错误可无视（ZMQ 正常即可）；默认脚本必须保持启用"],
])

H(1, "2. 总进度（交接清单 6 步）")
table(["步骤", "状态", "说明"], [
    ["夹爪真实开合", "✅ 完成", "走模型信号接口 signal.target，实测张开/闭合手指间距 0.117m/0.030m"],
    ["机械臂上下", "✅ 完成", "s0=servo_motor_0 主升降（负角抬正角降），s1=前伸微调"],
    ["机械臂+夹爪协同", "✅ 完成", "home→下降→闭合→抬升→张开→回 home 全链路正常"],
    ["Grid1 抓取搬运", "✅ 完成", "Grid1 宽0.12m 夹不住 → 用“跟随式假抓取”"],
    ["小车移动", "✅ 完成(有条件)", "平移=小步传送精确；车轮转向只在仿真刚启动时可靠"],
    ["完整流程+分类箱", "✅ 完成", "full_pipeline_demo.py：6/6 全部按颜色放入对应箱，误差 0.0000 m"],
])

H(1, "3. 各模块关键结论（务必以本场景实测为准，勿照搬 Lua 参考数值）")
H(2, "3.1 夹爪")
L("真实路径：/RoboMaster/.../Force_sensor/gripper_link_respondable/Prismatic_joint")
L("故障根因：Prismatic_joint 是动态关节（电机开、无位置环），且 gripper_link_respondable 上挂原厂控制器脚本(h≈91)，每步用 setJointTargetVelocity 接管 → setJointPosition/setJointTargetPosition 无效，位置被钉在 ~0.0007（=控制器 open 上限 0.00098）。")
L("正确控制：给 gripper_link_respondable 写 int 属性信号：signal.target=1(OPEN)/2(CLOSE)/0(pause)，轮询 signal.state 到一致为止。")
L("关节区间 [-0.023, 0.024]；open≈+0.001、close≈-0.023（与 Lua 的 0.05/-0.025 语义同，但会被截断，无需自填数值）。")
L("闭合时若两指间有“非静态+可碰撞”物体，控制器会接近传感器检测并自动 setObjectParent 挂到 attachPoint（打印 Attached shape）。Grid1 太宽不会触发（见 3.4）。")

H(2, "3.2 机械臂")
table(["关节", "作用(本场景)", "有效区间", "控制方式"], [
    ["servo_motor_0 (s0)", "主升降：负角抬升、正角下降（实测 -25°→Z0.209→0.259；+30°→Z0.163）", "约[-46°,+60°]", "setJointTargetPosition 直接可用（位置环开着）"],
    ["servo_motor_1 (s1)", "前伸/俯仰微调：正角前伸略降", "约[-15.7°,+95°]", "同上"],
])
P("注意：本场景 s0/s1 分工与“Lua 参考场景”相反（那里是 s1 抬降）。每次动作后应等待 |实际角-目标角|<1° 再继续。home=(0,0)，夹爪根部 Z≈0.2086 可精确复现。")

H(2, "3.3 机械臂+夹爪协同")
L("顺序已跑通：home+OPEN → s0=+30 下降 → CLOSE(手指间距→0.030) → s0=-25 抬升(保持闭合) → OPEN → 回 home。文件：arm_gripper_sync_test.py。")

H(2, "3.4 Grid1“跟随式假抓取”")
L("原因：Grid1/所有 Grid 都是 OBJECT_DIAMETER=0.12m 的批次，夹爪张开最大间距约 0.117m，物理夹不住 → 用 Lua 成功方案同款：物体跟随夹爪中心+offset。")
L("方法：下降摆位后，offset = Grid1中心 − 手指中点(left/right_gripper_5_respondable 均值)；搬运期间每 ~0.05s 执行 setObjectPosition(Grid1, 手指中点+offset)；放下=解除跟随并把 Z 设 0.11。搬运期间夹爪保持 OPEN（闭合会让手指中点漂移）。")
L("实测：抬升 +9.5cm、前伸 X 0.267→0.353、放下回 Z0.11 全部正常。文件：grid1_follow_test.py、demo.py（三段连播演示）。")

H(2, "3.5 小车移动")
table(["方式", "结果", "备注"], [
    ["车轮物理驱动（4全向轮）", "平移3s/1.13m；差速原地转2s/198°位移<1cm；对角可横移", "真实但全向轮打滑非线性"],
    ["平移=小步传送 setObjectPosition 插值（不要每步 resetDynamicObject）", "精确到毫米、朝向不变", "推荐用于流程"],
    ["车轮闭环转向（起点刚启动仿真时）", "1.3s 转 90°，漂移 1mm", "推荐；只在未传送过时可靠"],
    ["传送跑远后再车轮转向", "❌ 卡滞打滑", "已知 bug，未解决（见第 8 节）"],
    ["setObjectOrientation 直接转 / 悬空转 / pause 传送", "❌ 被动力学/插件干扰", "别用"],
])
L("完整流程策略：用固定初始朝向(≈106.9°)＋平移传送导航；不依赖运行中转向。")

H(2, "3.6 完整流程（分类整理 v2）—— 主线已完成")
L("脚本 full_pipeline_v2.py（无录制）/ full_pipeline_recorded_v2.py（带录制出视频）：对每个物体执行 归位→开车到物体前→夹爪闭合抓取→抬升→开车到料盒外侧→手臂伸入→放到箱内水平线→夹爪张开→（全部完成后）回起点。")
L("【放置规则（按要求）】箱内物品全部放在同一条水平线 x = 0.380，间距 0.15，高度 z = 0.13：")
L("     LeftBin  三个位置 y = -1.05 / -0.90 / -0.75     RightBin 三个位置 y = +0.75 / +0.90 / +1.05")
L("【放置顺序（按要求，从左到右依次）】Grid1→LeftBin 最左(-1.05)、Grid2→LeftBin 中间(-0.90)、Grid3→LeftBin 最右(-0.75)、")
L("     Grid4→RightBin 最左(+0.75)、Grid5→RightBin 中间(+0.90)、Grid6→RightBin 最右(+1.05)，最后小车返回起点。")
L("【直角走位 + 底盘不碰料盒（v2.1 已加强）】按要求改为“只走轴向直线、绝不斜走”：")
L("     循环：① 平移到物体所在行 → ② 前移一点进入抓取位 → ③ 夹爪闭合抓取 → ④ 抬到高位 → ⑤ 直线后退到安全通道 →")
L("           ⑥ 平移到料盒前方 → ⑦ 手臂高位跨过箱壁 → ⑧ 箱内下降 → ⑨ 轻放到 x=0.380 水平线 → ⑩ 张开夹爪并收臂。")
L("     安全通道 X_SAFE=0.015：车体 x=0.015、车头前缘 0.205，料盒左壁外表面 0.275（壁厚 0.02）→ 余量 7cm，每次摆放都打印验证 ✅")
L("     物品跨壁时用高位姿态（物品底部 z≈0.105 > 箱壁顶 0.08），从箱壁上方越过再下降，物品本身也不穿模。")
L("【夹爪全程真实动作】抓取前 CLOSE（signal.target=2）、放下后 OPEN（=1）；物体与手指的 offset 在闭合后记录，避免漂移。")
L("实测结果：6/6 全部到位，末端误差 0.0000 m（同一水平线、间距 0.15、底盘未碰料盒），机器人自动返回起点。")
L("重要前提：场景中 Grid、料盒都是 static=1/respondable=0 的视觉件，互不碰撞 → 流程稳定，不会物理爆炸。")

H(1, "4. 已知的坑（务必记住）")
L("① CoppeliaSim 停止仿真：物体位置会复位到存档，但 static/respondable 属性不会恢复！凡改过物理属性的物体（如 Grid1），下次运行前必须先显式重设。")
L("② 模型子脚本必须保持启用（夹爪控制器靠它跑）；/RoboMaster 子脚本含 simRobomaster.create_ep，自己代码不要再 create_ep。")
L("③ 仿真可能被“非凸动态形状”警告拖慢甚至卡死：曾因把车传送/开进物体堆导致物理爆炸、ZMQ 挂起。症状=脚本无输出超时。处理：杀 python 进程 + 重启 CoppeliaSim（场景自动复位），不要硬等。")
L("④ /LeftBin 等“路径”不存在；分类箱是 LeftBin_bottom/front/back/left/right、RightBin_* 一组零件形状。")
L("⑤ ZMQ 请求被孤儿 python 阻塞会卡死：进程列表中若有多余 python 先杀掉。")

H(1, "5. 项目文件清单（重要）")
table(["文件", "类型", "说明"], [
    ["gripper_min_test.py", "✅交付", "夹爪最小真实开合测试"],
    ["arm_min_test.py", "✅交付", "机械臂最小升降测试"],
    ["arm_gripper_sync_test.py", "✅交付", "机械臂+夹爪协同测试"],
    ["grid1_follow_test.py", "✅交付", "Grid1 跟随式抓取-搬运-放下"],
    ["chassis_min_test.py", "✅交付", "小车移动测试（平移传送+起点车轮转向）"],
    ["demo.py", "✅交付", "夹爪→机械臂→Grid1 三段连播演示"],
    ["full_pipeline_demo.py", "旧版(主线v1)", "完整分类整理流程 v1：放入箱内 x=0.37/0.50/0.63（车体会进箱）"],
    ["full_pipeline_v2.py", "✅交付(主线v2)", "v2：箱内统一 x=0.380 水平线、间距 0.15、底盘不进料盒"],
    ["full_pipeline_recorded_v2.py", "✅交付(演示)", "v2 流程 + 内部相机录制，直接产出演示视频"],
    ["实验三_视觉分类演示_正面 / _俯视 / _侧面.mp4", "✅成果", "视觉分类演示 3 个视角（正面/俯视/侧面，6 物体全流程）：含视觉决策 + 夹爪第一视角画中画 + 左上全程夹爪摄像头"],
    ["实验三_视觉分类演示_正面_位置4无物体_自动跳过.mp4", "✅成果", "漏检跳过演示（正面，单独文件）：位置4 物体已移除 → 夹爪相机判断无物体则跳过不抓取，其余 5 个正常分类放置"],
    ["实验三_直角走位演示_正面 / _俯视.mp4", "✅成果", "不含视觉模块的对照版本（直角走位抓放）"],
    ["实验三_旧版_斜线走位演示_侧面.mp4", "旧版留档", "早期斜线走位版本（对比用）"],
    ["演示视频清单.md", "✅文档", "所有演示视频的 演示内容/拍摄视角/时长/备注 一览（随视频同目录）"],
    ["add_grid.py", "✅工具", "在物体下方绘制地面黑色正方形网格（最外侧边缘避开料盒 7.5cm）"],
    ["vision_config.py / camera_node.py / vision_calibrate.py", "✅视觉", "视觉配置、相机节点、相机标定节点"],
    ["detector_node.py / gripper_camera.py", "✅视觉", "识别节点（类别/检测框/置信度）、夹爪第一视角相机"],
    ["vision_pipeline_recorded.py", "✅视觉", "视觉决策 + 抓取 + 录制（含画中画）主流程"],
    ["vision_test.py / verify_vision_video.py", "✅视觉", "视觉链路自检、演示视频验收"],
    ["recolor_classes.py / rebuild_shapes.py", "✅场景", "按类别规则改色 / 重建几何（1/3/5 蓝棱柱、2/4/6 红圆柱）"],
    ["实验三_视觉识别结果.jpg / 实验三_夹爪视角识别结果.jpg", "✅成果", "俯视识别标注图 / 夹爪第一视角识别标注图（桌面）"],
    ["set_bins_x.py", "✅工具", "把左右料盒 x 平移到 0.50 并保存场景"],
    ["probe_reach2.py", "✅工具", "量车体尺寸与机械臂伸展范围（v2 姿态参数的依据）"],
    ["second_imitation_backup_*.ttt", "备份", "料盒改坐标前的原始场景备份"],
    ["record_demo.py", "备用", "屏幕录制脚本（窗口抓屏版，已被内部相机方案取代）"],
    ["诊断结论_夹爪不动.md / 诊断结论_机械臂.md / 诊断结论_Grid1抓取.md / 诊断结论_小车移动.md", "✅文档", "各里程碑结论"],
    ["ep_gripper_test.py / ep_grid1_pick_test.py 等旧脚本", "⏳旧版", "旧控制方式已废弃，勿直接用"],
    ["diag_*.py / probe*.py / _script_*.lua / *_out.txt", "过程", "排查脚本与日志，可删"],
    ["本文件", "✅文档", "明日接力依据"],
])

H(1, "6. 下一步可做的优化（主线 6 步已全部完成）")
L("① 演示与归档：演示视频共 6 个，统一放在桌面文件夹《实验三_演示视频》并附《演示视频清单.md》：")
L("     视觉分类演示 3 视角（正面/俯视/侧视，各约 91s，含视觉决策 + 夹爪第一视角画中画）；")
L("     直角走位演示 2 视角（正面/俯视，不含视觉模块的对照版）；旧版斜线走位演示 1 个（留档）。")
L("     重录方式：python vision_pipeline_recorded.py <front|top|side> <输出mp4>（机位参数在脚本顶部 CAM_PRESETS）。")
L("② 报告材料：把 5 份《诊断结论_*.md》与演示视频整理成实验报告/答辩 PPT。")
L("③ 稳定性复跑：连续跑 3 遍 full_pipeline_demo.py，确认每次都是 6/6（脚本末尾已带结果自检表）。")
L("④ 视觉定位替代硬编码坐标：场景内有 Vision_sensor / Machine_vision_sensor 与 RoboMaster 相机模块，可用颜色识别自动得到 Grid 位置（当前流程用的是固定坐标）。")
L("⑤ 真实物理抓取（可选、较难）：需换更小的物体，或改夹爪开口（当前物体 0.12m > 夹爪开口 0.117m，物理夹不住，故用跟随式假抓取）。")
L("⑥ 转向问题（见第 7 节第 1 条）：若需在流程中转向，建议把转向安排在仿真刚启动、未做任何传送之前完成。")
H(2, "6.1 如何运行（明日复现步骤）")
L("① 启动 CoppeliaSim 载入 second_imitation.ttt，确认 ZMQ 23000 通；")
L("② 快速自检：python gripper_min_test.py（夹爪应能开合）→ python arm_min_test.py（臂应升降）；")
L("③ 跑主线：python full_pipeline_demo.py（约 3~4 分钟，看 6 个物体依次被放入左右分类箱）。")

H(1, "7. 遗留问题清单（按优先级）")
table(["#", "问题", "建议"], [
    ["1", "传送跑远后再用车轮转向会打滑卡滞", "把转向放在流程开始（未传送前）；或先停 2~3s 低速点动“松卡”；当前主线靠固定朝向+平移传送绕开"],
    ["2", "防物理卡死", "脚本已用 try/finally + stopSimulation；若仍卡死：杀多余 python 进程 + 重启 CoppeliaSim"],
    ["3", "料盒无整体对象（LeftBin 不存在）", "料盒已整体移到 x=0.50；箱内投放点固定为 x=0.380、y=箱中心±0.15、z=0.13（v2 脚本内置）"],
    ["4", "颜色/形状分类规则是硬编码的物体名", "若要通用化，需接入视觉识别（见 6 ④）"],
    ["5", "Grid 位置为固定坐标", "换场景/移动物体后需重新量取，或改为视觉定位"],
])

P("—— 以上内容均由当日实测获得，数值与接口以本场景为准。——", color=RGBColor(0x88, 0x88, 0x88))

doc.save(OUT)
print("saved:", OUT)
