"""Build the bilingual Lab6 PDF reports and measured RPE plots."""
import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from xml.sax.saxutils import escape

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import Paragraph, Table, TableStyle

root = Path(__file__).resolve().parent.parent
ap = argparse.ArgumentParser()
ap.add_argument("--results", default="office_preview")
ap.add_argument("--full", action="store_true")
args = ap.parse_args()
results = root / "results" / args.results
charts = root / "results" / "charts"
charts.mkdir(parents=True, exist_ok=True)
shots = root / "results" / "screenshots"
reports = root / "reports"
reports.mkdir(exist_ok=True)
methods = ("NISTER5", "EIGHT8", "KNOWN2", "ARUN3", "NISTER5_RAW")
names = ("Nistér 5pt", "8pt", "Known-R 2pt", "Arun 3pt", "5pt without RANSAC")
series = {m: [] for m in methods}
with (results / "rpe.csv").open() as file:
    for row in csv.DictReader(file):
        series[row["method"]].append(row)
assert all(len(rows) == len(series[methods[0]]) for rows in series.values())
pairs = len(series[methods[0]])
assert pairs > 0
start = float(series[methods[0]][0]["time_s"])
end = float(series[methods[0]][-1]["time_s"])
colorset = ["#2765ab", "#d07220", "#2c8b69", "#734c9e", "#8a929b"]

def vals(rows, key):
    return [v for r in rows if math.isfinite(v := float(r[key]))]

def med(data):
    return f"{statistics.median(data):.4g}" if data else "—"

summary = []
for m, name in zip(methods, names):
    rows = series[m]
    ok = sum(r["status"] == "ok" for r in rows)
    rot = vals(rows, "rotation_deg")
    trans = vals(rows, "translation_rpe")
    summary.append([name, f"{ok}/{pairs}", str(len(trans)), med(rot), med(trans),
                    med(vals(rows, "inliers")), med(vals(rows, "solve_ms"))])
with (results / "summary.csv").open("w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["method", "solved_over_pairs", "translation_samples",
                "median_rotation_deg", "median_translation_rpe", "median_inliers", "median_solver_ms"])
    w.writerows(summary)

def plot(file, ids, key, ylabel, title):
    fig, ax = plt.subplots(figsize=(9, 3.2), layout="constrained")
    for i in ids:
        rows = series[methods[i]]
        ax.plot([float(r["time_s"]) for r in rows], [float(r[key]) for r in rows],
                label=names[i], color=colorset[i], lw=0.9, alpha=.9)
    ax.set(title=title, xlabel="Time from first evaluated image (s)", ylabel=ylabel)
    if key == "rotation_frobenius" and len(ids)>1:
        ax.set_yscale("symlog", linthresh=0.005)
    ax.grid(alpha=.22)
    ax.legend(ncol=2 if len(ids)>1 else 1, fontsize=8)
    fig.savefig(charts/f"{file}.png", dpi=180)
    fig.savefig(charts/f"{file}.pdf")
    plt.close(fig)

plot("05_2D_Rotation_RPE",(0,1,2,4),"rotation_frobenius","Chordal distance","2D–2D rotation error")
plot("06_2D_Translation_RPE",(0,1,2,4),"translation_rpe","Unit-direction distance","2D–2D translation direction error")
plot("07_3D_Rotation_RPE",(3,),"rotation_frobenius","Chordal distance","Arun 3D–3D rotation error")
plot("08_3D_Translation_RPE",(3,),"translation_rpe","Metric error (m)","Arun 3D–3D translation error")

fonts = Path("C:/Windows/Fonts")
pdfmetrics.registerFont(TTFont("Song", str(fonts / "simsun.ttc"), subfontIndex=0))
pdfmetrics.registerFont(TTFont("Hei", str(fonts / "simhei.ttf")))
navy = colors.HexColor("#173854")
muted = colors.HexColor("#516b7f")
ink = colors.HexColor("#243645")
line = colors.HexColor("#c9d9e7")

def render(en):
    def t(cn, english): return english if en else cn
    file = t("LAB6_无人机视觉定位实验报告.pdf", "LAB6_Visual_Localization_Report_EN.pdf")
    c = Canvas(str(reports / file), pagesize=(595.28,841.89))
    c.setTitle(t("LAB6 无人机视觉定位实验报告","Lab6 UAV Visual Localization"))
    c.setAuthor("Boyu Huang")
    font, heading = t("Song","Helvetica"), t("Hei","Helvetica-Bold")
    y, page = 0, 0
    def text(value, size=11, lead=17, bold=False, color=ink):
        nonlocal y
        s=ParagraphStyle("body",fontName=heading if bold else font,fontSize=size,
                         leading=lead,textColor=color,wordWrap=None if en else "CJK",
                         alignment=TA_LEFT)
        p=Paragraph(escape(str(value)),s)
        _,h=p.wrap(493,1000)
        if y-h < 54: raise RuntimeError((file,page,y,h,value))
        p.drawOn(c,51,y-h)
        y-=h+9
    def page_start(title, subtitle):
        nonlocal y,page
        if page: c.showPage()
        page+=1
        c.setFillColor(navy)
        c.rect(0,785,595.28,56.89,fill=1,stroke=0)
        c.setFont("Helvetica-Bold",12)
        c.setFillColor(colors.white)
        c.drawString(51,810,"VNAV / LAB 06")
        c.setFont("Helvetica",9)
        c.drawRightString(544,810,"VISUAL LOCALIZATION")
        c.setStrokeColor(line)
        c.line(51,49,544,49)
        c.setFillColor(muted)
        c.setFont("Helvetica",9)
        c.drawString(51,33,"Boyu Huang  /  2026-11-09")
        c.drawRightString(544,33,f"{page:02d}")
        y=756
        text(title,23,30,True,navy)
        text(subtitle,10,15,False,muted)
        y-=4
    def table(rows, widths):
        nonlocal y
        style=ParagraphStyle("cell",fontName=font,fontSize=9,leading=14,
                            wordWrap=None if en else "CJK",textColor=ink)
        data=[[Paragraph(escape(str(v)),style) for v in row] for row in rows]
        tab=Table(data,colWidths=widths,hAlign="LEFT")
        tab.setStyle(TableStyle([
            ("BACKGROUND",(0,0),(-1,0),colors.HexColor("#dceaf4")),
            ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.white,colors.HexColor("#f2f6fa")]),
            ("LINEBELOW",(0,0),(-1,0),.6,line),
            ("LINEBELOW",(0,-1),(-1,-1),.5,line),
            ("VALIGN",(0,0),(-1,-1),"TOP"),
            ("TOPPADDING",(0,0),(-1,-1),6),
            ("BOTTOMPADDING",(0,0),(-1,-1),6)
        ]))
        _,h=tab.wrap(493,1000)
        if y-h<54: raise RuntimeError((file,page,"table overflow"))
        tab.drawOn(c,51,y-h)
        y-=h+13
    def figure(p, caption, max_height=200):
        nonlocal y
        with Image.open(p) as im: w,h=im.size
        scale=min(493/w,max_height/h)
        w,h=w*scale,h*scale
        if y-h<70: raise RuntimeError((file,page,"image overflow"))
        c.drawImage(str(p),51+(493-w)/2,y-h,width=w,height=h)
        y-=h+5
        text(caption,9,13,False,muted)

    page_start(t("无人机视觉定位","UAV visual localization"),
               t("中英文实验报告 · ROS 1 Noetic · 个人推导与团队实现",
                 "Experiment report · ROS 1 Noetic · Individual derivations and team implementation"))
    text(t("作者：黄博宇 / Boyu Huang","Author: Boyu Huang"),12,18,True)
    text(t("实验估计无人机在相邻相机画面之间如何转动、向哪里移动。先复用 Lab5 的 SIFT 配对，再用 OpenGV 求位姿，并与仿真真值比较。","The experiment estimates how the camera rotates and moves between consecutive frames. It reuses Lab5 SIFT matches, solves pose with OpenGV, and evaluates it against simulator ground truth."))
    text(t(f"数据范围：MIT Office ROS 2 归档的连续片段，转换为 ROS 1。评估 {pairs} 对画面，末帧距首帧 {end:.2f} 秒。完整版尚未评估。",
           f"Data scope: a contiguous prefix of the MIT Office ROS 2 archive, converted to ROS 1. Evaluation covers {pairs} pairs up to {end:.2f} s from the first image. Full-sequence evaluation is pending.") if not args.full else
         t(f"数据范围：转换后的完整 MIT Office 归档，共评估 {pairs} 对画面。",f"Data scope: the converted full MIT Office archive, with {pairs} evaluated pairs."))
    table([t(["交付项","分值","本报告"],["Deliverable","Points","Coverage"]),
           t(["D1 五点法","10","求解步骤、退化与 RANSAC 次数"],["D1 Five-point solver","10","Steps, degeneracy, and RANSAC count"]),
           t(["D2 最小求解器","10","已知旋转的两点推导与 RANSAC 伪代码"],["D2 Minimal solver","10","Known-rotation derivation and RANSAC"]),
           t(["D3 环境与标定","10","ROS、数据话题、K 和深度"],["D3 Setup","10","ROS, topics, intrinsics, and depth"]),
           t(["D4 二维对应","40","五点、八点、两点、无 RANSAC 对照"],["D4 2D–2D","40","5pt, 8pt, 2pt, plus no-RANSAC baseline"]),
           t(["D5 三维对应","30","Arun + RANSAC、米制误差"],["D5 3D–3D","30","Arun + RANSAC and metric error"])],[150,54,289])
    text(t("分值按本地课程说明。团队检查可用演示脚本播放真实 RGB、深度和估计位姿。","Points follow the local course rubric. For team inspection, the demo replays measured RGB, depth and estimated poses."))

    page_start(t("D1 · Nistér 五点法","D1 · Nistér's five-point solver"),t("五对像点约束一个相对姿态","Five correspondences constrain a relative pose"))
    text(t("约定 R 和 t 把第二相机坐标变到第一相机。对去畸变的单位射线 x1、x2，有 x1^T E x2 = 0，其中 E = [t]_cross R。旋转有 3 个自由度，平移方向有 2 个。","Use R and t to map camera 2 into camera 1. For calibrated unit bearings x1 and x2, x1^T E x2 = 0 with E = [t]_cross R. Rotation has three degrees of freedom and translation direction has two."))
    table([t(["步骤","计算"],["Stage","Computation"]),
           t(["1. 找零空间","把五条对极约束堆叠成 5×9 矩阵。一般秩为 5，E 写成四个零空间基矩阵的线性组合。"],["1. Null space","Stack the epipolar constraints into a 5×9 matrix. With rank 5, express E using four null-space basis matrices."]),
           t(["2. 加本质矩阵约束","消去整体尺度后，代入 det(E)=0 与 2EE^TE-tr(EE^T)E=0。用消元得到一元十次多项式。"],["2. Essential constraints","Fix the global scale and impose det(E)=0 and 2EE^TE-tr(EE^T)E=0. Eliminate variables to obtain a degree-ten polynomial."]),
           t(["3. 选物理解","保留实根并回代。分解 E 取得候选 R、±t；用三角化正深度筛选，再用额外匹配给候选打分。"],["3. Physical pose","Back-substitute real roots. Decompose E into candidate rotations and ±t, check positive triangulated depth, then score on further correspondences."])],[139,354])
    text(t("退化：纯旋转时射线经 R 对齐，平移方向没有可观测约束；点近乎共线或视差极小时也难以稳定估计。平面场景可以得到有限解，但可能存在多个相容候选，不能把共面一概判为无解。[2]","Degeneracy: pure rotation aligns rays after R and leaves translation direction unobservable. Nearly collinear points or very small parallax make estimates unstable. A plane can yield finite solutions, sometimes with multiple compatible candidates. [2]"))
    text(t("设内点率 w，每次独立抽取五点。成功概率约为 w^5，首次抽中无外点样本的期望次数为 1/w^5。要达到置信度 p，需 N = ceil(log(1-p) / log(1-w^5))。w=0.7、p=0.99 时，期望约 5.95 次，预算 26 次。OpenGV 的 NISTER 包装额外取 3 对用于候选分解和消歧，实际抽样量为 8。","For independent five-point draws at inlier fraction w, the success probability is about w^5 and the expected wait is 1/w^5. At confidence p, N = ceil(log(1-p) / log(1-w^5)). For w=0.7 and p=0.99, the mean is 5.95 and the budget is 26. The OpenGV NISTER wrapper adds three pairs for decomposition and disambiguation, so it draws eight."))

    page_start(t("D2 · 已知旋转的两点法","D2 · Two points with known rotation"),
               t("两条约束确定平移方向","Two constraints determine the direction of translation"))
    text(t("IMU 给定 R 后，令 n_i = (Rx2_i) × x1_i，则 n_i^Tt = 0。每对射线确定一个可能包含平移的平面。两平面相交得到平移直线：t = ±(n1 × n2) / ||n1 × n2||。","With R supplied by the IMU, set n_i = (Rx2_i) × x1_i. Each ray pair gives n_i^Tt = 0, a plane containing translation. The two planes intersect along t = ±(n1 × n2) / ||n1 × n2||."))
    text(t("两法向量须独立；零视差或法向量平行时拒绝该样本。用正深度选择符号。多内点精化可用法向量矩阵最小奇异值对应的右奇异向量。","The normals must be independent. Reject zero-parallax or parallel-normal samples. Use cheirality to select the sign. A refinement can take the right singular vector of the inlier-normal matrix with the smallest singular value."))
    table([t(["RANSAC","步骤"],["RANSAC","Procedure"]),
           t(["抽样","随机抽两对，拒绝病态叉积，形成候选方向。"],["Sample","Draw two matches, reject degenerate normals, form a direction."]),
           t(["评分","用三角化与两视图射线重投影代价给所有匹配打分；低于阈值即内点。"],["Score","Triangulate and use bearing reprojection cost; count residuals below the threshold."]),
           t(["保留","按内点数选最佳模型，同分时可比较残差。用 w^2 更新置信预算。"],["Keep","Keep the largest consensus set; use residuals to break ties if needed. Update the budget with w^2."]),
           t(["输出","返回最优方向和内点，必要时以全部内点精化。"],["Return","Return the best direction and inliers, optionally refit using all inliers."])],[120,373])
    text(t("代码里的两点法使用真值相对旋转模拟已知 IMU 旋转。旋转误差接近零来自这个输入条件，不能用来证明视觉算法估出了旋转。","The two-point experiment uses ground-truth relative rotation as the known IMU rotation. Its near-zero rotation error follows from this input."))

    page_start(t("D3–D5 · 数据和实现","D3–D5 · Data and implementation"),
               t("从像素到相对变换","From pixels to relative transforms"))
    table([t(["环节","实现与参数"],["Stage","Implementation and parameters"]),
           t(["读取","Ubuntu 20.04、ROS Noetic、OpenCV 4.2 contrib、OpenGV。读取左 RGB、注册深度、CameraInfo 与真值里程计。"],["Input","Ubuntu 20.04, ROS Noetic, OpenCV 4.2 contrib, OpenGV. Read left RGB, registered depth, CameraInfo and ground-truth odometry."]),
           t(["标定","fx=fy=415.6922，cx=360，cy=240，畸变为零。undistortPoints 后将 (x,y,1) 归一化为单位射线。"],["Calibration","fx=fy=415.6922, cx=360, cy=240, zero distortion. Undistort and normalize (x,y,1) to unit bearings."]),
           t(["对齐","用时间戳匹配深度，容差 15 ms。位姿平移线性插值，旋转用四元数 SLERP。加上题面给定的机体到相机外参。"],["Alignment","Match depth timestamps within 15 ms. Interpolate odometry translation linearly and rotation by SLERP. Apply the starter camera-to-body extrinsic."]),
           t(["2D–2D","沿用 Lab5 SIFT、FLANN、0.8 比例过滤。OpenGV Nistér、8 点和已知旋转 2 点用 RANSAC，双视图代价阈值 5e-6。"],["2D–2D","Reuse Lab5 SIFT, FLANN and the 0.8 ratio test. OpenGV Nistér, eight-point and known-rotation two-point use RANSAC, threshold 5e-6 in the two-view cost."]),
           t(["3D–3D","用 Z 深度乘以 (x,y,1) 反投影；过滤越界及无效深度。Arun 对中心化三维点做 SVD、修正反射并恢复米制平移。RANSAC 阈值 0.04 m。"],["3D–3D","Back-project with Z depth and reject invalid samples. Arun aligns centered 3D points by SVD, corrects reflections and recovers metric translation. RANSAC threshold 0.04 m."]),
           t(["评估与可视化","比较 inv(T上一帧) · T当前帧。旋转用矩阵 Frobenius 距离，二维平移用单位方向距离，三维平移用米。RViz 用上一帧真值锚定，二维显示另用真值尺度。"],["RPE and RViz","Compare previous-to-current relative transforms. Rotation uses Frobenius distance; 2D translation uses unit directions and 3D uses meters. RViz is anchored to the previous ground-truth pose; 2D display also uses its scale."])],[117,376])
    text(t("真值基线小于 1 mm 时不报告二维平移方向误差。求解失败写入 CSV，曲线保留缺口。RANSAC 上限 300 次，置信度 0.99。无 RANSAC 对照取描述子距离最小的一组，只形成一次假设。","Skip 2D direction error below a 1 mm ground-truth baseline. Failures remain in the CSV as gaps. RANSAC uses up to 300 iterations at 0.99 confidence. The baseline scores one hypothesis from the lowest descriptor-distance matches."))

    page_start(t("D4 · 二维对应结果","D4 · 2D correspondence results"),
               t("三个 RANSAC 方法和一次假设对照","Three RANSAC methods and a single-hypothesis baseline"))
    figure(charts/"05_2D_Rotation_RPE.png",t("图 05  二维方法的旋转矩阵距离。","Fig. 05  Chordal rotation error for the 2D methods."),181)
    figure(charts/"06_2D_Translation_RPE.png",t("图 06  二维平移方向误差，数值范围为 0–2。","Fig. 06  Translation direction error, ranging from 0 to 2."),181)
    table([t(["方法","成功对数","方向样本","旋转中位数 °","方向中位数"],["Method","Solved","Dir. samples","Median rot. °","Median dir."])]
          +[r[:5] for i,r in enumerate(summary) if i!=3],[145,70,73,95,110])
    text(t("低视差时，两个方向很容易因少量像素噪声而拉开；二维方向误差与米制误差应分别解读。","With low parallax, small pixel errors can produce large direction errors. Interpret direction and metric errors in their respective units."),10,15)

    page_start(t("D5 · 三维对应结果","D5 · 3D correspondence results"),
               t("Arun + RANSAC 恢复米制平移","Arun + RANSAC recovers metric translation"))
    figure(charts/"07_3D_Rotation_RPE.png",t("图 07  Arun 的旋转矩阵距离。","Fig. 07  Arun chordal rotation error."),181)
    figure(charts/"08_3D_Translation_RPE.png",t("图 08  Arun 的米制相对平移误差。","Fig. 08  Arun metric relative translation error."),181)
    r=summary[3]
    table([t(["成功对数","旋转中位数 °","平移中位数 m","内点中位数","求解中位数 ms"],
             ["Solved","Median rot. °","Median trans. m","Median inliers","Median solve ms"]),
           [r[1],r[3],r[4],r[5],r[6]]],[84,102,104,97,106])
    text(t("深度让同一个像点变成有单位的空间点。可逐帧累乘米制变换，但局部误差会积累，本实验的 RViz 显示采用真值锚定的单步位姿。","Depth turns pixels into points with a metric scale. Metric transforms can be composed across frames, but drift accumulates. The RViz display shows ground-truth-anchored single-step poses."),10,15)

    for group in ((0,1),(2,3)):
        page_start(t("RViz 实际运行截图","RViz replay screenshots"),
                   t("左侧显示深度和 RGB，中央显示估计和真值相机位姿。","Depth and RGB are on the left; estimated and reference poses are in the center."))
        for i in group:
            figure(shots/f"{i+1:02d}_RViz_{methods[i]}.png",
                   t(f"图 {i+1:02d}  {names[i]} 的 Office 片段回放。",
                     f"Fig. {i+1:02d}  {names[i]} replay of the Office prefix."),254)

    page_start(t("结果核对与引用","Verification and references"),
               t("复现脚本、测量口径与限制","Reproduction, metrics and limitations"))
    table([t(["结果","用途"],["Artifact","Purpose"]),
           t(["rpe.csv","逐帧对误差、匹配数、内点数、真值基线、求解耗时。"],["rpe.csv","Per-pair errors, match counts, inliers, baseline and solver timing."]),
           t(["summary.csv","有效对数与中位数。耗时只计位姿求解，包含 RANSAC。"],["summary.csv","Valid counts and medians. Timing covers pose solving and RANSAC."]),
           t(["演示操作文档","Ubuntu 上运行 RViz、切换方法、重新计算及查看结果。"],["Demo guide","Run RViz, switch methods, recompute and inspect output."])],[140,353])
    text(t("数据范围限制：2023 题面里的旧 Google Drive 文件返回 404。本次使用同课程新版提供的 ROS 2 Office 归档，先转换了可读取的连续前段。整段数据的性能与题面预期仍需用完整归档复核。Lab4 RGB-D 补充采集在旧仿真器的深度相机信息解析处失败，日志单独保存。","Scope: the original Google Drive file returned 404. The current course ROS 2 Office archive supplied the contiguous prefix used here. Full-sequence performance still needs validation. An optional Lab4 RGB-D recording failed during depth camera metadata parsing in the older simulator; its log is retained."))
    text(t(f"本次二维方向误差中位数：五点 {summary[0][4]}，八点 {summary[1][4]}，已知旋转两点 {summary[2][4]}。无 RANSAC 对照为 {summary[4][4]}。Arun 米制平移误差中位数为 {summary[3][4]} m。不同尺度的平移误差不作直接排名。",f"Median 2D direction errors were {summary[0][4]} (5pt), {summary[1][4]} (8pt), {summary[2][4]} (known rotation) and {summary[4][4]} (no RANSAC). Arun's median metric error was {summary[3][4]} m. The translation metrics use different units."),10,14)
    refs=[
      ("[1] MIT VNAV, Lab 6 Exercises (ROS 1).","https://vnav.mit.edu/labs_2023/lab6/exercises.html"),
      ("[2] D. Nistér, An Efficient Solution to the Five-Point Relative Pose Problem, CVPR 2003.","https://www-users.cse.umn.edu/~hspark/CSci5980/nister.pdf"),
      ("[3] OpenGV, library and sample-consensus implementation.","https://github.com/ethz-asl/opengv"),
      ("[4] MIT VNAV, Office dataset (ROS 2 archive).","https://drive.google.com/file/d/1TohOnfjR_FKf_y9wYTEkmv-XCuFPFRPt/view"),
      ("[5] MIT-SPARK, VNAV Lab starter code.","https://github.com/MIT-SPARK/VNAV-labs")]
    for label,url in refs:
        text(label,10,14,True)
        text(url,8,11,False,muted)
    c.save()
    print(file,page,"pages")
for language in (False,True): render(language)
