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
ap.add_argument("--results", default="office_extended")
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
        c.drawString(51,33,"Boyu Huang  /  2026-11-10")
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

    figure(shots/"01_RViz_NISTER5.png",
           t("实测预览（见图 01）。RViz 左侧是 Office 深度与 RGB，右侧是相机位姿。",
             "Experiment preview (Fig. 01). RViz shows Office depth and RGB on the left and camera poses on the right."),215)

    page_start(t("实验目标与阅读路线","Experiment goals and reading guide"),
               t("从图像对应走到可检查的相机位姿","From image correspondences to verifiable camera poses"))
    text(t("核心问题是估计相邻两帧的相机运动。SIFT 回答两个像点是否可能来自同一处场景，位姿求解器再利用这些对应关系判断相机怎么动。","The goal is to recover camera motion between adjacent images. SIFT proposes which pixels represent the same scene point, then the pose solver uses those correspondences to estimate the motion."))
    table([t(["阶段","输入","输出"],["Stage","Input","Output"]),
           t(["配对","前后两张 RGB 图","像素坐标对与描述子距离"],["Match","Two RGB frames","Pixel pairs and descriptor distances"]),
           t(["标定","像素位置与内参 K、D","归一化的相机视线"],["Calibrate","Pixel locations and K, D","Normalized camera bearings"]),
           t(["求解","视线，或有深度的三维对应","相对旋转、平移方向或米制平移"],["Solve","Bearings, or depth-backed 3D pairs","Relative rotation and translation direction or metric translation"]),
           t(["评估","相对估计和里程计参考","逐帧 RPE、求解状态与时间"],["Evaluate","Estimate and odometry reference","Per-pair RPE, solver status and timing"])],[98,159,236])
    table([t(["内容","页码"],["Content","Pages"]),
           t(["任务、数据流程、坐标约定、算法比较","1–4"],["Tasks, data flow, coordinate convention and algorithm comparison","1–4"]),
           t(["D1、D2 理论推导与采样分析","5–7"],["D1 and D2 derivations and sampling analysis","5–7"]),
           t(["环境、数据校验、工程实现","8–10"],["Setup, input checks and implementation","8–10"]),
           t(["二维与三维结果、误差分布与对照","11–14"],["2D and 3D results, error distributions and comparison","11–14"]),
           t(["复现流程、RViz 与匹配证据","15–20"],["Reproduction, RViz and match evidence","15–20"]),
           t(["资料核对与引用","21"],["Verification and references","21"])],[414,79])

    figure(shots/"12_NISTER5_Inliers.png",
           t("配对示例（见图 12）。绿色连线连接五点 RANSAC 保留的匹配。",
             "Matching example (Fig. 12). Green lines connect matches retained by five-point RANSAC."),188)

    page_start(t("坐标、尺度与参考值","Coordinates, scale and reference"),
               t("先确定变换方向，再比较误差","Fix the transform direction before comparing errors"))
    text(t("用 T_ab 表示把坐标从 b 系变换到 a 系。里程计给出 T_world_body，乘固定外参 T_body_camera 得到相机在世界系的位姿。相邻帧参考变换为 inv(T_world_prev) · T_world_curr。","Let T_ab map coordinates in frame b into frame a. Odometry supplies T_world_body. Composing it with the fixed T_body_camera gives the camera pose in the world. The relative reference is inv(T_world_prev) · T_world_curr."))
    text(t("如果旧相机是原点，新相机沿旧相机的 x 轴移动 0.10 m，那么该相对变换的平移为 (+0.10, 0, 0) m。新帧坐标系中的点经过变换后应落在旧帧坐标系。","If the previous camera is at the origin and the current camera moves 0.10 m along its x-axis, the relative translation is (+0.10, 0, 0) m. Applying the transform maps a current-camera point into previous-camera coordinates."))
    table([t(["量","计算","读数含义"],["Quantity","Computation","Interpretation"]),
           t(["旋转 RPE","两旋转矩阵的 Frobenius 距离","零表示两种旋转一致"],["Rotation RPE","Frobenius distance of two rotation matrices","Zero means identical rotations"]),
           t(["旋转角","从 R_ref^T R_est 提取角度","以度报告，可直接读作角度偏差"],["Rotation angle","Angle of R_ref^T R_est","Angular discrepancy in degrees"]),
           t(["二维平移方向","分别单位化后取欧氏距离","取值 0 到 2，方向相反时为 2"],["2D translation direction","Euclidean distance between unit directions","Ranges from 0 to 2, with 2 for opposite directions"]),
           t(["三维平移","直接比较两个平移向量","数值单位为米"],["3D translation","Compare the two translation vectors directly","Error is measured in meters"])],[103,195,195])
    text(t("报告中的定量误差先计算。RViz 再用上一帧真值锚定，并给二维平移方向补上真值长度，用于观察单步结果。","Errors are computed first. RViz then anchors each estimate to the previous reference pose and scales the 2D translation direction by the reference length for single-step visualization."))

    figure(shots/"04_RViz_ARUN3.png",
           t("米制位姿示例（见图 04）。Arun 利用深度估计平移，RViz 将单步结果锚定到上一帧真值。",
             "Metric pose example (Fig. 04). Arun estimates translation from depth. RViz anchors the result to the preceding reference pose."),198)

    page_start(t("四种方法怎么比较","How the four solvers compare"),
               t("相同的 SIFT 输入，不同的运动约束","Shared SIFT matches, different motion constraints"))
    table([t(["方法","需要什么","求解结果","主要限制"],["Method","Inputs","Output","Main limitation"]),
           t(["Nistér 五点","5 对标定视线","旋转、平移方向","多候选，需要用更多匹配消歧"],["Nistér 5pt","5 calibrated bearing pairs","Rotation and translation direction","Multiple candidates require disambiguation"]),
           t(["八点","8 对标定视线","旋转、平移方向","线性方程对点分布和噪声敏感"],["8pt","8 calibrated bearing pairs","Rotation and translation direction","Linear system is sensitive to point layout and noise"]),
           t(["已知旋转两点","2 对视线及已知旋转","平移方向","旋转输入误差会传到平移"],["Known-R 2pt","2 bearing pairs and known rotation","Translation direction","Rotation input error affects translation"]),
           t(["Arun 三点","3 对非共线三维点","旋转、米制平移","依赖深度有效性和对齐"],["Arun 3pt","3 non-collinear 3D pairs","Rotation and metric translation","Requires valid, registered depth"])],[102,120,115,156])
    text(t("公平比较时，三个二维方法使用同一批通过 SIFT 距离比筛选的匹配。Arun 还要求对应点两侧深度均有效。所有方法共享同一帧对和真值对齐过程。","The three 2D methods share the same matches surviving the SIFT ratio test. Arun also requires valid depth on both sides. All methods share the same frame pairs and reference alignment."))
    text(t("无 RANSAC 对照按描述子距离选一组点，只提出一次假设。它和五点 RANSAC 使用同一种几何模型，可用于观察重复采样和一致性评分的影响。","The baseline takes one group of points ranked by descriptor distance and proposes a single hypothesis. It shares the five-point geometry and illustrates the effect of repeated sampling and consensus scoring."))


    figure(shots/"13_EIGHT8_Inliers.png",
           t("八点法实测（见图 13）。从同一帧对的 SIFT 匹配中筛出符合模型的内点。",
             "Eight-point output (Fig. 13). The model retains inliers from SIFT matches for this pair."),188)

    page_start(t("D1 · Nistér 五点法","D1 · Nistér's five-point solver"),t("五对像点约束一个相对姿态","Five correspondences constrain a relative pose"))
    text(t("约定 R 和 t 把第二相机坐标变到第一相机。对去畸变的单位射线 x1、x2，有 x1^T E x2 = 0，其中 E = [t]_cross R。旋转有 3 个自由度，平移方向有 2 个。","Use R and t to map camera 2 into camera 1. For calibrated unit bearings x1 and x2, x1^T E x2 = 0 with E = [t]_cross R. Rotation has three degrees of freedom and translation direction has two."))
    table([t(["步骤","计算"],["Stage","Computation"]),
           t(["1. 找零空间","把五条对极约束堆叠成 5×9 矩阵。一般秩为 5，E 写成四个零空间基矩阵的线性组合。"],["1. Null space","Stack the epipolar constraints into a 5×9 matrix. With rank 5, express E using four null-space basis matrices."]),
           t(["2. 加本质矩阵约束","消去整体尺度后，代入 det(E)=0 与 2EE^TE-tr(EE^T)E=0。用消元得到一元十次多项式。"],["2. Essential constraints","Fix the global scale and impose det(E)=0 and 2EE^TE-tr(EE^T)E=0. Eliminate variables to obtain a degree-ten polynomial."]),
           t(["3. 选物理解","保留实根并回代。分解 E 取得候选 R、±t；用三角化正深度筛选，再用额外匹配给候选打分。"],["3. Physical pose","Back-substitute real roots. Decompose E into candidate rotations and ±t, check positive triangulated depth, then score on further correspondences."])],[139,354])
    text(t("退化：纯旋转时射线经 R 对齐，平移方向没有可观测约束；点近乎共线或视差极小时也难以稳定估计。平面场景可以得到有限解，但可能存在多个相容候选，不能把共面一概判为无解。[2]","Degeneracy: pure rotation aligns rays after R and leaves translation direction unobservable. Nearly collinear points or very small parallax make estimates unstable. A plane can yield finite solutions, sometimes with multiple compatible candidates. [2]"))
    text(t("设内点率 w，每次独立抽取五点。成功概率约为 w^5，首次抽中无外点样本的期望次数为 1/w^5。要达到置信度 p，需 N = ceil(log(1-p) / log(1-w^5))。w=0.7、p=0.99 时，期望约 5.95 次，预算 26 次。OpenGV 的 NISTER 包装额外取 3 对用于候选分解和消歧，实际抽样量为 8。","For independent five-point draws at inlier fraction w, the success probability is about w^5 and the expected wait is 1/w^5. At confidence p, N = ceil(log(1-p) / log(1-w^5)). For w=0.7 and p=0.99, the mean is 5.95 and the budget is 26. The OpenGV NISTER wrapper adds three pairs for decomposition and disambiguation, so it draws eight."))

    figure(shots/"16_NISTER5_RAW_Inliers.png",
           t("五点法的单次假设对照（见图 16）。绿色连线是这次假设解释的匹配。",
             "Single-hypothesis five-point baseline (Fig. 16). Green lines mark matches explained by that hypothesis."),188)

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

    figure(shots/"14_KNOWN2_Inliers.png",
           t("两点法实测（见图 14）。已知旋转后，用匹配点估计平移方向。",
             "Two-point output (Fig. 14). With rotation supplied, the matches constrain the translation direction."),188)

    page_start(t("抽样预算与外点","Sampling budget and outliers"),
               t("匹配越干净，越容易抽到可用的一组点","Cleaner matches make an all-inlier draw more likely"))
    text(t("如果内点率为 w，一次抽 m 对且近似独立，整组为内点的概率是 w^m。N 次都失败的概率是 (1−w^m)^N。令它不超过 0.01，便得到 99% 置信预算。","For inlier fraction w and sample size m, independent draws give an all-inlier probability of w^m. All N attempts fail with probability (1−w^m)^N. Requiring this to be at most 0.01 yields a 99% confidence budget."))
    header=t(["假设内点率","两点 m=2","五点 m=5","八点 m=8"],
             ["Inlier fraction","Two points m=2","Five points m=5","Eight points m=8"])
    rows=[header]
    for ratio in (.5,.7,.9):
        rows.append([f"{ratio:.0%}"] + [math.ceil(math.log(.01)/math.log1p(-ratio**n)) for n in (2,5,8)])
    table(rows,[121,124,124,124])
    text(t("表中是理论预算。OpenGV 的中心相机五点和八点封装还取额外对应来分解或选择候选。实验代码设定最多 300 次，并根据内点共识估计终止条件。","These are theoretical budgets. OpenGV's central five-point and eight-point wrappers draw additional matches for decomposition or candidate selection. The experiment caps iterations at 300 and uses inlier consensus to estimate when to stop."))
    text(t("同一个近似几何模型也可能解释一批错误匹配，因此应结合旋转和平移误差、内点数及场景视差一起判断。","Several mismatches can still fit an approximate geometric model. Assess inlier counts together with rotation and translation errors and scene parallax."))

    page_start(t("D3 · 环境与相机标定","D3 · Build and camera calibration"),
               t("构建产物与运行日志提供初始配置证据","Build outputs and run logs provide setup evidence"))
    figure(shots/"09_Catkin_Build.png",
           t("图 09  Ubuntu 桌面显示 Catkin 构建输出。","Fig. 09  Catkin build output displayed on the Ubuntu desktop."),217)
    figure(shots/"10_Office_Calibration.png",
           t("图 10  实际评估日志中的 CameraInfo、深度样本和帧数。","Fig. 10  Actual evaluation log showing CameraInfo, depth samples and frame counts."),217)

    page_start(t("D3 · 数据通道与对齐","D3 · Data topics and alignment"),
               t("检查位姿与深度能否对应到 RGB 帧","Check whether pose and depth can be aligned to RGB"))
    figure(shots/"11_Office_ROS_Topics.png",
           t("图 11  rosbag info 列出 Office 片段的输入话题与消息数。","Fig. 11  rosbag info shows input topics and counts for the Office segment."),225)
    table([t(["对齐步骤","检查点"],["Alignment step","Check"]),
           t(["相邻 RGB 帧","按时间遍历，跳过重复或倒退的时间戳"],["Adjacent RGB frames","Process in time order and skip duplicate or backwards timestamps"]),
           t(["关联深度","选择 15 ms 内最近的深度帧，像素在两帧中均需有有效深度"],["Associate depth","Choose nearest depth within 15 ms and require valid depth at both pixels"]),
           t(["关联真值","使用图像时间两侧的里程计，平移线性插值，旋转四元数插值"],["Associate reference","Use bracketing odometry and interpolate translation and quaternion rotation"])],[133,360])

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

    page_start(t("误差分布与达标比例","Error distribution and target rates"),
               t("用中位数和第 90 百分位同时看典型误差与较差帧对",
                 "Medians describe typical errors; the 90th percentile describes the upper tail"))
    def distribution(rows,key,threshold):
        data=sorted(vals(rows,key))
        if not data:
            return ["—","—","—"]
        index=math.ceil(.9*len(data))-1
        return [med(data),f"{data[index]:.4g}",f"{sum(v<threshold for v in data)}/{len(data)} ({sum(v<threshold for v in data)/len(data):.1%})"]
    rotation_rows=[t(["方法","中位数 °","P90 °","低于 1°"],["Method","Median °","P90 °","Below 1°"])]
    for m,name in zip(methods,names):
        rotation_rows.append([name,*distribution(series[m],"rotation_deg",1)])
    table(rotation_rows,[158,90,90,155])
    direction_rows=[t(["二维方法","中位数","P90","低于 0.5"],["2D method","Median","P90","Below 0.5"])]
    for i in (0,1,2,4):
        m=methods[i]
        direction_rows.append([names[i],*distribution(series[m],"translation_rpe",.5)])
    table(direction_rows,[158,90,90,155])
    arun=distribution(series["ARUN3"],"translation_rpe",.01)
    text(t(f"Arun 米制误差中位数为 {arun[0]} m，P90 为 {arun[1]} m，低于 1 cm 的帧对为 {arun[2]}。",
           f"Arun's median metric error is {arun[0]} m, with P90 of {arun[1]} m and {arun[2]} of pairs below 1 cm."))
    text(t("达标比例按有有效误差的帧对计算，求解成功数另见前两页。已知旋转两点法的旋转误差来自给定的旋转条件。这里的阈值用于解释统计，课程的验收按题面执行。",
           "Target rates use pairs with finite errors. Solve counts are shown on the preceding pages. The known-rotation solver receives rotation as input. The thresholds here describe the measurements; course acceptance follows the handout."))

    page_start(t("RANSAC 对照与结论","RANSAC comparison and findings"),
               t("同一种五点几何，比较单次假设与多次采样",
                 "Compare one hypothesis against repeated sampling with the same five-point geometry"))
    raw=summary[4]
    robust=summary[0]
    table([t(["五点方法","成功","旋转中位数 °","方向中位数","内点中位数"],
             ["Five-point method","Solved","Median rot. °","Median direction","Median inliers"]),
           [t("使用 RANSAC","With RANSAC"),robust[1],robust[3],robust[4],robust[5]],
           [t("单次假设","Single hypothesis"),raw[1],raw[3],raw[4],raw[5]]],[112,70,99,104,108])
    text(t("本片段中，多次抽样提高了内点数中位数，并降低了旋转和平移方向误差。八点法的两项中位误差均低于五点 RANSAC，这是这个片段上的观察，场景结构和帧间位移改变后还需要复测。",
           "On this segment, repeated sampling increased median inlier count and reduced both rotation and direction error. The eight-point medians are below the five-point RANSAC medians here. Changes in scene structure or frame-to-frame motion warrant another comparison."))
    text(t("两点法借助已知旋转，减少了要求解的未知量。Arun 在深度有效时能得到实际位移长度，可用厘米或毫米解释误差。二维的方向误差和三维的米制误差分开看。",
           "Known rotation reduces the unknowns in the two-point solver. Valid depth gives Arun an actual displacement length, so its errors can be read in centimeters or millimeters. Interpret 2D direction and 3D metric errors separately."))
    table([t(["异常现象","优先核对"],["Symptom","What to check first"]),
           t(["方向误差高","先查真值帧间基线，再查匹配是否集中在同一面墙上。"],
             ["Large direction error","Check the reference baseline, then whether matches cluster on a single wall."]),
           t(["Arun 可用点少","核对深度是否有效、与 RGB 是否配准。"],
             ["Few usable Arun points","Check depth validity and registration to RGB."]),
           t(["位姿箭头与曲线差异大","RViz 显示采用上一帧真值锚定，误差图比较相对变换。"],
             ["Pose arrows and curves look different","RViz uses the preceding reference pose as an anchor. The curves compare relative transforms."])],[158,335])

    page_start(t("数据来源与现场复现","Data provenance and live reproduction"),
               t("输入、操作、输出可以逐项核对","Trace inputs, commands and outputs"))
    scope=t(f"本次实际评估 {pairs} 对相邻帧，覆盖约 {end:.2f} 秒。报告主表和图 05–08 使用这次运行。图 01–04 是同一数据序列的短片段 RViz 回放。",
            f"The evaluation covers {pairs} adjacent pairs over about {end:.2f} s. The main tables and Figs. 05–08 use this run. Figs. 01–04 show RViz playback of a short segment from the same sequence.")
    text(scope)
    table([t(["步骤","操作","检查点"],["Step","Action","Verification"]),
           t(["1. 准备","使用已编译的 Ubuntu ROS Noetic 环境","可找到 lab_6 包与 Office 输入包"],
             ["1. Prepare","Use the built Ubuntu ROS Noetic environment","The lab_6 package and Office input bag are available"]),
           t(["2. 演示","进入 ~/lab6_complete，运行 bash demo.sh ARUN3","RViz 显示 RGB、深度和估计位姿"],
             ["2. Demo","In ~/lab6_complete, run bash demo.sh ARUN3","RViz displays RGB, depth and estimated poses"]),
           t(["3. 复算","运行 bash evaluate.sh","新目录生成逐帧 CSV 和位姿包"],
             ["3. Recompute","Run bash evaluate.sh","A new result directory contains per-pair CSV and pose bags"]),
           t(["4. 核对","打开摘要 CSV 和数据 SHA256","确认方法、分母与输入文件一致"],
             ["4. Verify","Open the summary CSV and input SHA256","Check method names, denominators and the input file"])],[103,182,208])
    if not args.full:
        text(t("完整归档约 8.57 GB。本次从已下载的约 5.00 GB 连续前段中提取完整消息。下载恢复后，应对完整录包重新转换和评估，再用全量结果重建图表。","The full archive is about 8.57 GB. This run extracts complete messages from about 5.00 GB of contiguous downloaded bytes. Once available, convert and evaluate the full recording, then regenerate the plots from that run."))

    for group in ((0,1),(2,3)):
        page_start(t("RViz 实际运行截图","RViz replay screenshots"),
                   t("左侧显示深度和 RGB，中央显示估计和真值相机位姿。","Depth and RGB are on the left; estimated and reference poses are in the center."))
        for i in group:
            figure(shots/f"{i+1:02d}_RViz_{methods[i]}.png",
                   t(f"图 {i+1:02d}  {names[i]} 的 Office 片段回放。",
                     f"Fig. {i+1:02d}  {names[i]} replay of the Office prefix."),254)

    image_pairs = (
        ((12,"NISTER5"),(13,"EIGHT8")),
        ((14,"KNOWN2"),(15,"ARUN3")),
        ((16,"NISTER5_RAW"),),
    )
    for group in image_pairs:
        page_start(t("实验匹配与内点证据","Match and inlier evidence"),
                   t("Ubuntu 实际输出，绿色连线是相应模型保留的匹配。","Actual Ubuntu output. Green lines connect correspondences retained by each model."))
        for index, method in group:
            figure(shots/f"{index:02d}_{method}_Inliers.png",
                   t(f"图 {index:02d}  {method} 的代表性画面对。",
                     f"Fig. {index:02d}  Representative pair for {method}."),227)
        if len(group)==1:
            text(t("无 RANSAC 对照只从描述子排序较前的匹配提出一次假设。图中展示一个成功帧对。时序曲线和汇总表展示该方法在全部已评估帧对上的误差。","The baseline proposes one hypothesis from the highest-ranked matches. The screenshot shows one successful pair. The error curves and summary cover all evaluated frame pairs."))
            table([t(["检查点","观察方法"],["Check","What to inspect"]),
                   t(["匹配","观察绿色线是否连接两张图中同一处边缘或纹理"],["Matches","Check whether green lines connect corresponding edges or texture"]),
                   t(["稳健性","结合五点 RANSAC 的方向误差与内点统计比较"],["Robustness","Compare with five-point RANSAC direction errors and inlier counts"])],[135,358])


    page_start(t("结果核对与引用","Verification and references"),
               t("复现脚本、测量口径与限制","Reproduction, metrics and limitations"))
    table([t(["结果","用途"],["Artifact","Purpose"]),
           t(["rpe.csv","逐帧对误差、匹配数、内点数、真值基线、求解耗时。"],["rpe.csv","Per-pair errors, match counts, inliers, baseline and solver timing."]),
           t(["summary.csv","有效对数与中位数。耗时只计位姿求解，包含 RANSAC。"],["summary.csv","Valid counts and medians. Timing covers pose solving and RANSAC."]),
           t(["演示操作文档","Ubuntu 上运行 RViz、切换方法、重新计算及查看结果。"],["Demo guide","Run RViz, switch methods, recompute and inspect output."])],[140,353])
    text(t("数据范围限制：2023 题面里的旧 Google Drive 文件返回 404。课程公告改用 ROS 2 Office 归档。Google Drive 返回配额限制，当前评估覆盖已下载部分中可完整读取的连续前段。RViz 回放截图来自同一序列的短片段，图 09–16 展示扩展实测的日志和匹配。Lab4 RGB-D 补充采集在旧仿真器的深度相机信息解析处失败，日志单独保存。","Scope: the original Google Drive file returned 404. The course now provides a ROS 2 archive. The download quota limited evaluation to complete records in the downloaded prefix. The RViz replay screenshots show a short segment of the same sequence. Figures 09–16 show logs and matches from the extended run. An optional Lab4 RGB-D recording failed during depth camera metadata parsing in the older simulator; its log is retained."))
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
