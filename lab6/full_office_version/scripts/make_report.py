#!/usr/bin/env python3
"""Recreate figures, statistics, and the typeset Lab6 report from raw CSV."""
import csv,json,sys,os,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Image,PageBreak,Table,TableStyle
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors

root=Path(sys.argv[1] if len(sys.argv)>1 else '/home/joe/lab6_results')
rows=list(csv.DictReader(open(root/'rpe.csv')))
names=['Nister 5pt','Longuet-Higgins 8pt','Known-R 2pt','Arun 3D-3D']
palette=['#1864ab','#e67700','#2b8a3e','#9c36b5']
data=[]; summary={}
for i in range(4):
    group=[r for r in rows if int(r['method'])==i];good=[r for r in group if r['status']=='ok'];data.append(good)
    stats={'pairs':len(group),'success':len(good),'failed':len(group)-len(good)}
    for field in ['translation_error','rotation_chordal','rotation_deg','solve_ms']:
        x=np.array([float(r[field]) for r in good]);x=x[np.isfinite(x)]
        stats[field]={'count':len(x),'mean':float(np.mean(x)),'median':float(np.median(x)),'p95':float(np.percentile(x,95)),'rmse':float(np.sqrt(np.mean(x*x)))} if len(x) else {}
    stats['rotation_below_1deg']=sum(float(r['rotation_deg'])<1 for r in good)/max(1,len(good))
    stats['translation_below_target']=sum(float(r['translation_error'])<(0.1 if i==2 else 0.5) for r in good)/max(1,len(good))
    summary[names[i]]=stats
(root/'summary.json').write_text(json.dumps(summary,indent=2))
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':130})
def plot(file,field,ids,ylabel,title):
    fig,ax=plt.subplots(figsize=(10,4.2))
    for i in ids:
        ax.plot([float(r['elapsed']) for r in data[i]],[float(r[field]) for r in data[i]],color=palette[i],lw=.7,alpha=.9,label=names[i])
    ax.set(xlabel='Time from first RGB frame (s)',ylabel=ylabel,title=title);ax.grid(alpha=.18);ax.legend(loc='upper right');fig.tight_layout();fig.savefig(root/file);plt.close(fig)
plot('rotation_error.png','rotation_chordal',range(3),'Rotation RPE (Frobenius norm)','Monocular relative rotation error | all evaluated pairs')
plot('translation_error.png','translation_error',range(3),'Unit translation direction error','Monocular translation error | scale-free, range [0, 2]')
plot('arun_rotation.png','rotation_chordal',[3],'Rotation RPE (Frobenius norm)','RGB-D relative rotation error | Arun + RANSAC')
plot('arun_translation.png','translation_error',[3],'Translation RPE (m)','RGB-D metric translation error | Arun + RANSAC')
plot('rotation_degrees.png','rotation_deg',range(4),'Geodesic angle error (degrees)','Rotation angle error | supplementary diagnostic')

pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
body=ParagraphStyle('body',fontName='STSong-Light',fontSize=10.5,leading=17,spaceAfter=9,textColor=colors.HexColor('#263445'))
title=ParagraphStyle('title',parent=body,fontSize=24,leading=31,spaceAfter=20,textColor=colors.HexColor('#12385b'))
heading=ParagraphStyle('heading',parent=body,fontSize=15,leading=22,spaceAfter=12,textColor=colors.HexColor('#12385b'))
small=ParagraphStyle('small',parent=body,fontSize=8,leading=12)
story=[]
def p(text,style=body):
    for a,b in [('·',' / '),('₁','1'),('₂','2'),('⁵','<super>5</super>'),('²','<super>2</super>'),('ᵢ','i'),('ᵀ','T'),('①','1.'),('②','2.'),('③','3.'),('④','4.'),('⑤','5.')]:text=text.replace(a,b)
    story.append(Paragraph(text,style))
def eq(text,name):
    fig=plt.figure(figsize=(7.5,.65));fig.text(.03,.42,'$'+text+'$',fontsize=16);fig.savefig(root/(name+'.png'),bbox_inches='tight',pad_inches=.12,dpi=180,transparent=True);plt.close(fig)
    im=Image(str(root/(name+'.png')));im.drawHeight=im.imageHeight*450/im.imageWidth;im.drawWidth=450;story.append(im);story.append(Spacer(1,8))
def page(h):story.append(PageBreak());p(h,heading)
def image(name):story.append(Image(str(root/name),width=480,height=202))
def table(content):
    cells=[[Paragraph(str(c),small) for c in row] for row in content]
    t=Table(cells,colWidths=[115]+[(480-115)/(len(content[0])-1)]*(len(content[0])-1),hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#edf3f8')),('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7),('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor('#bbcddd'))]));story.append(t);story.append(Spacer(1,12))
p('LAB 6 · 团队实验报告',title)
p('MIT VNAV 2023 / 基于 office 数据集的双视图运动估计',heading)
p('实验日期：2026-11-11；交付整理：2026-11-16。运行环境：Ubuntu 20.04、ROS Noetic、OpenCV 4.2、Eigen 3.3、OpenGV。')
p('本报告包含团队 Deliverable 3–5：相机标定与归一化、三种单目估计器和 RGB-D Arun 配准。所有误差与统计均由虚拟机实际执行后导出的 CSV 计算。')
table([['数据与覆盖范围','核验结果'],['原始输入','vnav-lab6-office（ROS2 SQLite3）'],['ROS1 转换','rosbags 0.9.23；仅转换本实验所需四个话题，原始数据保持不变'],['数据规模','236.995 s；3890 RGB；3890 depth；3890 camera_info；47400 odom'],['实际评测',' / '.join('%s: %d 对 / %d 成功'%(n,summary[n]['pairs'],summary[n]['success']) for n in names)],['实现位置','~/lab6_ws/src/lab6/src/pose_estimation.cpp'],['实验输出','~/lab6_results；~/lab6_ws/src/lab6/results']])
p('评测使用所有相邻同步图像，第一帧仅初始化。若存在求解失败，该图像对保留 failed 状态及 NaN 误差；不以真值填补失败估计。单目零基线时方向误差不可定义，单独记 NaN。',small)

page('Deliverable 3 · 数据、坐标系与实现设置')
p('相机标定读取 /tesse/left_cam/camera_info。转换后 K 的最后一个元素设为 1，避免初始模板中矩阵不完整。calibrateKeypoints 调用 cv::undistortPoints，再将 (x,y,1) 归一化为单位视线。')
eq(r'f_x=f_y=415.6922,\quad c_x=360,\ c_y=240,\quad K_{33}=1','eq_K')
p('相机外参沿用官方模板：camera x 对应 body -y，camera y 对应 body -z，camera z 对应 body x；相机在 body y 方向偏移 0.05 m。离线评测按图像头时间戳精确配对 RGB/depth，并对 200 Hz 里程计平移插值、四元数 SLERP，获得图像时刻的相机真值。')
eq(r'T^W_C=T^W_B T^B_C,\qquad T^{C_{t-1}}_{C_t}=(T^W_{C_{t-1}})^{-1}T^W_{C_t}','eq_pose')
p('SIFT 每帧最多 1600 个特征，FLANN k=2 匹配，Lowe ratio=0.7，并去除指向同一目标特征的重复匹配。四种算法共享同一组匹配。OpenGV RANSAC 最大 1000 次，置信度 0.999；单目阈值为 2(1-cos(arctan(0.25/f)))，约对应 0.25 像素角误差；Arun 欧氏距离阈值为 0.05 m。')
p('OpenCV 4.2 系统包未提供可用 SIFT，项目内附 OpenCV contrib 4.2 的 SIFT 源文件及原许可，使用系统 OpenCV 其余组件。构建统一启用 C++17 和 -march=native，解决 OpenGV 与调用方 Eigen 对齐不一致导致的崩溃。ROS2 CameraInfo 的 d/k/r/p 在转换后修复为 ROS1 的 D/K/R/P，并修正消息 MD5；图像与数值载荷保持不变。')
p('不使用真值挑选模型或修正平移方向。五点、八点仅在可视化时利用上一帧真值与当前真值位移尺度恢复世界坐标；两点法还按题意使用真值旋转。Arun 不缩放平移。')

page('Deliverable 4 · 2D–2D 误差曲线')
p('单目平移均归一化后比较；旋转使用要求的 Frobenius 弦距。曲线包含全部有效相邻帧，不裁剪尖峰，不平滑。')
eq(r'e_t=\|\hat t/\|\hat t\|-t/\|t\|\|_2,\quad e_R=\|\hat R-R\|_F','eq_errors')
image('rotation_error.png');image('translation_error.png')

page('Deliverable 5 · RGB-D Arun 配准')
p('利用像素深度将视线反投影到三维：X = z * f / f_z。过滤非有限、非正及超过 100 m 的深度，然后调用 OpenGV PointCloudSacProblem，以三点 Arun 为最小求解器运行 RANSAC，并使用全部内点重新拟合刚体变换。')
eq(r'\min_{R\in SO(3),t}\sum_{i\in I}\|X_{1i}-(RX_{2i}+t)\|_2^2','eq_arun')
p('两组三维点去质心，计算互协方差矩阵的 SVD，强制 det(R)=+1，最后由质心恢复 t。这里平移以米为单位，不与单目方向误差直接比较。')
image('arun_rotation.png');image('arun_translation.png')

page('实测统计与讨论')
content=[['方法','成功/总数','平移中位数','旋转中位数 (°)','旋转 P95 (°)']]
for n in names:
    s=summary[n];content.append([n,'%d/%d'%(s['success'],s['pairs']),'%.5f'%s['translation_error']['median'],'%.5f'%s['rotation_deg']['median'],'%.5f'%s['rotation_deg']['p95']])
table(content)
p('前三种方法的平移列为无量纲方向距离，Arun 的平移列为米。更完整的均值、RMSE、P95、求解时间和阈值占比见 summary.json，逐帧记录见 rpe.csv。')
for n in names:
    s=summary[n];p('%s：旋转误差低于 1° 的有效帧占 %.2f%%；平移误差中位数 %.6f，P95 为 %.6f。'%(n,100*s['rotation_below_1deg'],s['translation_error']['median'],s['translation_error']['p95']))
p('单目误差尖峰往往出现在短基线、低视差、平面结构占主导或特征误匹配时；这些是解释误差的可能机制，并未通过逐帧场景标注逐一证实。方向归一化使极小位移的噪声被放大。八点法需要更多独立匹配，在局部近似平面和低视差条件下可能更不稳定。')
p('两点法因旋转已知而约束更强，但仍可能受低视差影响；Arun 从深度获得公制尺度，因此可以直接评估米制平移误差。深度边界、像素取样和局部匹配噪声仍会导致误差。')
p('可视化发布的是“上一帧真值 × 本次相对估计”，并非独立累积的全局视觉里程计，不能据此宣称消除了长期漂移。')

page('复现实验与交付清单')
p('虚拟机中运行：<br/>cd ~/lab6_ws<br/>source /opt/ros/noetic/setup.bash<br/>catkin_make -j2<br/>source devel/setup.bash<br/>bash src/lab6/scripts/run_all.sh<br/>python3 src/lab6/scripts/make_report.py ~/lab6_results',small)
p('ROS 与 RViz 演示：<br/>source ~/lab6_ws/devel/setup.bash<br/>roslaunch lab_6 video_tracking.launch<br/>可设置 pose_estimator:=0/1/2/3，默认 -1 在同一节点比较全部算法。默认以 0.15 倍速播放以减少回调丢帧；完整定量结果来自离线逐帧运行。',small)
p('交付内容：完整 lab6 包、依赖安装与运行脚本、报告生成脚本、原始误差 CSV、统计 JSON、五张误差图、SIFT 匹配示例、编译与运行日志。本次不包含可选的 Lab4 赛车数据重采集，GitHub 上传记录随本次交付材料提供。')
p('参考资料',heading)
p('[1] MIT VNAV Lab6 (2023): https://vnav.mit.edu/labs_2023/lab6/exercises.html<br/>[2] D. Nistér, An Efficient Solution to the Five-Point Relative Pose Problem, CVPR 2003. https://www-users.cse.umn.edu/~hspark/CSci5980/nister.pdf<br/>[3] OpenGV 官方使用说明及源码: https://laurentkneip.github.io/opengv/page_how_to_use.html ; https://github.com/laurentkneip/opengv<br/>[4] OpenCV contrib 4.2 SIFT source: https://github.com/opencv/opencv_contrib/tree/4.2.0/modules/xfeatures2d<br/>[5] rosbags conversion: https://ternaris.gitlab.io/rosbags/',small)
def footer(c,doc):
    c.setFont('Helvetica',8);c.setFillColor(colors.HexColor('#728095'));c.drawString(44,26,'VNAV LAB 6 | office | 2026-11-11');c.drawRightString(550,26,str(doc.page))
SimpleDocTemplate(str(root/'LAB6_Team_Report.pdf'),pagesize=(595.28,841.89),rightMargin=50,leftMargin=45,topMargin=40,bottomMargin=45).build(story,onFirstPage=footer,onLaterPages=footer)
print(json.dumps(summary,indent=2));print(root/'LAB6_Team_Report.pdf')
