from pathlib import Path
import csv,json,html,copy
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,PageBreak,Image,Table,TableStyle,KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
root=Path(__file__).parent;sub=root.parent if root.name=='scripts' else root/'submission'
pdfmetrics.registerFont(TTFont('Lab5Song',r'C:\Windows\Fonts\simsun.ttc'))
pdfmetrics.registerFont(TTFont('Lab5Hei',r'C:\Windows\Fonts\simhei.ttf'))
blue=colors.HexColor('#193A50');line=colors.HexColor('#C7D5E0');light=colors.HexColor('#E7EFF5')
methods=['SIFT','SURF','ORB','FAST_BRIEF','LK']
sources=[('MIT VNAV LAB5 Exercises','https://vnav.mit.edu/labs_2023/lab5/exercises.html'),('MIT-SPARK VNAV-labs VNAV2021 release','https://github.com/MIT-SPARK/VNAV-labs/releases/tag/VNAV2021'),('MIT-SPARK VNAV lab data','https://github.com/MIT-SPARK/VNAV-lab-data'),('OpenCV 4.2 Feature Detection and Description','https://docs.opencv.org/4.2.0/d5/d51/group__features2d__main.html'),('OpenCV 4.2 Object Tracking','https://docs.opencv.org/4.2.0/dc/d6b/group__video__track.html'),('OpenCV 4.2 Camera Calibration and 3D Reconstruction','https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html'),('LAB5 measured CSV records and numbered evidence','https://github.com/hbyhby123/vnav-personal/tree/master/lab5/results')]
def get_core(filename):
 ns={'__file__':str(root/filename)};s=(root/filename).read_text(encoding='utf-8-sig');exec(s[:s.rfind('SimpleDocTemplate(')],ns)
 groups=[]
 for x in ns['story']:
  if isinstance(x,Paragraph):
   if x.style.name=='heading':groups.append([x.text,[]])
   elif x.style.name in ['normal','small']:groups[-1][1].append(x.text.replace('README_提交说明.md','the repository README') if filename.endswith('_en.py') else x.text)
 return groups
cores={False:get_core('report_content_cn.py'),True:get_core('report_content_en.py')}
for en in [False,True]:
 T=lambda cn,eng:eng if en else cn
 bodyfont='Times-Roman' if en else 'Lab5Song';headfont='Helvetica-Bold' if en else 'Lab5Hei'
 normal=ParagraphStyle('Body',fontName=bodyfont,fontSize=10.3,leading=15.1,spaceAfter=7)
 small=ParagraphStyle('Small',parent=normal,fontSize=8.1,leading=11,spaceAfter=6)
 head=ParagraphStyle('Heading',parent=normal,fontName=headfont,fontSize=14.5,leading=20,textColor=blue,spaceAfter=10)
 title=ParagraphStyle('Title',parent=head,fontSize=21,leading=27,alignment=TA_CENTER,spaceAfter=13)
 cap=ParagraphStyle('Caption',parent=small,alignment=TA_CENTER,spaceAfter=7)
 metadata=ParagraphStyle('Metadata',parent=small,alignment=TA_CENTER)
 story=[];htmlout=['<!doctype html><meta charset="utf-8"><style>body{font:16px Georgia,"Microsoft YaHei";max-width:1000px;margin:40px auto;line-height:1.7;color:#263746}h1,h2{color:#193a50}img{max-width:100%}table{border-collapse:collapse;width:100%}td,th{border:1px solid #c7d5e0;padding:8px}th{background:#e7eff5}</style>'];tabno=0;core=cores[en]
 def p(s,style=normal):story.append(Paragraph(s,style));htmlout.append('<p>'+s+'</p>')
 def h(c,e):p(T(c,e),head)
 def page(c,e):story.append(PageBreak());h(c,e)
 def paras(idx,cite=None,limit=None):
  texts=core[idx][1] if limit is None else core[idx][1][:limit]
  for i,s in enumerate(texts):p(s+(f' [{cite}]' if cite and i==len(texts)-1 else ''))
 def fig(folder,name,num,cn,eng,maxh=200):
  path=sub/'results'/folder/name;im=Image(str(path));factor=min(475/im.imageWidth,maxh/im.imageHeight);im.drawWidth=im.imageWidth*factor;im.drawHeight=im.imageHeight*factor
  label=T(f'图{num} {cn}',f'Figure {num} {eng}');story.extend([im,Paragraph(label,cap)]);htmlout.append(f'<figure><img src="../results/{folder}/{name}"><figcaption>{label}</figcaption></figure>')
 def screenshot(n,name,cn,eng,maxh=190):fig('screenshots',name,n,cn,eng,maxh)
 def chart(n,name,cn,eng,maxh=235):fig('charts',name,n,cn,eng,maxh)
 def tab(cn,eng,headers,rows,widths=None):
  nonlocal_dummy=None
  global_dummy=None
  # Paragraph cells wrap long file names rather than extending beyond the page.
  captext=T('表','Table ')+str(len([x for x in htmlout if x.startswith('<table')])+1)+' '+T(cn,eng)
  p(captext,cap);data=[[Paragraph(str(v),small) for v in headers]]+[[Paragraph(str(v),small) for v in row] for row in rows]
  widths=widths or [475/len(headers)]*len(headers)
  table=Table(data,colWidths=widths,repeatRows=1,hAlign='CENTER');commands=[('GRID',(0,0),(-1,-1),.5,line),('BACKGROUND',(0,0),(-1,0),light),('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),4)]
  for i in range(2,len(data),2):commands.append(('BACKGROUND',(0,i),(-1,i),colors.HexColor('#F5F7F9')))
  table.setStyle(TableStyle(commands));story.extend([table,Spacer(1,8)]);htmlout.append('<table>'+''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row)+'</tr>' for row in [headers]+rows)+'</table>')
 def stats(dataset):
  rows=[]
  for m in methods:
   f=sub/'results'/dataset/(m+'_summary.csv')
   if f.exists():
    r=next(csv.DictReader(f.open()));rows.append([m,r['pairs']]+[f'{float(r[k]):.1f}' for k in ['keypoints1','keypoints2','matches','good','inliers']]+[f'{float(r["inlier_ratio"])*100:.1f}%',f'{float(r["ms"]):.2f}'])
  tab(T('实测统计','Measured statistics')+' '+dataset,'Measured statistics '+dataset,T(['方法','帧对','特征1','特征2','原始','筛后','内点','内点率','耗时ms'],['Method','Pairs','KP1','KP2','Raw','Good','Inliers','Ratio','ms']),rows,[62,31,48,48,48,48,48,48,48])
 p(T('VNAV Lab5 无人机视觉跟踪','VNAV Lab5 UAV Visual Tracking'),title)
 p(T('黄博宇 24020036026；中外计科一班；第8组','Boyu Huang, 24020036026; International Computer Science, Class 1; Group 8'),metadata)
 p(T('实验记录：2026年10月30日至31日；报告整理：2026年10月31日','Experiment: 30-31 October 2026; report prepared: 31 October 2026'),metadata)
 h('摘要','Summary')
 p(T('本实验在 VMware Ubuntu 20.04 与 ROS Noetic 中完成视觉特征匹配和跟踪。补全 SIFT、SURF、ORB、FAST+BRIEF 与 Harris+LK 的 C++ 实现，并对两段官方序列的 1,583 和 1,385 个相邻帧对进行完整评估。第一段和第二段中，ORB 的平均几何内点数分别为 528.2 和 316.5；FAST+BRIEF 的平均计算耗时分别为 1.92 和 1.65 ms。报告保留17张实际截图，增加7张原始数据图表，并给出投影、消失点推导及可复现的统计口径。几何内点不等同于人工真值。[1-3,7]','The experiment implements visual feature matching and tracking in VMware Ubuntu 20.04 with ROS Noetic. SIFT, SURF, ORB, FAST+BRIEF and Harris+LK are evaluated on every neighboring frame pair in two official bags: 1,583 and 1,385 pairs. ORB yields 528.2 and 316.5 mean geometric inliers; FAST+BRIEF takes 1.92 and 1.65 ms on average. The report retains 17 actual screenshots and adds seven reproducible charts, alongside projection and vanishing-point derivations. Model-based geometric inliers are not human ground truth. [1-3,7]'))
 h('1 引言','1 Introduction')
 p(T('Lab3 关注怎样跟上目标，Lab4 关注怎样规划路线；Lab5 进一步关注如何从图像找到可反复识别的点，并把相邻帧的点对应起来。特征检测相当于挑选路标，描述子给路标建立“特征卡”，匹配和几何验证则检查是不是同一处。本文按实验设置、实现、结果、讨论与结论组织，与此前 Lab3/Lab4 报告保持一致。','Lab3 follows a reference and Lab4 plans a route. Lab5 asks how to find repeatable image landmarks and associate them between frames. Detection selects landmarks, descriptors encode local appearance, and matching plus geometry checks their correspondence. The report follows the same settings, implementation, results, discussion and conclusions structure as the earlier Lab3/Lab4 reports.'))
 tab('实验任务与交付证据','Tasks and reports',T(['交付项','内容','证据入口'],['Item','Task','Evidence']),[[f'D{i}',T(c,e),a] for i,c,e,a in [(1,'球体投影','Sphere projection','Section 4'),(2,'消失点','Vanishing points','Section 4'),(3,'SIFT检测描述','SIFT detection','Figures 03-04'),(4,'匹配与比值筛选','Matching and ratio test','Figures 04-05'),(5,'几何内点和统计','Inliers and statistics','Figure 06, tables'),(6,'算法与真实序列比较','Method and sequence comparison','Figures 07-11,19-24'),(7,'Harris+LK','Harris+LK','Figures 12-13,17'),(8,'Farneback选做','Farneback extension','Figures 14-15')]], [42,185,248])
 page('2 实验设置','2 Experimental Settings')
 tab('实验环境和统一参数','Environment and shared parameters',T(['项目','配置','用途'],['Item','Setting','Purpose']),[[T('系统','System'),'Ubuntu 20.04 / ROS Noetic',T('运行ROS程序','Run ROS nodes')],['OpenCV','4.2.0 contrib; NONFREE=ON',T('支持SIFT和SURF','Enable SIFT and SURF')],[T('工作空间','Workspace'),'~/lab5_complete/ws',T('隔离已有实验','Isolate previous labs')],[T('比值筛选','Ratio test'),'d1 &lt; 0.8*d2',T('删除模糊匹配','Reject ambiguous matches')],['RANSAC','3 px; confidence 0.99',T('检查极线一致性','Check epipolar consistency')],[T('评估','Evaluation'),'1 OpenCV thread; seed 42',T('完整遍历与可复算统计','Full traversal and reproducible records')]], [76,205,194])
 screenshot(1,'01_Ubuntu_ROS_Environment.png','环境与独立OpenCV构建','Environment and isolated OpenCV build',190)
 screenshot(2,'02_Catkin_Build_and_ROS_Test.png','catkin编译与启动验证','catkin build and launch check',190)
 p(T('图01和图02对应实际 Ubuntu 环境。OpenCV 独立编译，避免直接替换系统依赖；编译参数和动态链接记录保存在 logs。课程代码来源与构建依据见[2,4]。','Figures 01 and 02 show the actual Ubuntu environment. OpenCV is built separately; build options and dynamic linking are retained in logs. Course code and API sources are [2,4].'),small)
 page('3 实现流程与参数','3 Implementation Pipeline and Parameters')
 chart(18,'18_Feature_Tracking_Pipeline.png','特征匹配与跟踪流程','Feature matching and tracking pipeline',155)
 tab('各方法的组成','Method components',T(['方法','检测和描述','匹配或跟踪'],['Method','Detection and description','Matching or tracking']),[['SIFT',T('尺度空间；128维浮点','Scale space; 128 floats'),'FLANN, ratio 0.8'],['SURF',T('Hessian阈值400；浮点描述子','Hessian 400; float descriptor'),'FLANN, ratio 0.8'],['ORB',T('最多1000点；32字节二值','Up to 1000 points; 32 bytes'),'Hamming BF, ratio 0.8'],['FAST+BRIEF',T('FAST阈值20；BRIEF 32字节','FAST 20; BRIEF 32 bytes'),'Hamming BF, ratio 0.8'],['Harris+LK',T('最多500角点；质量0.01；间隔8px','500 corners; quality .01; spacing 8 px'),T('21x21窗口；最大层级3','21x21 window; max level 3')]], [75,235,165])
 paras(3,'2,4,6',limit=3)
 p(T('图18中描述子路径先匹配再验证；LK路径在局部窗口估计运动，再使用相同几何检查。实现对空描述子、二近邻不足、少于8对点和零分母都作了保护。','The descriptor path matches before geometric validation; LK estimates local motion before the same geometric check. Empty descriptors, incomplete neighbor lists, fewer than eight pairs and zero denominators are guarded.'))
 page('4 个人理论题','4 Individual Theory Exercises')
 h('4.1 球体的透视投影','4.1 Perspective projection of a sphere');paras(1,'1')
 h('4.2 平行直线的消失点','4.2 Vanishing points of parallel lines');paras(2,'1')
 page('5 双图实验 特征提取与原始匹配','5 Image Pair Keypoints and Raw Matching')
 paras(5,'4')
 screenshot(3,'03_SIFT_Keypoints.png','两幅图像的SIFT关键点','SIFT keypoints in both images',185)
 screenshot(4,'04_SIFT_Raw_Matches.png','未经筛选的最近邻匹配','Unfiltered nearest-neighbor matches',185)
 page('5.1 距离比筛选与几何内点','5.1 Ratio Filtering and Geometric Inliers')
 screenshot(5,'05_SIFT_Ratio_Filter.png','距离比筛选后的匹配','Matches after the ratio test',170)
 screenshot(6,'06_SIFT_RANSAC.png','绿色内点与红色外点','Green inliers and red outliers',170)
 stats('pair');paras(6,'6,7')
 page('5.2 SURF ORB 与 FAST BRIEF','5.2 SURF ORB and FAST BRIEF')
 for n,f,c,e in [(7,'07_SURF_RANSAC.png','SURF几何验证','SURF geometric verification'),(8,'08_ORB_RANSAC.png','ORB几何验证','ORB geometric verification'),(9,'09_FAST_BRIEF_RANSAC.png','FAST与BRIEF几何验证','FAST and BRIEF geometric verification')]:screenshot(n,f,c,e,157)
 paras(8,'4,7',limit=2)
 page('6 真实序列与评估口径','6 Real Sequences and Evaluation Protocol')
 tab('数据集与实际样本数','Datasets and actual sample counts',T(['序列','RGB话题','图像数','帧对数'],['Dataset','Selected RGB topic','Images','Pairs']),[['Dataset 1','/camera/color/image_raw','1584','1583'],['Dataset 2','/tesse/left_cam/rgb/image_raw','1386','1385']], [68,295,56,56])
 p(T('完整遍历同一RGB话题的所有相邻图像，不跳帧、不混入深度图和其他相机。数据文件与桌面原始文件的SHA256一致，校验值记录在datasets.json。第二段使用仿真相对时间，不应把包内历史日期误认为实验日期。[3,7]','Every neighboring pair on one RGB topic is traversed, without subsampling or mixing depth images and cameras. SHA256 checks agree with the original desktop copies and are recorded in datasets.json. Dataset 2 uses relative simulation time; its displayed historical date is not the experiment date. [3,7]'))
 chart(19,'19_Mean_Geometric_Inliers.png','两段序列的平均几何内点数','Mean geometric inliers across both sequences',265)
 tab('关键统计量的含义','Definitions of reported statistics',T(['指标','定义','比较时注意'],['Metric','Definition','Interpretation']),[[T('原始匹配','Raw'),T('每个查询点的最佳候选','Best candidate per query'),T('不是两近邻元素总数','Not total two-neighbor elements')],[T('筛后匹配','Good'),T('距离比通过的候选；LK为有效跟踪','Ratio-passing matches; valid LK tracks'),T('LK不使用距离比','No ratio test for LK')],[T('内点率','Inlier ratio'),T('先算每对内点/筛后，再平均','Mean of per-pair inliers/good'),T('不等于累计数之比','Not ratio of accumulated counts')]], [75,225,175])
 page('6.1 第一段序列结果','6.1 Results on Dataset 1')
 stats('dataset1');paras(9,'3,7',limit=3)
 screenshot(10,'10_Dataset1_Statistics.png','第一段完整统计输出','Complete statistics for dataset 1',235)
 p(T('数据来源：results/dataset1/*_frames.csv 与 *_summary.csv。图10为这些结果的终端展示，表格直接读取同一组文件。','Source: results/dataset1/*_frames.csv and *_summary.csv. Figure 10 displays those records; the table is generated from the same files.'),small)
 page('6.2 第二段序列结果','6.2 Results on Dataset 2')
 stats('dataset2');paras(10,'3,7',limit=3)
 screenshot(11,'11_Dataset2_Statistics.png','第二段完整统计输出','Complete statistics for dataset 2',235)
 p(T('数据来源：results/dataset2。所有方法均评估1,385个帧对，与第一段采用相同参数和统计公式。','Source: results/dataset2. All methods evaluate 1,385 pairs using the same settings and definitions as dataset 1.'),small)
 page('6.3 质量与计算代价','6.3 Quality and Computation Cost')
 chart(20,'20_Mean_Inlier_Ratio.png','平均内点率比较','Mean per-pair inlier ratio',225)
 chart(21,'21_Computation_Time_Log.png','计算耗时比较 对数纵轴','Computation time on a logarithmic axis',225)
 p(T('图20显示内点比例，图19显示绝对内点数量，两者不能相互替代。图21仅统计检测、描述、匹配、几何验证，排除文件读取、绘图和截图。纵轴采用对数刻度以同时展示约1至135 ms的范围；不应按柱高的线性差值估计倍数。虚拟机后台任务与调度会影响时间，这些结果不是硬实时保证。[7]','Figure 20 measures proportion, while Figure 19 measures absolute count; neither substitutes for the other. Figure 21 includes detection, description, matching and geometry, excluding file loading, drawing and capture. Its logarithmic axis spans approximately 1-135 ms; linear height differences are not time differences. VM background work and scheduling affect timing, so these are not hard real-time guarantees. [7]'))
 page('6.4 筛选阶段的数量变化','6.4 Candidate Retention by Stage')
 chart(22,'22_Ratio_Test_Retention.png','距离比筛选的平均保留率','Mean retention after the ratio test',225)
 chart(23,'23_Box_Pair_Matching_Stages.png','盒子图对的原始 筛后与内点数量','Raw, filtered and inlier counts on the box pair',225)
 p(T('图22直接计算每帧对good/raw后再取平均，空原始集合记为0。LK没有描述子距离比，因此不放入该图。图23使用双图原始CSV，纵轴为对数刻度；筛掉的点多不一定意味着更好，必须结合留下的内点数量和场景变化解释。[7]','Figure 22 averages good/raw over pairs, assigning zero to an empty raw set. LK is excluded because it has no descriptor ratio test. Figure 23 uses the box-pair CSV and a logarithmic count axis. Removing more candidates is not automatically better; the remaining geometric inliers and scene changes must also be considered. [7]'))
 page('6.5 时序稳定性与耗时分布','6.5 Temporal Stability and Timing Distribution')
 chart(24,'24_Temporal_Inlier_Ratio.png','内点率随帧变化 51帧对后向均值','Inlier-ratio traces with a 51-pair trailing mean',305)
 p(T('图24保留完整序列顺序，用51帧对的后向平均减少单帧抖动；这只用于可视化，前述表格仍由全部原始帧对直接计算。曲线下降可能来自弱纹理、遮挡或运动变化，不能在缺少真值时只凭曲线确定原因。[7]','Figure 24 retains sequence order and uses a 51-pair trailing mean only for visualization. Tables still use every original pair. Drops may be associated with weak texture, occlusion or changing motion, but the plot alone does not establish their cause without ground truth. [7]'))
 timing=[]
 for m in methods:
  values=[]
  for d in ['dataset1','dataset2']:
   a=sorted(float(r['ms']) for r in csv.DictReader((sub/'results'/d/(m+'_frames.csv')).open()));n=len(a);values.extend([f'{a[n//2]:.2f}',f'{a[min(n-1,int(.95*(n-1)))]:.2f}'])
  timing.append([m]+values)
 tab('逐帧耗时的中位数与95分位数 单位ms','Per-pair median and 95th-percentile time in ms',['Method','D1 median','D1 P95','D2 median','D2 P95'],timing,[95]*5)
 page('7 Harris 与 Lucas Kanade 跟踪','7 Harris and Lucas Kanade Tracking')
 paras(11,'5',limit=2)
 screenshot(12,'12_Dataset1_LK.png','第一段Harris与LK轨迹','Harris+LK tracks in dataset 1',180)
 screenshot(13,'13_Dataset2_LK.png','第二段Harris与LK轨迹','Harris+LK tracks in dataset 2',180)
 for s in core[11][1][2:]:p(s+' [5]')
 page('8 Farneback 稠密光流选做','8 Farneback Dense Optical Flow Extension')
 paras(13,'5',limit=1)
 screenshot(14,'14_Dataset1_Farneback.png','第一段稠密光流','Dense flow in dataset 1',200)
 screenshot(15,'15_Dataset2_Farneback.png','第二段稠密光流','Dense flow in dataset 2',200)
 for s in core[13][1][1:]:p(s+' [5]')
 page('9 ROS 交互运行证据','9 ROS Interactive Verification')
 screenshot(16,'16_ROS_Live_SIFT_Window.png','SIFT实时匹配窗口','Live SIFT matching window',215);paras(4,'7')
 screenshot(17,'17_ROS_Live_LK_Video.png','rosbag播放与LK跟踪窗口','rosbag playback and LK tracking',215);paras(12,'7')
 page('10 讨论','10 Discussion')
 tab('各方法的实际取舍','Observed tradeoffs',T(['方法','本实验观察','使用时注意'],['Method','Observed behavior','Caution']),[['SIFT',T('对盒子视角变化保留较稳定的内点','Stable box-pair geometric correspondences'),T('本次虚拟机计算代价较高','Higher measured computation cost')],['SURF',T('第二段特征数比SIFT多','More features than SIFT in dataset 2'),T('点多不保证内点率更高','More points need not improve ratio')],['ORB',T('两段平均内点数均最高','Highest mean inlier count in both sequences'),T('受尺度 纹理和检测预算影响','Depends on scale, texture and point budget')],['FAST+BRIEF',T('本次平均计算耗时最低','Lowest mean computation time'),T('盒子图对匹配较弱','Weaker matches on the box pair')],['Harris+LK',T('连续小位移下保持较高内点率','High model consistency for local motion'),T('特征2和筛后定义不同','KP2 and Good have different meanings')]], [75,205,195])
 p(T('本实验的内点依据估计的基本矩阵，而不是人工标注。盒子场景近似平面、相邻帧基线较小，可能使几何估计退化。不能把较高内点率直接写成“真实匹配准确率”。[6,7]','Inliers come from an estimated fundamental matrix, not manual labels. The box scene is approximately planar, and neighboring frames have a small baseline; both may affect geometric estimation. A high inlier ratio must not be presented as ground-truth matching accuracy. [6,7]'))
 p(T('在线ROS跟踪与离线统计承担不同任务：前者检查消息和显示链路，后者避免队列丢帧并形成完整表格。LK在线类持续保存角点，离线表每对重新检测，因此两者不能混成同一条实验记录。[7]','Live ROS tracking verifies the message and display path; offline traversal avoids queue-related omissions and supplies complete tables. Online LK retains points, whereas the offline comparison detects anew per pair. Their records must not be mixed. [7]'))
 p(T('图18至图24由保存的CSV通过make_report_charts.py生成，未手工填写结果。原始CSV、results/charts/chart_data.json、编号截图和数据集SHA256均保留，便于复算与检查。[7]','Figures 18-24 are generated from saved CSV files by make_report_charts.py, without manually entered measurements. Raw CSV, results/charts/chart_data.json, numbered captures and dataset hashes are retained for independent checks. [7]'))
 page('11 结论与复现','11 Conclusions and Reproduction')
 paras(14,'2,4,5,7')
 tab('复现材料索引','Reproduction file index',T(['材料','路径','用途'],['Material','Path','Purpose']),[[T('源码','Source'),'feature_tracking/',T('ROS节点与离线评估器','ROS nodes and evaluator')],[T('统计','Statistics'),'results/pair, dataset1, dataset2',T('原始逐帧表和均值','Per-pair and summary CSV')],[T('截图','Captures'),'results/screenshots/01...17',T('真实运行证据','Actual execution evidence')],[T('图表','Charts'),'results/charts/18...24',T('可复算PNG与矢量PDF','Reproducible PNG and vector PDF')],[T('脚本','Scripts'),'scripts/',T('依赖 运行 图表 报告','Build, run, plot and report')]], [65,230,180])
 page('参考文献与数据出处','References and Data Provenance')
 p(T('正文中的方括号编号对应下列来源。算法和API说明引用官方文档；具体性能数字引用本实验原始数据，不归因于文档中的演示结果。','Bracketed numbers in the text correspond to the sources below. Algorithm and API descriptions cite official documentation; measured performance values cite the experiment records, not illustrative documentation results.'))
 for i,(name,url) in enumerate(sources,1):p(f'[{i}] {name}.<br/><link href="{url}" color="#245b83">{url}</link>',small)
 p(T('数据记录：2026-10-30至31。上游代码版本：2443ec7e491001d09ab8dd66f6b953177043b784。图01至17为本次Ubuntu实际截图；图18至24为原始CSV生成的统计图与流程图。排版参考同仓库Lab3/Lab4报告的章节、页眉、蓝灰色表格和图注形式，未复用其测量数据。','Data recorded: 30-31 October 2026. Upstream code revision: 2443ec7e491001d09ab8dd66f6b953177043b784. Figures 01-17 are actual Ubuntu captures; Figures 18-24 are generated charts and the pipeline diagram. Layout follows the chapter structure, header, blue-gray tables and captions of the repository Lab3/Lab4 reports; their measurements are not reused.'),small)
 hashes=json.loads((sub/'scripts'/'datasets.json').read_text())
 for r in hashes:p(r['filename']+'<br/>SHA256: '+r['sha256'],small)
 def footer(canvas,doc):
  canvas.setStrokeColor(line);canvas.line(48,A4[1]-32,A4[0]-48,A4[1]-32);canvas.setFillColor(colors.HexColor('#718494'));canvas.setFont(bodyfont,7.5);canvas.drawString(48,A4[1]-24,T('VNAV LAB5 无人机视觉跟踪','VNAV LAB5 UAV Visual Tracking')+' | '+T('黄博宇','Boyu Huang')+' 24020036026');canvas.drawCentredString(A4[0]/2,24,str(doc.page))
 report_dir=sub/'reports';report_dir.mkdir(exist_ok=True)
 filename='LAB5_Visual_Tracking_Report_EN' if en else 'LAB5_无人机视觉跟踪实验报告'
 SimpleDocTemplate(str(report_dir/(filename+'.pdf')),pagesize=A4,leftMargin=48,rightMargin=48,topMargin=47,bottomMargin=42,title=T('VNAV Lab5 无人机视觉跟踪','VNAV Lab5 UAV Visual Tracking'),author='Boyu Huang').build(story,onFirstPage=footer,onLaterPages=footer)
 print(filename)
