from pathlib import Path
import sys,csv,json
root=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(root/'.report_packages'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
sub=root;out=sub/'charts';out.mkdir(exist_ok=True)
methods=['SIFT','SURF','ORB','FAST_BRIEF','LK'];labels=['SIFT','SURF','ORB','FAST+BRIEF','Harris+LK'];datasets=['dataset1','dataset2']
colors=['#245b83','#b56a37'];palette=['#245b83','#348d81','#b56a37','#8b6c9e','#687d36']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.titleweight':'bold','axes.labelcolor':'#263746','text.color':'#263746','grid.alpha':.2,'savefig.facecolor':'white'})
summaries={d:{m:next(csv.DictReader((sub/'results'/d/(m+'_summary.csv')).open())) for m in methods} for d in datasets}
raw={d:{m:list(csv.DictReader((sub/'results'/d/(m+'_frames.csv')).open())) for m in methods} for d in datasets}
meta={}
def save(name,fig):fig.savefig(out/(name+'.png'),dpi=180,bbox_inches='tight');fig.savefig(out/(name+'.pdf'),bbox_inches='tight');plt.close(fig)
def compare(name,key,title,ylabel,percent=False,log=False):
 fig,ax=plt.subplots(figsize=(9,4.2));x=np.arange(5);width=.36
 vals=[]
 for j,d in enumerate(datasets):
  v=[float(summaries[d][m][key])*(100 if percent else 1) for m in methods];vals.append(v)
  bars=ax.bar(x+(j-.5)*width,v,width,label=f'Dataset {j+1}',color=colors[j],alpha=.92)
  ax.bar_label(bars,labels=[f'{z:.1f}' for z in v],padding=3,fontsize=8)
 ax.set_xticks(x,labels);ax.set_ylabel(ylabel);ax.set_title(title);ax.legend(loc='upper right',frameon=False)
 if percent:
  ax.legend(loc='lower center',bbox_to_anchor=(.5,1.02),ncol=2,frameon=False);ax.set_title(title,pad=36);ax.grid(axis='y');ax.set_axisbelow(True)
 if log:ax.set_yscale('log');ax.set_ylim(.7,400)
 elif percent:ax.set_ylim(0,114)
 else:ax.set_ylim(0,max(max(v) for v in vals)*1.2)
 fig.tight_layout();save(name,fig);meta[name]=dict(key=key,dataset1=vals[0],dataset2=vals[1])
compare('19_Mean_Geometric_Inliers','inliers','Geometric inliers per neighboring frame pair','Mean number of inliers')
compare('20_Mean_Inlier_Ratio','inlier_ratio','Mean per-pair inlier ratio','Inlier ratio (%)',True)
compare('21_Computation_Time_Log','ms','Feature computation time in the virtual machine','Mean computation time (ms, logarithmic scale)',log=True)
fig,ax=plt.subplots(figsize=(9,4.2));x=np.arange(4);width=.36;vals=[]
for j,d in enumerate(datasets):
 v=[]
 for m in methods[:4]:
  rates=[float(r['good'])/float(r['matches']) if float(r['matches']) else 0 for r in raw[d][m]];v.append(float(np.mean(rates)*100))
 vals.append(v);bars=ax.bar(x+(j-.5)*width,v,width,color=colors[j],label=f'Dataset {j+1}');ax.bar_label(bars,fmt='%.1f',padding=3,fontsize=8)
ax.set_xticks(x,labels[:4]);ax.set_ylim(0,110);ax.set_ylabel('Mean good/raw per pair (%)');ax.set_title('Retention after the descriptor distance-ratio test');ax.legend(frameon=False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True);fig.tight_layout();save('22_Ratio_Test_Retention',fig);meta['22_Ratio_Test_Retention']=vals
pair={m:next(csv.DictReader((sub/'results/pair'/(m+'_summary.csv')).open())) for m in methods[:4]}
fig,ax=plt.subplots(figsize=(9,4.2));x=np.arange(4)
for j,(key,name,col) in enumerate([('matches','Raw','#91afc6'),('good','After ratio test','#44818e'),('inliers','RANSAC inliers','#b56a37')]):
 v=[float(pair[m][key]) for m in methods[:4]];bars=ax.bar(x+(j-1)*.25,v,.25,label=name,color=col);ax.bar_label(bars,fmt='%.0f',padding=2,fontsize=8)
ax.set_xticks(x,labels[:4]);ax.set_yscale('log');ax.set_ylim(5,4000);ax.set_ylabel('Matches (logarithmic scale)');ax.set_title('Box pair: candidate reduction by stage');ax.legend(frameon=False,ncol=3,loc='upper left');ax.grid(axis='y');ax.set_axisbelow(True);fig.tight_layout();save('23_Box_Pair_Matching_Stages',fig)
fig,axes=plt.subplots(2,1,figsize=(9,6),sharey=True)
for j,d in enumerate(datasets):
 for m,label,col in zip(methods,labels,palette):
  a=np.array([float(r['inlier_ratio'])*100 for r in raw[d][m]]);window=51;smooth=np.convolve(a,np.ones(window)/window,'valid');axes[j].plot(np.arange(window,len(a)+1),smooth,lw=1.2,label=label,color=col)
 axes[j].set_title(f'Dataset {j+1}: 51-pair trailing mean');axes[j].set_ylim(0,103);axes[j].set_ylabel('Inlier ratio (%)');axes[j].set_xlabel('Frame-pair index');axes[j].grid(alpha=.2)
axes[0].legend(ncol=5,fontsize=8,loc='lower left',frameon=False);fig.tight_layout();save('24_Temporal_Inlier_Ratio',fig)
fig,ax=plt.subplots(figsize=(10,3.5));ax.axis('off')
steps=['RGB frames\n(single topic)','Detect\nkeypoints','Describe\nlocal patches','2-NN +\nratio test','RANSAC\ngeometry','CSV +\nvisual evidence']
for i,s in enumerate(steps):
 x=.015+i*.165;patch=FancyBboxPatch((x,.48),.14,.29,boxstyle='round,pad=0.012',fc='#e6eef4',ec='#7392aa',lw=1.1);ax.add_patch(patch);ax.text(x+.07,.625,s,ha='center',va='center',fontsize=10)
 if i<5:ax.annotate('',xy=(x+.161,.625),xytext=(x+.145,.625),arrowprops={'arrowstyle':'->','color':'#4e6c83'})
ax.text(.5,.2,'Harris + LK: detect corners -> track local motion -> RANSAC -> statistics',ha='center',fontsize=11,color='#b56a37');ax.set_title('LAB5 feature matching and tracking pipeline',pad=18);save('18_Feature_Tracking_Pipeline',fig)
(out/'chart_data.json').write_text(json.dumps(meta,indent=2),encoding='utf-8');print('Created 7 reproducible chart PNG/PDF pairs')
