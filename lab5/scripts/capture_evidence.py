from pathlib import Path
import tkinter as tk
from PIL import Image
import io,base64
import subprocess,os,time,json
root=Path.home()/'lab5_complete'
items=[('03_SIFT_Keypoints',['pair/SIFT_keypoints1.png','pair/SIFT_keypoints2.png']),('04_SIFT_Raw_Matches',['pair/SIFT_raw.png']),('05_SIFT_Ratio_Filter',['pair/SIFT_ratio.png']),('06_SIFT_RANSAC',['pair/SIFT_inliers.png']),('07_SURF_RANSAC',['pair/SURF_inliers.png']),('08_ORB_RANSAC',['pair/ORB_inliers.png']),('09_FAST_BRIEF_RANSAC',['pair/FAST_BRIEF_inliers.png']),('12_Dataset1_LK',['dataset1/LK_tracks.png']),('13_Dataset2_LK',['dataset2/LK_tracks.png']),('14_Dataset1_Farneback',['dataset1/Farneback.png']),('15_Dataset2_Farneback',['dataset2/Farneback.png'])]
app=tk.Tk();app.geometry('1180x720+30+35');app.configure(bg='white');title=tk.Label(app,font=('Sans',18),bg='white');title.pack(pady=8);body=tk.Frame(app,bg='white');body.pack(expand=True,fill='both');footer=tk.Label(app,text='VMware Ubuntu 20.04 | Actual C++ OpenCV outputs | Source: ~/lab5_complete/results',bg='white',font=('Sans',11));footer.pack(pady=10)
def show(index=0):
 if index>=len(items):app.destroy();return
 name,paths=items[index]
 if not all((root/'results'/p).exists() for p in paths):app.after(10,lambda:show(index+1));return
 app.title('LAB5 '+name);title.config(text='LAB5 '+name.replace('_',' '));
 for child in body.winfo_children():child.destroy()
 app.photos=[]
 for p in paths:
  img=Image.open(root/'results'/p);img.thumbnail((1120//len(paths),590));buffer=io.BytesIO();img.save(buffer,format="PNG");photo=tk.PhotoImage(data=base64.b64encode(buffer.getvalue()));app.photos.append(photo);tk.Label(body,image=photo,bg='white').pack(side='left',expand=True)
 app.lift();app.update()
 def capture():
  subprocess.run(['gnome-screenshot','-f',str(root/'screenshots'/(name+'.png'))],check=True)
  app.after(500,lambda:show(index+1))
 app.after(1800,capture)
app.after(500,show);app.mainloop()
