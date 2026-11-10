"""Capture real Lab6 outputs displayed on the Ubuntu desktop."""
import argparse
import base64
import csv
import io
import subprocess
import tkinter as tk
from pathlib import Path
from PIL import Image

root = Path.home() / 'lab6_complete'
shots = root / 'screenshots'
parser = argparse.ArgumentParser()
parser.add_argument('--extended', action='store_true', help='Capture the extended Office run')
args = parser.parse_args()
dataset = 'office_extended' if args.extended else 'office_preview'
bag = 'office_extended.bag' if args.extended else 'office_verified_prefix.bag'
log = 'office_extended_evaluate.log' if args.extended else 'office_evaluate.log'
results = root / 'results' / dataset
shots.mkdir(exist_ok=True)
window = tk.Tk()
window.title('Lab6 evidence viewer')
window.attributes('-fullscreen', True)
window.configure(bg='#112737')
window.bind('<Escape>', lambda _: window.destroy())
title = tk.Label(window, bg='#112737', fg='white', font=('Sans', 23, 'bold'))
title.pack(pady=(23, 9))
sub = tk.Label(window, bg='#112737', fg='#a9c5db', font=('Sans', 12))
sub.pack(pady=(0, 14))
body = tk.Label(window, bg='#112737', fg='#ecf4f9', font=('Monospace', 11), justify='left')
body.pack(expand=True)
foot = tk.Label(window, text='VNAV Lab6  |  Ubuntu evidence from the verified MIT Office segment',
                bg='#112737', fg='#a9c5db', font=('Sans', 11))
foot.pack(pady=14)

def capture(index, name, heading, source, content=None, image=None):
    title.config(text=f'{index:02d}  {heading}')
    sub.config(text=source)
    if image:
        with Image.open(image) as im:
            im.thumbnail((window.winfo_screenwidth()-48, window.winfo_screenheight()-156))
            buffer = io.BytesIO()
            im.save(buffer, format='PNG')
        body.photo = tk.PhotoImage(data=base64.b64encode(buffer.getvalue()).decode())
        body.config(image=body.photo, text='')
    else:
        body.config(image='', text=content)
    window.update()
    window.after(420)
    window.update()
    subprocess.run(['gnome-screenshot', '-f', str(shots/f'{index:02d}_{name}.png')], check=True)

build = (root/'logs'/'build.log').read_text().splitlines()
build = [line for line in build if any(term in line for term in (
    'Built target', 'Linking', 'Building CXX', 'Base path', 'Source space',
    'Build space', 'Devel space', 'Running command'))]
capture(9, 'Catkin_Build', 'Catkin build', 'Source: logs/build.log',
        '\n'.join(line[:105] for line in build[-17:]))
lines = (root/'logs'/log).read_text().strip().splitlines()
if args.extended and len(lines)>14:
    lines = lines[:5] + ['...'] + lines[-6:]
capture(10, 'Office_Calibration', 'Office input and camera calibration',
        f'Source: logs/{log}', '\n'.join(lines))
capture(11, 'Office_ROS_Topics', 'ROS bag topics and frame counts',
        f'Source: rosbag info data/{bag}',
        '\n'.join(line.rstrip() for line in subprocess.check_output(
            ['rosbag','info',str(root/'data'/bag)],text=True).splitlines()
            if not line.startswith(('path:', 'version:'))))
for index, method, heading in (
    (12,'NISTER5','Nister five-point inliers'),
    (13,'EIGHT8','Eight-point inliers'),
    (14,'KNOWN2','Known-rotation two-point inliers'),
    (15,'ARUN3','Arun RGB-D inliers'),
    (16,'NISTER5_RAW','Five-point single-hypothesis inliers')):
    capture(index, f'{method}_Inliers', heading, f'Source: results/{dataset}/{method}_inliers.png',
            image=results/f'{method}_inliers.png')
window.destroy()
