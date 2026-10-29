#!/usr/bin/env python3
"""Record a new real Unity run, with timing from the odometry gate counter."""
import argparse
import ctypes
from datetime import datetime,timezone,timedelta
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def activate(window_id):
    """Request the observed Linux X11 Unity window, not a terminal, in front."""
    class Data(ctypes.Union):
        _fields_=[('l',ctypes.c_long*5)]
    class Message(ctypes.Structure):
        _fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),
                  ('send_event',ctypes.c_int),('display',ctypes.c_void_p),
                  ('window',ctypes.c_ulong),('message_type',ctypes.c_ulong),
                  ('format',ctypes.c_int),('data',Data)]
    class Event(ctypes.Union):
        _fields_=[('message',Message),('pad',ctypes.c_long*24)]
    x=ctypes.CDLL('libX11.so.6')
    x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=ctypes.c_void_p
    x.XDefaultRootWindow.argtypes=[ctypes.c_void_p];x.XDefaultRootWindow.restype=ctypes.c_ulong
    x.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int];x.XInternAtom.restype=ctypes.c_ulong
    x.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.POINTER(Event)]
    x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
    display=x.XOpenDisplay(None)
    if not display:raise RuntimeError('Cannot access the Ubuntu graphical session')
    try:
        event=Event();m=event.message
        m.type=33;m.send_event=1;m.display=display;m.window=window_id
        m.message_type=x.XInternAtom(display,b'_NET_ACTIVE_WINDOW',0)
        m.format=32;m.data.l[0]=2;m.data.l[1]=0
        # Leave space for the Ubuntu dock so the recording contains only Unity.
        move=Event();move.message.type=33;move.message.send_event=1
        move.message.display=display;move.message.window=window_id
        move.message.message_type=x.XInternAtom(display,b'_NET_MOVERESIZE_WINDOW',0)
        move.message.format=32;move.message.data.l[0]=(1<<8)|(1<<9)|(2<<12)
        move.message.data.l[1]=88;move.message.data.l[2]=64
        x.XSendEvent(display,x.XDefaultRootWindow(display),0,(1<<20)|(1<<19),ctypes.byref(move))
        x.XSendEvent(display,x.XDefaultRootWindow(display),0,(1<<20)|(1<<19),ctypes.byref(event))
        x.XFlush(display)
    finally:x.XCloseDisplay(display)


def unity_geometry():
    tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    lines=[line for line in tree.splitlines() if '"lab4.x86_64"' in line]
    if len(lines)!=1:raise RuntimeError('Expected exactly one visible Lab4 Unity window')
    window=int(re.search(r'0x[0-9a-fA-F]+',lines[0]).group(),16)
    activate(window);time.sleep(.4)
    info=subprocess.check_output(['xwininfo','-id',hex(window)],text=True)
    def value(label):return int(re.search(r'^\s*'+re.escape(label)+r':\s*(-?\d+)',info,re.M).group(1))
    width=value('Width');height=value('Height')
    return dict(window_id=hex(window),x=value('Absolute upper-left X'),
                y=value('Absolute upper-left Y'),width=width-width%2,height=height-height%2)


def write_overlay(path,timing=None):
    timing=timing or {}
    count=timing.get('passed_gates',0);phase=timing.get('phase','PREPARING')
    elapsed=timing.get('elapsed_ms')
    clock='---.--- s' if elapsed is None else '{:.3f} s'.format(elapsed/1000.)
    start=timing.get('first_gate_ros_s');end=timing.get('last_gate_ros_s')
    stamp=lambda value:'---.--- s' if value is None else '{:.3f} s'.format(value)
    text=('{} | Gates {}/12\nSTART (sim): {} | END (sim): {}\n{}: {} | Gate 1 to gate 12').format(
        phase,count,stamp(start),stamp(end),'FINAL' if end is not None else 'ELAPSED',clock)
    temp=path.with_suffix('.tmp');temp.write_text(text);os.replace(temp,path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    desktop=Path(subprocess.check_output(['xdg-user-dir','DESKTOP'],text=True).strip())
    parser.add_argument('--output-dir',type=Path,default=desktop)
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    token=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000000)
    video=args.output_dir/('Lab4_Sub4_'+token+'.mp4')
    data=args.output_dir/('Lab4_Sub4_'+token+'_data');data.mkdir()
    overlay=data/'timer.txt';write_overlay(overlay)
    recorder=None;flight=None;success=False
    lock=(ROOT/'results/.sub4_record.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another video recording is active')
    try:
        for program in ('ffmpeg','ffprobe','xwininfo'):
            import shutil
            if not shutil.which(program):raise RuntimeError('Missing tool: '+program)
        # Stop only this project, and wait for its original flight lock to close.
        subprocess.run(['bash',str(ROOT/'stop_sub4.sh')],check=True)
        with (ROOT/'results/.sub7_race.lock').open('a') as flight_lock:
            subprocess.run(['flock','-w','20',str(flight_lock.fileno()),'true'],
                           pass_fds=(flight_lock.fileno(),),check=True)
        launcher_log=data/'launcher.log'
        with launcher_log.open('w') as log:
            flight=subprocess.Popen(['bash',str(ROOT/'run_sub4_race.sh')],cwd=ROOT,
                                    stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                                    start_new_session=True)
        deadline=time.monotonic()+120
        directory=None
        while time.monotonic()<deadline:
            if flight.poll() is not None:raise RuntimeError('Flight launcher stopped; see '+str(launcher_log))
            match=re.search(r'^Logs: (.+)$',launcher_log.read_text(),re.M)
            if match:directory=Path(match.group(1));break
            time.sleep(.1)
        if directory is None:raise RuntimeError('Unity startup timed out')
        geometry=unity_geometry()
        if geometry['x']<0 or geometry['y']<0:raise RuntimeError('Move the Unity window fully onto the desktop and retry')
        (data/'capture.json').write_text(json.dumps(geometry,indent=2))
        filter_text=('drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf:'
                     'textfile='+str(overlay)+':reload=1:fontsize=17:fontcolor=white:'
                     'box=1:boxcolor=black@0.65:x=10:y=10')
        display=os.environ.get('DISPLAY',':0')
        if '.' not in display.split(':')[-1]:display+='.0'
        capture=display+'+{},{}'.format(geometry['x'],geometry['y'])
        with (data/'ffmpeg.log').open('w') as log:
            recorder=subprocess.Popen(['ffmpeg','-nostdin','-hide_banner','-loglevel','warning',
                '-f','x11grab','-draw_mouse','0','-framerate','25','-video_size',
                '{}x{}'.format(geometry['width'],geometry['height']),'-i',capture,
                '-vf',filter_text,'-c:v','libx264','-preset','ultrafast','-crf','18',
                '-threads','1','-pix_fmt','yuv420p','-movflags','+faststart','-n',str(video)],
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
        print('正在录制 Unity 窗口，请保持窗口可见。视频：'+str(video),flush=True)
        timing_path=ROOT/'results'/(directory.name[:-3]+'_timing.json')
        result_path=directory/'race_acceptance.json'
        deadline=time.monotonic()+240
        result=None
        while time.monotonic()<deadline:
            if recorder.poll() is not None:raise RuntimeError('Video recorder stopped; see '+str(data/'ffmpeg.log'))
            try:write_overlay(overlay,json.loads(timing_path.read_text()))
            except (OSError,ValueError):pass
            if result_path.exists():
                result=json.loads(result_path.read_text());break
            if flight.poll() is not None:raise RuntimeError('Flight ended before analysis; video retained')
            time.sleep(.1)
        if result is None:raise RuntimeError('Flight analysis timed out; video retained')
        timing=json.loads(timing_path.read_text())
        timing.update(phase='ACCEPTED' if result['passed'] and result['first_gate_to_last_gate_ms']<4000 else 'NOT ACCEPTED',
                      passed_gates=result['passed_gates'])
        write_overlay(overlay,timing)
        time.sleep(4.)
        recorder.send_signal(signal.SIGINT);recorder.wait(timeout=20);recorder=None
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams',
                                                  '-of','json',str(video)],text=True))
        metadata=dict(video=str(video),run_directory=str(directory),race_acceptance=result,
                      actual_run_under_4s=result['passed'] is True and result['first_gate_to_last_gate_ms']<4000,
                      recording='Unity window; real elapsed odometry timestamps; 25fps, no time scaling',
                      gate_timing=timing,capture=geometry,probe=probe)
        (data/'recording_result.json').write_text(json.dumps(metadata,indent=2))
        success=metadata['actual_run_under_4s']
        print('视频已保存：'+str(video),flush=True)
        print('本次：{:.3f} 秒，{}/12 门，验收 {}'.format(result['first_gate_to_last_gate_ms']/1000.,
              result['passed_gates'],'通过' if success else '未通过'),flush=True)
        print('原始录包与参数：'+str(directory),flush=True)
        if not success:raise RuntimeError('Recording retained, but this flight did not pass the sub-four-second check')
        print('无人机继续保持终点悬停。停止：bash '+str(ROOT/'stop_sub4.sh'),flush=True)
        return 0
    except KeyboardInterrupt:
        print('录制已中断；保留已录制内容。',flush=True)
        return 130
    except Exception as error:
        print('录制未完成：'+str(error),flush=True)
        return 1
    finally:
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:recorder.terminate();recorder.wait(timeout=5)
        if not success and flight is not None and flight.poll() is None:
            os.killpg(flight.pid,signal.SIGTERM)
        lock.close()


if __name__=='__main__':raise SystemExit(main())
