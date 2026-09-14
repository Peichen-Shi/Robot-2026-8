"""RoboMaster EP column scanner and cup/mouse sorter.
逻辑：扫描找目标 -> 看到目标后先往前走一段固定距离（测距传感器监控，够近就提前停）
     -> 到位后做一次有上限的左右居中 -> 夹取 -> 搬运 -> 放下 -> 机械臂回零
"""
import logging, queue, time, os, threading, socket, csv, math
from pathlib import Path
import cv2
import faulthandler
faulthandler.enable()
DEVICE_PREFER = 'cuda:0'
def _resolve_device(prefer):
    if not str(prefer).startswith('cuda'):
        return prefer
    try:
        import torch
        if not torch.cuda.is_available():
            print('[设备] 没检测到可用的 CUDA -> 用 CPU')
            return 'cpu'
        cap = torch.cuda.get_device_capability()
        want = 'sm_%d%d' % (cap[0], cap[1])
        have = torch.cuda.get_arch_list()
        if want not in have:
            print('[设备] 显卡算力 %s 不在 torch 编译列表 %s 里 -> 退回 CPU' % (want, have))
            return 'cpu'
        print('[设备] GPU 可用：%s (%s)  arch=%s' % (torch.cuda.get_device_name(0), want, want))
        return prefer
    except Exception as e:
        print('[设备] 检测 CUDA 时出错(%s) -> 退回 CPU' % str(e)[:80])
        return 'cpu'
DEVICE = _resolve_device(DEVICE_PREFER)
if DEVICE == 'cpu':
    os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
from robomaster import config as rm_config, robot
from ultralytics import YOLO
if not hasattr(rm_config, 'DEFAULT_CONN_PROTO'):
    rm_config.DEFAULT_CONN_PROTO = rm_config.DEFAULT_PROTO_TYPE
_HERE = Path(__file__).resolve().parent
USE_MODEL = 'coco_m'
MODEL_IMGSZ = {
    'trained': 1280,
    'coco_s':  1280,
    'coco_m':  1280,
    'coco_l':  1280,
}
CLASS_BY_MODEL = {
    'trained': 'cup',
    'coco_s':  'bottle',
    'coco_m':  'bottle',
    'coco_l':  'bottle',
}
_MODEL_CHOICES = {
    'trained': [
        _HERE/'models'/'best.pt',
        _HERE/'.venv'/'runs'/'detect'/'runs'/'train'/'mouse_bottle_exp-14'/'weights'/'best.pt',
    ],
    'coco_s': [
        Path(r'C:\Users\Lenovo\Desktop\yolov8s.pt'),
        _HERE/'models'/'yolov8s.pt',
    ],
    'coco_m': [
        _HERE/'models'/'yolov8m.pt',
    ],
    'coco_l': [
        _HERE/'models'/'yolov8l.pt',
    ],
}
MODEL_CANDIDATES = _MODEL_CHOICES.get(USE_MODEL, []) + \
    [p for k, v in _MODEL_CHOICES.items() if k != USE_MODEL for p in v]
MODEL = next((p for p in MODEL_CANDIDATES if p.is_file()), MODEL_CANDIDATES[0])
if not MODEL.is_file():
    raise FileNotFoundError('YOLO weights not found: {}  candidates: {}'.format(MODEL, [str(p) for p in MODEL_CANDIDATES]))
TARGET_CLASS = CLASS_BY_MODEL.get(USE_MODEL, 'cup')
TISSUE_ENABLE     = True
TISSUE_CLASS      = 'tissue'
TISSUE_CONF       = 0.60
TISSUE_MODEL_PATH = _HERE / 'models' / 'tissue_v2.pt'
RIGHT_SIGN, SCAN_SPEED, SCAN_PERIOD, MAX_SCAN = 1.0, .10, .08, 3.60
USE_FRAME_CONVERT = False
SCAN_USE_MOVE    = True
SCAN_STEP_M      = 0.06
SCAN_MOVE_SPEED  = 0.30
CONF_BY_CLASS = {TARGET_CLASS: .75}
if TISSUE_ENABLE: CONF_BY_CLASS[TISSUE_CLASS] = TISSUE_CONF
GROUND_MIN_Y = 300
CONF = min(CONF_BY_CLASS.values())
IMGSZ = MODEL_IMGSZ.get(USE_MODEL, 640)
CENTER_FRAC, CENTER_MIN_PX = .20, 20
CUP_WIDTH_M      = .07
SCAN_PULSE_MIN, SCAN_PULSE_MAX = .12, .60
SCAN_CENTER_SPEED = .15
LAT_FLIP_MARGIN_PX = 25
CONFIRM_HOLD_SEC   = 3.0
TARGET_CONFIRM_SEC = 1.5
BACKOFF_EXTRA_M  = 0.05
MAX_SCAN_SEC     = 120.0
ARM_X_MIN_MM     = 70
ARM_X_MAX_MM     = 205
ARM_X_FREE_MM    = 150
ARM_Y_MIN_MM     = 30
ARM_Y_MIN_EXT_MM = -25
ARM_Y_MAX_MM     = 120
HOME_X, HOME_Y   = 70, 45
ARM_LIFT_FOR_HOME = 60
SCAN_X, SCAN_Y = 200, -25
PICK_X, PICK_Y, SAFE_Y = 200, -25, 90
LEFT_ZONE, RIGHT_ZONE = -.85, .85
PLACE_BACKOFF_M  = 0.60
AREA_A_OFFSET_M  = 0.60
PLACE_STEP_M     = 0.15
AREA_B_OFFSET_M  = 0.90
ROW_OBJECTS      = 6
ROW_SPACING_M    = 0.30
CARRY_X, CARRY_Y = HOME_X, SAFE_Y
LAT_STEP_M       = 0.50
LAT_SPEED        = 0.70
LAT_STEP_TIMEOUT = 3.0
LAT_COMP_MAX     = 0.05
FWD_FIX_SPEED    = 0.25
DROP_LEFT_M      = AREA_A_OFFSET_M
DROP_ALIGN_X     = False
MAX_FORWARD_M    = 2.50
FWD_STEP_M       = 0.05
FWD_FINE_CM      = 30.0
FWD_STEP_FINE_M  = 0.010
FWD_PULSE_MIN_S  = 0.20
APPROACH_SPEED_FINE = 0.03
FWD_STEP_FINE2_M = 0.005
FWD_FINE2_BOX_PX = 250
APPROACH_SPEED   = .06
USE_TOF_STOP     = True
APPROACH_STOP_CM = 6.0
FINAL_CREEP_M    = 0.04
TOF_BACKTRACK_CM = 25.0
VISION_STOP_PX   = 700
LOST_ABORT_M     = 1.20
TOF_STALE_STOP_SEC = 1.5
FWD_LINE_TOL_M   = 0.035
BOX_SAFE_RATIO   = 2.50
GRASP_WIDTH_PX   = {TARGET_CLASS: 360}
OBJ_WIDTH_CM     = {TARGET_CLASS: 6.5, TISSUE_CLASS: 8.0}
if TISSUE_ENABLE: GRASP_WIDTH_PX[TISSUE_CLASS] = 220
def stop_px(label):
    ref = OBJ_WIDTH_CM[TARGET_CLASS]
    return int(round(VISION_STOP_PX * OBJ_WIDTH_CM.get(label, ref) / float(ref)))
MAX_APPROACH_SEC = 75.0
GRASP_CENTER_FRAC  = .05
ALIGN_TOF_CM   = 30.0
ALIGN_BOX_W_PX = 150.0
ALIGN_FRAC     = .10
ALIGN_MAX_MID  = 10
ALIGN_FRAC_END = .20
ALIGN_MAX_END  = 6
GRASP_XE_OFFSET_PX = 0
ALIGN_CENTER_SPEED = .06
ALIGN_PULSE_MIN, ALIGN_PULSE_MAX = .18, .40
ALIGN_DAMP      = .5
ALIGN_MIN_SCALE = .25
ALIGN_LOST_RETRY = 4
KEEP_HEADING     = True
HEADING_TOL_DEG  = 2.0
HEADING_Z_SPEED  = 25
LINE_KEEP        = False
POS_TOL_M        = 0.02
SCAN_POS_TOL_M   = 0.025
FWD_Y_COMP       = 0.00
FWD_Y_COMP_MAX   = 0.60
FWD_Y_COMP_ALPHA = 0.60
SCAN_X_COMP      = 0.50
SCAN_LOCK_FRAC   = 0.45
TEXT_SDK_ENABLE  = False
TEXT_SDK_PORT    = 40923
TEXT_SDK_TIMEOUT = 0.5
IR_SENSOR_ID     = 1
IR_QUERY_DIV     = 10.0
IR_QUERY_TTL     = 0.3
RANGE_MIN_CM, RANGE_MAX_CM = 0.5, 300.0
RANGE_FRESH_SEC = 2.0
TOF_MAX_JUMP_CM = 40.0
CAMERA_FEED_THREAD   = False
CAMERA_AUTO_RESTART  = True
CAMERA_FREEZE_FRAMES = 4
CAMERA_MAX_RESTARTS  = 8
CAM_RETRY_MAX       = 3
CAM_RETRY_READS     = 4
CAMERA_MOVE_M        = 0.03
CAMERA_DIFF_TH       = 1.5
CALIBRATE        = False
CALIB_HOVER_MM   = 30
ARM_POS_TOL_MM   = 15
ARM_STUCK_WARN_MM = 40
ARM_STUCK_ABORT  = True
CLOSE_GRASP_CM    = 4.0
GRIP_OPEN_POWER  = 40
GRIP_CLOSE_POWER = 45
GRIP_SECONDS     = 3.0
log = logging.getLogger('sorter'); logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
RUN_TS  = time.strftime('%Y%m%d_%H%M%S')
LOG_DIR = _HERE / 'logs'
try:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    _fh = logging.FileHandler(str(LOG_DIR / ('run_%s.log' % RUN_TS)), encoding='utf-8')
    _fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    log.addHandler(_fh)
except Exception as _e:
    print('[日志] 建不了 logs 目录或写不了文件（%s），本轮回退到只打屏幕' % _e)
CSV_PATH = LOG_DIR / ('steps_%s.csv' % RUN_TS)
CSV_COLS = ('t','event','it','label','xe','box_w','tof_cm','fwd_m','step_m',
            'vx','vy','yaw','pos_x','pos_y','arm_x','arm_y','note')
_T0 = time.time()
_csv_lock = threading.Lock()
_csv_head = [False]
def _fmt(v):
    if v is None: return ''
    if isinstance(v, float): return ('%.3f' % v).rstrip('0').rstrip('.')
    return str(v)
def csv_row(event, **kw):
    try:
        with _csv_lock:
            with open(str(CSV_PATH), 'a', encoding='utf-8-sig', newline='') as fp:
                w = csv.writer(fp)
                if not _csv_head[0]:
                    w.writerow(CSV_COLS); _csv_head[0] = True
                row = dict(kw); row['t'] = round(time.time() - _T0, 2); row['event'] = event
                w.writerow([_fmt(row.get(c)) for c in CSV_COLS])
    except Exception as e:
        try: log.warning('CSV 写不进去（%s）', e)
        except Exception: pass
def csv_state(s, tag):
    d = {}
    try:
        p = s.get_pos()
        if p: d['pos_x'], d['pos_y'] = p[0], p[1]
    except Exception: pass
    try:
        y = s.get_yaw()
        if y is not None: d['yaw'] = y
    except Exception: pass
    try:
        a = s.get_arm_pos()
        if a: d['arm_x'], d['arm_y'] = a[0], a[1]
    except Exception: pass
    d['note'] = tag
    return d
def norm180(a):
    """把角度差归一化到 (-180, 180]"""
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a
def s32(v):
    """把"无符号 32 位"回报的负数转回来。

    实测：机械臂 y=-20 时位置反馈报的是 4294967277（= 2^32 - 19），
    不转的话到位校验会算出 42 亿的偏差，误判成"超出可达范围"，
    每次都要白等 settle 再"按极限位置接受"。"""
    v=float(v)
    return v-4294967296.0 if v>2147483647.0 else v
class D:
    def __init__(self,label,conf,bbox): self.label,self.conf,self.bbox=label,conf,bbox
    @property
    def center(self):
        x1,y1,x2,y2=self.bbox; return ((x1+x2)//2,(y1+y2)//2)
    @property
    def area(self):
        x1,y1,x2,y2=self.bbox; return max(0,x2-x1)*max(0,y2-y1)
class Sorter:
    def __init__(self):
        self.ep=robot.Robot(); self.connected=False; self.camera=self.chassis=self.arm=self.gripper=None; self.scanned=0
        self.sensor=None
        self.model=YOLO(str(MODEL))
        log.info('YOLO loaded %s classes=%s',MODEL,self.model.names)
        self.models=[self.model]
        if TISSUE_ENABLE and TISSUE_MODEL_PATH.is_file():
            try:
                self.tissue_model=YOLO(str(TISSUE_MODEL_PATH))
                self.models.append(self.tissue_model)
                log.info('第二个模型（纸巾）loaded %s classes=%s',
                         TISSUE_MODEL_PATH,self.tissue_model.names)
            except Exception as e:
                log.warning('纸巾模型加载失败(%s) -> 本轮只认 %s',str(e)[:120],TARGET_CLASS)
        elif TISSUE_ENABLE:
            log.warning('找不到 %s -> 只认 %s（先跑 训练纸巾.py）',
                        TISSUE_MODEL_PATH,TARGET_CLASS)
        log.info('推理设备=%s  输入尺寸 imgsz=%d  (USE_MODEL=%s)',DEVICE,IMGSZ,USE_MODEL)
        try:
            import torch
            if str(DEVICE).startswith('cuda') and torch.cuda.is_available():
                log.info('GPU: %s  显存 %.1f GB',torch.cuda.get_device_name(0),
                         torch.cuda.get_device_properties(0).total_memory/1073741824.0)
        except Exception: pass
        try:
            import numpy as _np
            _blank=_np.zeros((720,1280,3),_np.uint8)
            _t0=time.time()
            for _m in self.models:
                _m.predict(_blank,imgsz=IMGSZ,conf=CONF,verbose=False,device=DEVICE,max_det=10)
            log.info('模型预热完成（%.1fs；共 %d 个模型，含 CUDA 内核加载，只做一次）',
                     time.time()-_t0,len(self.models))
        except Exception as e:
            log.warning('模型预热失败(%s)，扫描时第一次推理会慢',str(e)[:120])
        self._range_mm=[None]*4; self._range_lock=threading.Lock(); self._range_t=0.0; self._range_events=0
        self._tof_last=None; self._tof_last_t=0.0
        self._last_edge_log=0.0
        self._cam_io_lock=threading.RLock()
        self._yaw=None; self._yaw_lock=threading.Lock(); self._yaw_events=0; self._yaw_sign=-1.0
        self._pos=None; self._pos_lock=threading.Lock(); self._pos_events=0
        self._arm_pos=None; self._arm_lock=threading.Lock(); self._arm_events=0
        self._scan_log_t=0.0
        self._irq_val=None; self._irq_t=0.0; self._irq_raw=None
        self._tsock=None; self._tbuf=b''
    def connect(self):
        try:
            self.ep.initialize(conn_type='ap'); self.connected=True
            self.camera,self.chassis=self.ep.camera,self.ep.chassis; self.arm,self.gripper=self.ep.robotic_arm,self.ep.gripper
            self.sensor=self.ep.sensor
            self.text_sdk_connect()
            self.start_range()
            self.start_attitude()
            self.start_grip_status()
            self.start_position()
            self.start_arm_pos()
            if self.camera.start_video_stream(display=False,resolution='720p') is False: raise RuntimeError('video stream failed')
            self.frame(8)
            self.start_feeder()
            log.info('RoboMaster ready (测距事件=%d 次)', self._range_events)
        except Exception: self.stop(); raise
    def _cam_read(self,timeout=1.0,strategy='newest'):
        lk=getattr(self,'_cam_io_lock',None)
        if lk is None:
            lk=self._cam_io_lock=threading.RLock()
            log.warning('  _cam_io_lock 缺失 -> 在 _cam_read 里补建（请检查 __init__）')
        with lk:
            return self.camera.read_cv2_image(timeout=timeout,strategy=strategy)
    def start_feeder(self):
        if not CAMERA_FEED_THREAD: return
        self._feed_stop=False
        self._feed_thread=threading.Thread(target=self._feed_loop,daemon=True)
        self._feed_thread.start()
        log.info('✓ 只读喂帧线程已启动（长动作期间保持视频流流动）')
    def _feed_loop(self):
        while not getattr(self,'_feed_stop',True):
            try:
                self._cam_read(timeout=0.5)
            except Exception:
                pass
            time.sleep(0.02)
    def _feed_camera(self):
        try:
            self._cam_read(timeout=0.05)
        except Exception:
            pass
    def sleep_feed(self,sec,step=0.1):
        t0=time.time()
        while time.time()-t0<sec:
            self._feed_camera()
            time.sleep(min(step,max(0.0,sec-(time.time()-t0))))
    def wait_feed(self,a,timeout=15,name=''):
        self._feed_camera()
        ok=a.wait_for_completed(timeout=timeout)
        self._feed_camera()
        if not ok:
            log.warning('  [等待超时] %s（%.1fs）',name,timeout)
        return ok
    def restart_video(self,why=''):
        """★ 2026-09-12 晚改动：不再重启视频流，改成 丢帧重试 / 干净退出。

        依据（22:02 那次的完整崩溃调用栈）：
            Windows fatal exception: access violation
              media.py:108 in _h264_decode        <- SDK 自带的 C++ H264 解码器
              media.py:131 in _video_decoder_task
          取帧异常时调用 stop/start_video_stream 会和 SDK 解码线程抢，解码器
          拿到不完整数据就越界 -> 访问违例 -> 整个进程死（exit -1073741819）。
          访问违例不是 Python 异常，try/except 抓不到，只能从源头避免。

        现在的策略：
          前 CAM_RETRY_MAX 次 -> 只等一等、重读几帧（几十毫秒，绝不动视频流）
          实在不行            -> 抛异常干净退出，交给 启动识别夹取.py 看门狗重启
        """
        n=getattr(self,'_cam_retry',0)+1
        self._cam_retry=n
        log.warning('★ 摄像头异常（%s）-> 第 %d 次丢帧重试（不重启视频流）',why,n)
        for i in range(CAM_RETRY_READS):
            time.sleep(0.15)
            try:
                f=self._cam_read(timeout=2)
            except Exception:
                f=None
            if f is not None:
                try:
                    sig=hash(f[::64,::64].tobytes())
                except Exception:
                    sig=None
                if sig is not None and sig!=getattr(self,'_cam_sig',None):
                    self._cam_sig=sig; self._cam_same=0; self._cam_retry=0
                    log.info('  丢帧重试成功（第 %d/%d 次读到新画面）',i+1,CAM_RETRY_READS)
                    return True
        if n>=CAM_RETRY_MAX:
            log.error('★★ 摄像头连续 %d 次取不到新画面 -> 主动退出，交给看门狗重启程序。',n)
            log.error('   不在运行中重启视频流的原因：会触发 SDK 解码器访问违例'
                      '(Windows fatal exception: access violation @ media.py:108)')
            raise RuntimeError('camera dead -> exit for watchdog restart')
        log.warning('  本次仍没读到新画面（%d/%d 次），先继续用旧帧',n,CAM_RETRY_MAX)
        return False
    def frame(self,timeout=2,retry=True):
        return self._frame_direct(timeout,retry)
    def _frame_direct(self,timeout=2,retry=True):
        try: f=self._cam_read(timeout=timeout)
        except queue.Empty as e:
            if retry and CAMERA_AUTO_RESTART and getattr(self,'_cam_restarts',0)<CAMERA_MAX_RESTARTS:
                self.restart_video('取帧超时')
                return self.frame(timeout=timeout,retry=False)
            raise RuntimeError('camera frame timeout') from e
        if f is None:
            if retry and CAMERA_AUTO_RESTART and getattr(self,'_cam_restarts',0)<CAMERA_MAX_RESTARTS:
                self.restart_video('空帧')
                return self.frame(timeout=timeout,retry=False)
            raise RuntimeError('empty camera frame')
        if CAMERA_AUTO_RESTART and retry:
            _need_restart=None
            try:
                sig=hash(f[::64,::64].tobytes())
                if sig==getattr(self,'_cam_sig',None):
                    self._cam_same=getattr(self,'_cam_same',0)+1
                else:
                    self._cam_same=0; self._cam_sig=sig
                if self._cam_same>=CAMERA_FREEZE_FRAMES:
                    log.warning('★ 画面连续 %d 帧逐字节相同 -> 判定摄像头卡住',self._cam_same)
                    self._cam_same=0; self._cam_sig=None; _need_restart='画面冻结'
            except Exception: pass
            if _need_restart is None:
                try:
                    small=f[::32,::32]
                    p=self.get_pos()
                    last=getattr(self,'_cam_probe',None)
                    if last is not None:
                        pp,ss=last[0],last[1]
                        if (p is not None and pp is not None and small.shape==ss.shape):
                            moved=abs(p[0]-pp[0])+abs(p[1]-pp[1])
                            diff=float(cv2.absdiff(small,ss).mean())
                            if moved>CAMERA_MOVE_M and diff<CAMERA_DIFF_TH:
                                log.warning('★ 车移动了 %.3f m 但画面几乎没变（差异 %.2f）'
                                            '-> 判定画面不刷新',moved,diff)
                                _need_restart='画面不随运动变化'
                    self._cam_probe=(p,small.copy())
                except Exception: pass
            if _need_restart and getattr(self,'_cam_restarts',0)<CAMERA_MAX_RESTARTS:
                self._cam_probe=None
                self.restart_video(_need_restart)
                return self.frame(timeout=timeout,retry=False)
        return f
    def detect(self,f):
        out=[]
        for _m in getattr(self,'models',[self.model]):
            r=_m.predict(f,conf=CONF,imgsz=IMGSZ,max_det=10,verbose=False,device=DEVICE)[0]
            for b in r.boxes:
                label=str(r.names[int(b.cls[0])]).lower()
                if label not in CONF_BY_CLASS or float(b.conf[0])<CONF_BY_CLASS[label]:
                    continue
                x1,y1,x2,y2=[int(v) for v in b.xyxy[0].tolist()]
                if y2 < GROUND_MIN_Y:
                    continue
                out.append(D(label,float(b.conf[0]),(x1,y1,x2,y2)))
        return out
    def text_sdk_connect(self):
        if not TEXT_SDK_ENABLE: return False
        try:
            ip=getattr(self.ep,'ip',None) or '192.168.2.1'
            self._tsock=socket.create_connection((ip,TEXT_SDK_PORT),timeout=TEXT_SDK_TIMEOUT)
            self._tsock.settimeout(TEXT_SDK_TIMEOUT)
            self._tbuf=b''
            log.info('明文SDK 已连接 %s:%d',ip,TEXT_SDK_PORT)
            log.info('红外测距 打开: %r', self.text_sdk_cmd('ir_distance_sensor measure on'))
            raw=self.text_sdk_cmd('ir_distance_sensor distance %d ?'%IR_SENSOR_ID)
            v=self._parse_ir(raw)
            log.info('红外测距 试查询(id=%d): 原始=%r -> %s',IR_SENSOR_ID,raw,
                     ('%.1f cm'%v) if v is not None else '无有效值')
            return True
        except Exception as e:
            log.warning('明文SDK 连接失败(%s)；红外测距将退回 DDS 订阅值',e)
            self._tsock=None
            return False
    def text_sdk_cmd(self,cmd):
        """发一条明文命令并取回以 ';' 结尾的回复（会跳过 DDS 推送）"""
        if getattr(self,'_tsock',None) is None: return None
        try:
            if not cmd.endswith(';'): cmd=cmd+';'
            self._tsock.sendall(cmd.encode('utf-8'))
            deadline=time.time()+TEXT_SDK_TIMEOUT
            while time.time()<deadline:
                if b';' in self._tbuf:
                    line,self._tbuf=self._tbuf.split(b';',1)
                    s=line.decode('utf-8','replace').strip()
                    if not s or 'mpry:' in s: continue
                    return s
                try:
                    chunk=self._tsock.recv(1024)
                except socket.timeout:
                    return None
                if not chunk: return None
                self._tbuf+=chunk
        except Exception as e:
            log.warning('明文SDK 命令失败(%s): %s',cmd,e)
        return None
    @staticmethod
    def _parse_ir(raw):
        if raw is None: return None
        try:
            s=str(raw).strip().rstrip(';').strip()
            if not s: return None
            v=float(s)/IR_QUERY_DIV
        except Exception:
            return None
        return v if 1.0<=v<=500.0 else None
    def read_ir_query_cm(self):
        """带短缓存的红外测距查询；失败返回 None"""
        if getattr(self,'_tsock',None) is None: return None
        now=time.time()
        if now-getattr(self,'_irq_t',0.0)<IR_QUERY_TTL:
            return getattr(self,'_irq_val',None)
        self._irq_t=now
        raw=self.text_sdk_cmd('ir_distance_sensor distance %d ?'%IR_SENSOR_ID)
        self._irq_raw=raw
        self._irq_val=self._parse_ir(raw)
        return self._irq_val
    def start_range(self):
        if self.sensor is None: return False
        try:
            ok=self.sensor.sub_distance(freq=10,callback=self._on_range)
            log.info('测距传感器订阅: %s',ok); return bool(ok)
        except Exception as e:
            log.warning('测距传感器订阅失败(%s)',e); return False
    def _on_range(self,distance):
        try:
            with self._range_lock:
                for i,v in enumerate(list(distance)[:4]): self._range_mm[i]=v
                self._range_t=time.time(); self._range_events+=1
        except Exception: pass
    def read_range_cm(self):
        """返回 (距离cm 或 None, 4路原始cm)。
        优先用明文协议直接查询（最直接、带短缓存）；查不到再退回 DDS 订阅值。
        DDS 值的跳变过滤：车不可能在 0.1 秒内移动几十厘米，所以与上次可信值相差超过
        TOF_MAX_JUMP_CM 的读数判为"丢回波"（近距离光束越过目标后测到远处墙面），沿用上次的值。"""
        with self._range_lock:
            raw=list(self._range_mm); t=self._range_t
        cm=[None if v is None else v*0.1 for v in raw]
        q=self.read_ir_query_cm()
        if q is not None:
            self._tof_last=q; self._tof_last_t=time.time()
            return q,cm
        now=time.time()
        if t<=0 or now-t>RANGE_FRESH_SEC: return None,cm
        valid=[c for c in cm if c is not None and RANGE_MIN_CM<=c<=RANGE_MAX_CM]
        if not valid: return None,cm
        val=min(valid)
        if self._tof_last is not None and now-self._tof_last_t<3.0 and abs(val-self._tof_last)>TOF_MAX_JUMP_CM:
            return self._tof_last,cm
        self._tof_last=val; self._tof_last_t=now
        return val,cm
    def start_grip_status(self):
        self._grip_status=None; self._grip_status_t=0.0
        try:
            ok=self.gripper.sub_status(freq=10,callback=self._on_grip_status)
            log.info('夹爪状态订阅(sub_status): %s',ok)
            return bool(ok)
        except Exception as e:
            log.warning('夹爪状态订阅失败(%s) -> 只靠视觉自检',str(e)[:100]); return False
    def start_attitude(self):
        try:
            ok=self.chassis.sub_attitude(freq=10,callback=self._on_attitude)
            log.info('航向订阅(sub_attitude): %s',ok)
            time.sleep(0.5)
            if self.get_yaw() is None:
                log.warning('没收到航向数据，航向保持将不生效（横移可能仍会斜）')
            self.calibrate_yaw_sign()
            return bool(ok)
        except Exception as e:
            log.warning('航向订阅失败(%s)，航向保持不生效',e); return False
    def _on_attitude(self,*args):
        """SDK 不同版本回调形式不同：可能是一个元组 (yaw,pitch,roll)，也可能是三个参数。"""
        try:
            if len(args)==1 and isinstance(args[0],(tuple,list)):
                yaw=args[0][0]
            elif len(args)>=3:
                yaw=args[0]
            else:
                return
            with self._yaw_lock:
                self._yaw=float(yaw); self._yaw_events+=1
        except Exception: pass
    def get_yaw(self):
        lock=getattr(self,'_yaw_lock',None)
        if lock is None: return None
        with lock:
            return getattr(self,'_yaw',None)
    def calibrate_yaw_sign(self):
        """用一次很小的转动，确定 z 正方向与 yaw 正方向是否一致（不同固件可能相反），
        否则"修正航向"会越修越歪。"""
        y0=self.get_yaw()
        if y0 is None: return
        try:
            self.chassis.move(x=0,y=0,z=5,xy_speed=0.5,z_speed=HEADING_Z_SPEED).wait_for_completed(timeout=6)
            time.sleep(0.4)
            y1=self.get_yaw()
            if y1 is None: return
            d=norm180(y1-y0)
            if abs(d) < 1.0 or abs(d) > 8.0:
                log.warning('★ 航向标定读数 %+.1f° 不合理（指令仅 5°，本机历史 -3.4~-4.1）'
                            ' -> 保留符号 %+d，不采用这次结果', d, int(getattr(self,'_yaw_sign',-1.0)))
                self.fix_heading(y0,' 标定后')
                return
            self._yaw_sign = 1.0 if d>=0 else -1.0
            log.info('航向标定: 指令 z=+5° 使 yaw 变化 %+.1f° -> 修正符号 %+d',d,int(self._yaw_sign))
            self.fix_heading(y0,' 标定后')
        except Exception as e:
            log.warning('航向标定失败(%s)，使用默认符号',e)
    def yaw_anchor(self):
        return getattr(self,'_yaw0',None) if getattr(self,'_yaw0',None) is not None \
               else self.get_yaw()
    def fix_heading(self,yaw_ref,where=''):
        """航向闭环：偏了就转回来。这就是"横移不斜着跑"的关键。"""
        if not KEEP_HEADING or yaw_ref is None: return
        y=self.get_yaw()
        if y is None: return
        err=norm180(yaw_ref-y)
        if abs(err)<HEADING_TOL_DEG: return
        log.info('  [航向]%s 偏了 %+.1f°，转回来',where,err)
        try:
            sign=getattr(self,'_yaw_sign',1.0)
            self.chassis.move(x=0,y=0,z=sign*err,xy_speed=0.5,z_speed=HEADING_Z_SPEED).wait_for_completed(timeout=3)
            time.sleep(0.15)
        except Exception as e:
            log.warning('  [航向] 修正失败: %s',e)
    def start_position(self):
        try:
            ok=self.chassis.sub_position(cs=0,freq=10,callback=self._on_position)
            log.info('位置订阅(sub_position): %s',ok)
            time.sleep(0.4)
            if self.get_pos() is None:
                log.warning('没收到位置数据，位置闭环将不生效')
            return bool(ok)
        except Exception as e:
            log.warning('位置订阅失败(%s)，位置闭环不生效',e); return False
    def _on_position(self,*args):
        try:
            if len(args)==1 and isinstance(args[0],(tuple,list)):
                x,y=args[0][0],args[0][1]
            elif len(args)>=2:
                x,y=args[0],args[1]
            else:
                return
            with self._pos_lock:
                self._pos=(float(x),float(y)); self._pos_events+=1
        except Exception: pass
    def get_pos(self):
        lock=getattr(self,'_pos_lock',None)
        if lock is None: return None
        with lock:
            return getattr(self,'_pos',None)
    def world_to_body(self,dxw,dyw):
        if not USE_FRAME_CONVERT:
            return (dxw,dyw)
        y=self.get_yaw()
        th=math.radians(y or 0.0)
        return (dxw*math.cos(th)-dyw*math.sin(th),
                dxw*math.sin(th)+dyw*math.cos(th))
    def body_forward(self,p,ref=None):
        if p is None or ref is None: return 0.0
        th=math.radians(self.get_yaw() or 0.0)
        dxw,dyw=p[0]-ref[0],p[1]-ref[1]
        return dxw*math.cos(th)+dyw*math.sin(th)
    def body_lateral(self,p,ref=None):
        if p is None or ref is None: return 0.0
        th=math.radians(self.get_yaw() or 0.0)
        dxw,dyw=p[0]-ref[0],p[1]-ref[1]
        return -dxw*math.sin(th)+dyw*math.cos(th)
    def hold_body_forward(self,ref,label=''):
        if not LINE_KEEP or ref is None: return
        p=self.get_pos()
        if p is None: return
        bx=self.body_forward(p,ref)
        if abs(bx)<=SCAN_POS_TOL_M: return
        log.info('  [位置]%s 车体前向偏了 %+.3f m -> 修正（按 yaw 换算，不再盯世界 x）',
                 label,bx)
        spd=-FWD_FIX_SPEED if bx>0 else FWD_FIX_SPEED
        dur=min(1.2,max(0.06,abs(bx)/FWD_FIX_SPEED*0.4))
        self.chassis_pulse(x=spd,dur=dur,yaw_ref=self.yaw_anchor())
        log.info('  [位置]%s 修正后车体前向 %+.3f m（推了 %.2fs，速度 %.2fm/s）',
                 label,self.body_forward(self.get_pos(),ref),dur,FWD_FIX_SPEED)
    def hold_body_lateral(self,ref,label=''):
        if not LINE_KEEP or ref is None: return
        p=self.get_pos()
        if p is None: return
        by=self.body_lateral(p,ref)
        tol=FWD_LINE_TOL_M
        if abs(by)<=tol: return
        log.info('  [位置]%s 车体横向偏了 %+.3f m -> 修正（按 yaw 换算，不盯世界 y）',label,by)
        spd=-FWD_FIX_SPEED if by>0 else FWD_FIX_SPEED
        dur=min(1.2,max(0.06,abs(by)/FWD_FIX_SPEED*0.4))
        self.chassis_pulse(y=spd,dur=dur,yaw_ref=self.yaw_anchor())
    def hold_line(self,ref,axis,where='',tol=None):
        """位置闭环：把"不该动的那一轴"拉回来。
        横移(axis='x')时前后不该动；前进(axis='y')时左右不该动。"""
        if not LINE_KEEP or ref is None: return
        p=self.get_pos()
        if p is None: return
        cur=p[0] if axis=='x' else p[1]
        d=ref-cur
        if abs(d)<(POS_TOL_M if tol is None else tol): return
        log.info('  [位置]%s %s轴漂了 %+.3f m，拉回来',where,axis,d)
        spd=.05 if d>0 else -.05
        dur=min(1.2,max(0.08,abs(d)/0.05))
        bx,by=self.world_to_body(spd,0.0) if axis=='x' else self.world_to_body(0.0,spd)
        self.chassis_pulse(x=bx,y=by,dur=dur)
    def start_arm_pos(self):
        try:
            ok=self.arm.sub_position(freq=10,callback=self._on_arm_pos)
            log.info('机械臂位置订阅(sub_position): %s',ok)
            time.sleep(0.4)
            if self.get_arm_pos() is None:
                log.warning('没收到机械臂位置数据，到位核对将跳过')
            return bool(ok)
        except Exception as e:
            log.warning('机械臂位置订阅失败(%s)',e); return False
    def _on_arm_pos(self,*args):
        try:
            if len(args)==1 and isinstance(args[0],(tuple,list)):
                x,y=args[0][0],args[0][1]
            elif len(args)>=2:
                x,y=args[0],args[1]
            else:
                return
            with self._arm_lock:
                self._arm_pos=(s32(x),s32(y)); self._arm_events+=1
        except Exception: pass
    def get_arm_pos(self):
        lock=getattr(self,'_arm_lock',None)
        if lock is None: return None
        with lock:
            return getattr(self,'_arm_pos',None)
    def go_pose_y_only(self,ty,what=''):
        """只改高度：x 保持当前位置不动。

        用户要求：回零完就让机械臂自己下降到 y=-20 再识别，x 不用管。
        这样下降是**纯垂直动作**，不会同时前后伸 —— 既不会去跟"退不回去的 x"较劲，
        也不会有斜向扫动把瓶子带倒。"""
        got=self.get_arm_pos()
        tx=int(round(got[0])) if got else SCAN_X
        log.info('  [机械臂] %s：x 保持 %d 不动，只把 y 降到 %d',what,tx,ty)
        return self.go_pose(tx,ty,what)
    def go_pose(self,tx,ty,what='',timeout=20,retry=True,settle=0.6):
        """把机械臂移到目标位，并用位置反馈核对是否真的到位。

        关键：位置反馈在超出可达范围时会**钳位**（实测指令 y=0 读数报 30）。
        所以偏差超容差时先等 settle 秒再读一次：
          读数没变 -> 手臂已经停住不动了 = 这就是它能到的极限位置，接受；
          读数还在变 -> 才是真的没走完，报警并重试一次。
        这样既保住"有没有真的到位"的排查能力，又不会对钳位值白重试（每次省约 2 秒）。"""
        _tx0,_ty0=tx,ty
        tx=min(max(tx,ARM_X_MIN_MM),ARM_X_MAX_MM)
        _ymin=ARM_Y_MIN_MM if tx<ARM_X_FREE_MM else ARM_Y_MIN_EXT_MM
        ty=min(max(ty,_ymin),ARM_Y_MAX_MM)
        if (tx,ty)!=(_tx0,_ty0):
            log.warning('  [机械臂] %s 目标 (%d,%d) 超出实测可达范围 -> 钳到 (%d,%d)'
                        '（x∈[%d,%d]；y下限 %d 或 %d 视 x 是否 >= %d 而定）',
                        what,_tx0,_ty0,tx,ty,ARM_X_MIN_MM,ARM_X_MAX_MM,
                        ARM_Y_MIN_MM,ARM_Y_MIN_EXT_MM,ARM_X_FREE_MM)
            try:
                csv_row('arm_clamp',note='%s (%d,%d)->(%d,%d)'%(what,_tx0,_ty0,tx,ty))
            except Exception: pass
        self.wait(self.arm.moveto(x=tx,y=ty),what,timeout=timeout)
        got=self.get_arm_pos()
        if got is None:
            log.info('  [机械臂] %s 无位置反馈，跳过到位校验',what); return
        dx,dy=got[0]-tx,got[1]-ty
        if abs(dx)<=ARM_POS_TOL_MM and abs(dy)<=ARM_POS_TOL_MM:
            self._arm_stuck=False
            log.info('  [机械臂] %s 已到位 (%.0f, %.0f)',what,got[0],got[1])
            csv_row('arm_ok',arm_x=got[0],arm_y=got[1],note='%s 指令(%d,%d)'%(what,tx,ty))
            return
        time.sleep(settle)
        g2=self.get_arm_pos()
        if g2 is not None and abs(g2[0]-got[0])<=1.0 and abs(g2[1]-got[1])<=1.0:
            if abs(dx)>ARM_STUCK_WARN_MM or abs(dy)>ARM_STUCK_WARN_MM:
                log.warning('  [机械臂] ★ %s 到不了 (%.0f, %.0f)！指令 (%d, %d)，'
                            '差了 (%.0f, %.0f) mm，且读数稳定 = 手臂卡住/被挡住/零位漂了。'
                            '请检查机械臂（必要时重启机器人重新回零）',
                            what,g2[0],g2[1],tx,ty,dx,dy)
                log.warning('='*72)
                log.warning('★★★ 机械臂卡住了：%s   指令=(%d,%d)  实际=(%.0f,%.0f)  偏差=(%.0f,%.0f)mm',
                            what,tx,ty,g2[0],g2[1],dx,dy)
                log.warning('★★★ 这条会污染后面所有视觉结论（手臂挡在镜头前），先解决它再谈别的：')
                log.warning('★★★ 断电重启 -> 只回零(0,0)（先别降 y）-> 看能否回到 x<180')
                log.warning('='*72)
                self._arm_stuck=True
                csv_row('arm_stuck',arm_x=g2[0],arm_y=g2[1],
                        note='%s 指令(%d,%d) 偏差(%.0f,%.0f)'%(what,tx,ty,dx,dy))
            else:
                log.info('  [机械臂] %s 停在 (%.0f, %.0f)，指令 (%d, %d) 超出可达范围（固件钳位），'
                         '读数 %.1fs 无变化 -> 按极限位置接受',
                         what,g2[0],g2[1],tx,ty,settle)
            return
        log.warning('  [机械臂] %s 未到位! 实际=(%.0f, %.0f) 目标=(%d, %d) 偏差=(%.0f, %.0f) mm',
                    what,got[0],got[1],tx,ty,dx,dy)
        if retry:
            log.info('  [机械臂] 重试一次')
            self.wait(self.arm.moveto(x=tx,y=ty),what+' retry',timeout=timeout)
            g3=self.get_arm_pos()
            log.info('  [机械臂] 重试后=(%s)',
                     ('%.0f, %.0f'%g3) if g3 is not None else '无数据')
    def home_arm(self):
        got=self.get_arm_pos()
        log.info('[归零] 起始读数 %s',('(%.0f, %.0f)'%got) if got else '无')
        log.info('[归零] ① 先抬后收：抬 y 到 %d -> 收 x 到 %d -> 落到 y=%d',
                 ARM_LIFT_FOR_HOME,HOME_X,HOME_Y)
        cur=self.get_arm_pos(); cx=int(round(cur[0])) if cur else ARM_X_MAX_MM
        self.go_pose(cx,ARM_LIFT_FOR_HOME,'归零①先抬y')
        self.go_pose(HOME_X,ARM_LIFT_FOR_HOME,'归零①收x')
        self.go_pose(HOME_X,HOME_Y,'归零①落下')
        if self._at_home():
            log.info('[归零] ① 成功 -> 手臂已到家 (x=%d, y=%d)',HOME_X,HOME_Y)
            self._arm_stuck=False; return True
        log.warning('[归零] ① 没到家 -> ② 再直接发一次 (x=%d, y=%d)',HOME_X,HOME_Y)
        self.go_pose(HOME_X,HOME_Y,'归零②直接')
        if self._at_home():
            log.info('[归零] ② 成功'); self._arm_stuck=False; return True
        log.warning('[归零] ② 没到家 -> ③ 相对移动退 x 两次（move() 是相对，moveto() 是绝对）')
        try:
            self.wait(self.arm.move(x=-100,y=0),'归零③相对退100',timeout=8)
            self.wait(self.arm.move(x=-100,y=0),'归零③相对退100-2',timeout=8)
        except Exception as e:
            log.warning('[归零] ③ 抛异常: %s',e)
        time.sleep(0.5)
        if self._at_home():
            log.warning('[归零] ③ 成功 —— 绝对坐标系漂了，但舵机是好的。'
                        '以后回零改用相对移动'); self._arm_stuck=False; return True
        got=self.get_arm_pos()
        log.error('[归零] ①②③ 全部失败 -> 判定真卡住。当前读数 %s',
                  ('(%.0f, %.0f)'%got) if got else '无')
        self._arm_stuck=True
        return False
    def _at_home(self):
        p=self.get_arm_pos()
        if p is None: return False
        return abs(p[0]-HOME_X)<=ARM_POS_TOL_MM and abs(p[1]-HOME_Y)<=ARM_POS_TOL_MM
    def chassis_pulse(self,x=0.0,y=0.0,speed=.06,dur=.15,yaw_ref=None,line_ref=None,line_axis=None,line_tol=None):
        self.chassis.drive_speed(x=x,y=y,z=0,timeout=dur)
        self.sleep_feed(dur+0.05)
        self.stop_drive()
        self.fix_heading(yaw_ref,' 横移后' if y else ' 前进后')
        if line_ref is not None and line_axis:
            _w=' 横移后' if y else ' 前进后'
            if line_axis=='bodyy':    self.hold_body_lateral(line_ref,_w)
            elif line_axis=='bodyx':  self.hold_body_forward(line_ref,_w)
            else:                     self.hold_line(line_ref,line_axis,_w,tol=line_tol)
    def stop_drive(self):
        try: self.chassis.drive_speed(x=0,y=0,z=0)
        except Exception: pass
    def show(self,f,ds,target,state,ratio=None,d_tof=None):
        c=f.copy(); h,w=c.shape[:2]
        cv2.line(c,(w//2,0),(w//2,h),(255,0,255),2)
        if target:
            box_w=target.bbox[2]-target.bbox[0]
            base_w=int(GRASP_WIDTH_PX.get(target.label,GRASP_WIDTH_PX[TARGET_CLASS])*h/720.0)
            cv2.line(c,(w//2-base_w//2,h-28),(w//2+base_w//2,h-28),(255,255,0),3)
            cv2.line(c,(w//2-box_w//2,h-14),(w//2+box_w//2,h-14),(0,255,255),3)
            cv2.putText(c,'box %d / limit %d px'%(box_w,base_w),(w//2-base_w//2,max(20,h-34)),0,.5,(255,255,0),1)
        for d in ds:
            x1,y1,x2,y2=d.bbox; col=(0,255,0) if d is target else (0,165,255); cv2.rectangle(c,(x1,y1),(x2,y2),col,2); cv2.putText(c,'{} {:.2f}'.format(d.label,d.conf),(x1,max(25,y1-8)),0,.65,col,2)
        cv2.putText(c,'{} scanned={:.2f}m Q=stop'.format(state,self.scanned),(15,30),0,.7,(255,255,255),2)
        tof_txt = ('%.1f' % d_tof) if d_tof is not None else '-'
        cv2.putText(c,'tof=%s cm   box_ratio=%s'%(tof_txt,('%.2f'%ratio) if ratio is not None else '-'),(15,58),0,.6,(0,255,255),2)
        try:
            cv2.imshow('RoboMaster Sorter',c)
            if cv2.waitKey(1)&255 in (ord('q'),ord('Q')): raise KeyboardInterrupt
        except KeyboardInterrupt:
            raise
        except Exception as e:
            self._gui_fail=getattr(self,'_gui_fail',0)+1
            if self._gui_fail<=3:
                log.warning('预览窗口异常(%s)，已忽略（不影响识别与抓取）',e)
    def lat_sign(self):
        if not hasattr(self,'_lat_sign'):
            self._lat_sign=1.0
        return self._lat_sign
    def _cal_lat_sign(self,ye,xe):
        if ye is None or xe is None: return
        last=getattr(self,'_cal_last',None)
        self._cal_last=(ye,xe)
        if last is None: return
        dy=ye-last[0]; dxe=xe-last[1]
        if abs(dy)<0.02 or abs(dxe)<15: return
        sign=1.0 if (dy*dxe)<0 else -1.0
        old=getattr(self,'_lat_sign',None)
        self._lat_sign=sign
        if old is None or abs(old-sign)>0.5:
            log.warning('★ 横移方向自学：dy=%+.3f m 引起 dxe=%+.0f px -> 横向方向符号 '
                        '%s 改为 %+d（原来那个是猜的）',
                        dy,dxe,('初值' if old is None else '%+d'%int(old)),int(sign))
    @staticmethod
    def center_ok(xe,box_w,frac=None):
        """紫线（画面正中 w//2）是否落在瓶子中间：容差按框宽比例算。
        固定像素在近距离会显得"没在正中"（框大），远距离又过于苛刻。
        frac=None 用扫描档(CENTER_FRAC)；夹取前传 GRASP_CENTER_FRAC，要求严得多。"""
        f=CENTER_FRAC if frac is None else frac
        return abs(xe)<=max(CENTER_MIN_PX,f*max(1,box_w))
    @staticmethod
    def center_tol_px(box_w,frac=None):
        f=CENTER_FRAC if frac is None else frac
        return int(max(CENTER_MIN_PX,f*max(1,box_w)))
    @staticmethod
    def center_pulse_dur(xe,box_w,speed=None,lo=None,hi=None):
        """估算"把紫线挪到瓶子中间"需要横移多久。

        小孔模型：需要横移的距离 L ≈ 瓶径 × xe / 框宽
          （焦距和距离在推导中约掉了：L = d·xe/f，而 d ≈ f·瓶径/框宽）
        所以不需要标定相机，也不需要知道瓶子多远。

        夹取前用更慢的 speed 和更小的下限(lo/hi)，这样最后一步不会跨太大。
        """
        est=CUP_WIDTH_M*abs(xe)/max(1.0,float(box_w))
        spd=SCAN_CENTER_SPEED if speed is None else speed
        _lo=SCAN_PULSE_MIN if lo is None else lo
        _hi=SCAN_PULSE_MAX if hi is None else hi
        return min(_hi,max(_lo,est/max(.01,spd)))
    def tof_stale_sec(self):
        """测距距离"上一次可信读数"过了多久（秒）。用于判断测距有没有真的在更新。"""
        t=getattr(self,'_tof_last_t',0.0)
        return 999.0 if not t else time.time()-t
    def scan(self):
        yaw_ref=self.yaw_anchor()
        p=self.get_pos()
        x_ref=p[0] if p else None
        y0=p[1] if p else None
        if getattr(self,'_scan_x0',None) is None and x_ref is not None:
            self._scan_x0=x_ref
            self._ref_pose=(x_ref,y0)
            log.info('★ 记下扫描起点：世界(x=%.3f, y=%.3f) —— 后面所有"前后/左右"'
                     '都相对它按车体坐标系换算',x_ref,y0 if y0 is not None else 0.0)
        self.scanned=0.0
        self._cam_restarts=0
        if not hasattr(self,'_scan_drift'): self._scan_drift=0.0
        self._cam_same=0
        self._cal_last=None
        t_scan0=time.time()
        if not hasattr(self,'_lat_sign'):
            self._lat_sign=getattr(self,'_yaw_sign',1.0)
            log.info('横向居中方向符号初值 = %+d（沿用航向标定符号；若走反会自动翻转）',int(self._lat_sign))
        if not hasattr(self,'_lat_flips'):   self._lat_flips=0
        if not hasattr(self,'_lat_flip_t'):  self._lat_flip_t=0.0
        log.info('横向方向符号 = %+d（已翻转 %d 次）',int(self._lat_sign),self._lat_flips)
        self._lat_probe=None
        _scan_ref=(x_ref,y0)
        log.info('扫描开始，航向基准 yaw=%s，前后基准 x=%s（本轮最多横移 %.1f m / %.0f s）',
                 ('%.1f°'%yaw_ref) if yaw_ref is not None else '无IMU',
                 ('%.3f m'%x_ref) if x_ref is not None else '无位置', MAX_SCAN, MAX_SCAN_SEC)
        while self.scanned<MAX_SCAN and time.time()-t_scan0<MAX_SCAN_SEC:
            f=self.frame(); ds=self.detect(f)
            t=min(ds,key=lambda d:abs(d.center[0]-f.shape[1]//2),default=None)
            if t is not None:
                w=f.shape[1]; xe=t.center[0]-w//2; box_w=max(1,t.bbox[2]-t.bbox[0])
                _p=self.get_pos()
                self._cal_lat_sign(
                    self.body_lateral(_p,(x_ref,y0)) if _p is not None else None,xe)
                _now=time.time()
                if getattr(self,'_t_seen_since',None) is None:
                    self._t_seen_since=_now
                    log.info('  看到 %s（框宽%d）开始计时：连续 %.1fs 才认',
                             t.label,box_w,TARGET_CONFIRM_SEC)
                _held=_now-self._t_seen_since
                _centered=self.center_ok(xe,box_w,SCAN_LOCK_FRAC)
                if _centered and _held<TARGET_CONFIRM_SEC:
                    self._confirming=True
                    self._confirm_deadline=_now+CONFIRM_HOLD_SEC
                    log.info('  紫线已在中间，但只连续看到 %.1fs（< %.1fs）-> 先不锁，原地确认',
                             _held,TARGET_CONFIRM_SEC)
                    self.stop_drive(); time.sleep(SCAN_PERIOD*3); continue
                if (not _centered) and getattr(self,'_confirming',False) \
                        and _now<getattr(self,'_confirm_deadline',0):
                    log.info('  确认中：本帧偏了 (xe=%d, 容差%d) -> 原地等，不往右扫',
                             xe,self.center_tol_px(box_w,SCAN_LOCK_FRAC))
                    self.stop_drive(); time.sleep(SCAN_PERIOD*3); continue
                self._confirming=False
                if _centered:
                    self.stop_drive(); self.show(f,ds,t,'TARGET LOCKED')
                    log.info('★ 紫线已在杯子中间 xe=%d (容差%d, 框宽%d) -> 锁定并开始前进：%s conf=%.2f bbox=%s',
                             xe,self.center_tol_px(box_w),box_w,t.label,t.conf,t.bbox)
                    pp=self.get_pos()
                    _d=self.body_forward(pp,(x_ref,y0))
                    if pp is not None and abs(_d)>0.03:
                        log.info('锁定后回正：车体前向漂了 %+.3f m -> 一次性拉回起点',_d)
                        self.chassis_pulse(x=(-0.05 if _d>0 else 0.05),
                                           dur=min(1.2,max(0.08,abs(_d)/0.05)),yaw_ref=yaw_ref)
                    elif pp is not None:
                        log.info('锁定后无需回正：车体前向 %+.3f m（世界x变化 %+.3f m 是坐标系效应）',
                                 _d,pp[0]-x_ref)
                    return self.approach(t.label)
                now=time.time()
                if now-getattr(self,'_last_edge_log',0.0)>1.0:
                    self._last_edge_log=now
                    log.info('看到 %s 但没在正中 (xe=%d, 容差%d) -> 继续往右扫（已取消左右微调）',
                             t.label,xe,self.center_tol_px(box_w))
                self.show(f,ds,t,'EDGE xe=%d keep scanning'%xe)
            else:
                if (getattr(self,'_confirming',False)
                        and time.time()<getattr(self,'_confirm_deadline',0)):
                    log.info('  确认中：本帧没检到目标 -> 原地等，不往右扫')
                    self.stop_drive(); time.sleep(SCAN_PERIOD*3); continue
                self._t_seen_since=None
                self._confirming=False
                self.show(f,ds,None,'SCAN')
            if SCAN_USE_MOVE:
                try:
                    _c=getattr(self,'_scan_drift',0.0)
                    _pb=self.get_pos()
                    self.chassis.move(x=-_c,y=RIGHT_SIGN*SCAN_STEP_M,z=0,
                                      xy_speed=SCAN_MOVE_SPEED,
                                      z_speed=HEADING_Z_SPEED).wait_for_completed(timeout=5)
                    _pa=self.get_pos()
                    if _pb is not None and _pa is not None:
                        _dbx=(self.body_forward(_pa,_scan_ref)
                              -self.body_forward(_pb,_scan_ref))
                        _nc=max(-0.08,min(0.08,_c+1.0*_dbx))
                        self._scan_drift=_nc
                        if abs(_dbx)>0.01:
                            log.info('  车体前向漂移 %+.3f m（世界x变化 %+.3f，车头 %+.1f°）'
                                     '-> 补偿 %+.3f -> %+.3f',
                                     _dbx,_pa[0]-_pb[0],self.get_yaw() or 0.0,_c,_nc)
                    self.stop_drive()
                    self.fix_heading(yaw_ref,' 横移后')
                    self.hold_body_forward((x_ref,y0),' 横移后')
                except Exception as e:
                    log.warning('位置控制横移失败(%s) -> 退回速度脉冲',e)
                    self.chassis_pulse(x=-SCAN_X_COMP*RIGHT_SIGN*SCAN_SPEED,
                                       y=RIGHT_SIGN*SCAN_SPEED,dur=.5,yaw_ref=yaw_ref,
                                       line_ref=x_ref,line_axis='x',line_tol=SCAN_POS_TOL_M)
            else:
                self.chassis_pulse(x=-SCAN_X_COMP*RIGHT_SIGN*SCAN_SPEED, y=RIGHT_SIGN*SCAN_SPEED,
                                   dur=.5,yaw_ref=yaw_ref,line_ref=x_ref,line_axis='x',
                                   line_tol=SCAN_POS_TOL_M)
            pp=self.get_pos()
            if pp is not None and y0 is not None:
                self.scanned=abs(self.body_lateral(pp,(x_ref,y0)))
            else:
                self.scanned+=SCAN_SPEED*0.5
            now=time.time()
            if now-getattr(self,'_scan_log_t',0.0)>1.0:
                self._scan_log_t=now
                yy=self.get_yaw()
                log.info('  扫描 %.2f m  yaw=%s  pos=%s  车体前向=%+.3f m 补偿=%+.3f',self.scanned,
                         ('%.1f°'%yy) if yy is not None else '-',
                         ('(x=%.3f, y=%.3f)'%pp) if pp else '-',
                         self.body_forward(pp,_scan_ref),getattr(self,'_scan_drift',0.0))
        self.stop_drive(); return None
    def approach(self,label):
        self._round_label=label
        t_start=time.time(); fwd=0.0; it=0; last_t=None; lost=0; lost_at=0.0; stopped_by='fixed'
        self._align_mid_done=False; self._align_mid_ok=False
        self._tof_min=None
        self._stop_tof=None
        yaw_ref=self.yaw_anchor()
        p0=self.get_pos(); y_ref=p0
        log.info('approach %s: 朝目标一直走，直到测距<=%.0fcm 停下（%s）；'
                 '安全上限 %.2f m；进到 %.0fcm 内改 %.0fmm 细步。航向基准=%s 左右基准=%s',
                 label,APPROACH_STOP_CM,
                 ('再盲走 %.0fmm'%(FINAL_CREEP_M*1000)) if FINAL_CREEP_M>0 else '不盲走',
                 MAX_FORWARD_M,FWD_FINE_CM,FWD_STEP_FINE_M*1000,
                 ('%.1f°'%yaw_ref) if yaw_ref is not None else '无IMU',
                 ('(x=%.3f, y=%.3f)'%y_ref) if y_ref is not None else '无位置')
        while fwd<MAX_FORWARD_M-1e-6 and time.time()-t_start<MAX_APPROACH_SEC:
            it+=1
            f=self.frame(); ds=[d for d in self.detect(f) if d.label==label]; t=max(ds,key=lambda d:(d.area,d.conf),default=None)
            d_tof,cm4=self.read_range_cm()
            _tof4=','.join('-' if c is None else '%.1f'%c for c in cm4)
            if d_tof is not None:
                if self._tof_min is None or d_tof<self._tof_min:
                    self._tof_min=d_tof
                elif d_tof>self._tof_min+TOF_BACKTRACK_CM:
                    log.warning('★ 测距不降反升（本段最小 %.1fcm，本帧 %.1fcm）-> '
                                '判定在读背景，本帧按"无测距"处理',self._tof_min,d_tof)
                    d_tof=None
            if USE_TOF_STOP and d_tof is not None and d_tof<=APPROACH_STOP_CM:
                log.info('测距 %.1fcm <= %.0fcm -> 已到位，停止前进',d_tof,APPROACH_STOP_CM)
                stopped_by='tof'
                self._stop_tof=d_tof
                _r={'tof_cm':d_tof,'fwd_m':fwd}; _r.update(csv_state(self,'stop=tof'))
                csv_row('stop',**_r)
                break
            if t is not None:
                lost=0; last_t=t
                h,w=f.shape[:2]; box_w=max(1,t.bbox[2]-t.bbox[0])
                self._last_seen_box=(t.center[0],box_w)
                ratio=box_w/(GRASP_WIDTH_PX.get(label,GRASP_WIDTH_PX[TARGET_CLASS])*h/720.0)
                if ratio>BOX_SAFE_RATIO:
                    log.info('目标框已达 %.2f 倍上限宽度，停止前进（防撞）',ratio); stopped_by='box'
                    _r={'tof_cm':d_tof,'box_w':box_w,'fwd_m':fwd}
                    _r.update(csv_state(self,'stop=box'))
                    csv_row('stop',**_r)
                    break
                _vsp=stop_px(label)
                if box_w>=_vsp:
                    log.info('★ 视觉停车：框宽 %dpx >= %dpx（%s 的阈值，已按物宽换算：'
                             '物宽%s cm）-> 停止前进',
                             box_w,_vsp,label,OBJ_WIDTH_CM.get(label,'?'),)
                    stopped_by='vision'
                    _r={'tof_cm':d_tof,'box_w':box_w,'fwd_m':fwd}
                    _r.update(csv_state(self,'stop=vision'))
                    csv_row('stop',**_r)
                    break
                _mid_by=None
                if d_tof is not None and d_tof<=ALIGN_TOF_CM:
                    _mid_by='测距 %.1fcm'%d_tof
                elif box_w>=ALIGN_BOX_W_PX:
                    _mid_by='框宽 %dpx(>=%.0f)'%(box_w,ALIGN_BOX_W_PX)
                if (not self._align_mid_done) and _mid_by:
                    self.stop_drive()
                    log.info('已到中距（%s）-> 先在这里做中距横向对准（框覆盖较全、像素换算合理）',_mid_by)
                    ok,last=self.align_lateral(label,yaw_ref,ALIGN_FRAC,ALIGN_MAX_MID,'ALIGN中距')
                    self._align_mid_done=True; self._align_mid_ok=bool(ok)
                    if last is not None: last_t=last
                    p=self.get_pos()
                    if p is not None: y_ref=p
                    continue
                xe=t.center[0]-f.shape[1]//2
                tol_now=self.center_tol_px(box_w)
                if abs(xe) > tol_now*1.5:
                    dur=self.center_pulse_dur(xe,box_w,ALIGN_CENTER_SPEED,ALIGN_PULSE_MIN,ALIGN_PULSE_MAX)
                    lat=self.lat_sign()*(1 if xe>0 else -1)*ALIGN_CENTER_SPEED
                    pa=self.get_pos()
                    self.chassis_pulse(y=lat,dur=dur,yaw_ref=yaw_ref,
                                       line_ref=pa,line_axis='bodyx')
                    log.info('  前进中横向纠正：xe=%d (容差%d) 横移%.2fs 朝%s',
                             xe,self.center_tol_px(box_w),dur,'右' if lat>0 else '左')
                    p=self.get_pos()
                    if p is not None: y_ref=p
            else:
                lost+=1
                if lost==1: lost_at=fwd
                if lost==10:
                    alls=self.detect(f)
                    log.warning('目标连续丢失 %d 帧（已走 %.2f m），但测距仍可用 -> 继续前进；'
                                '本帧检到的是：%s',lost,fwd,
                                ', '.join('%s(%.2f)'%(d.label,d.conf) for d in alls) or '什么都没检到')
                stale=self.tof_stale_sec()
                if stale>TOF_STALE_STOP_SEC:
                    log.warning('★ 视觉和测距同时失效：测距已 %.1fs 没有新的可信读数 '
                                '(容器里最后读数 %s cm) -> 停止前进，绝不盲开',
                                stale,('%.1f'%d_tof) if d_tof is not None else '无')
                    stopped_by='blind'
                    _r={'tof_cm':d_tof,'fwd_m':fwd}; _r.update(csv_state(self,'stop=blind'))
                    csv_row('stop',**_r)
                    break
                if fwd-lost_at>LOST_ABORT_M:
                    log.warning('目标丢失后又走了 %.2f m（累计 %.2f m）-> 放弃前进',fwd-lost_at,fwd)
                    stopped_by='lost'
                    _r={'tof_cm':d_tof,'fwd_m':fwd}; _r.update(csv_state(self,'stop=lost'))
                    csv_row('stop',**_r)
                    break
                box_w=None; ratio=None
            near  = (d_tof is not None and d_tof<=FWD_FINE_CM)
            finer = (box_w is not None and box_w>=FWD_FINE2_BOX_PX)
            if finer:
                step_unit=FWD_STEP_FINE2_M; spd=APPROACH_SPEED_FINE
            elif near:
                step_unit=FWD_STEP_FINE_M;  spd=APPROACH_SPEED
            else:
                step_unit=FWD_STEP_M;       spd=APPROACH_SPEED
            step=min(step_unit,MAX_FORWARD_M-fwd)
            if t is None and d_tof is None:
                log.warning('★ 本帧视觉和测距都没有数据 -> 停止前进（已走 %.2f m），绝不盲开',fwd)
                stopped_by='blind_step'
                _r={'fwd_m':fwd}; _r.update(csv_state(self,'stop=blind_step'))
                csv_row('stop',**_r)
                break
            _dur=max(step/spd,FWD_PULSE_MIN_S)
            _k=getattr(self,'_fwd_y_comp',FWD_Y_COMP)
            _pb=self.get_pos()
            self.chassis_pulse(x=spd,y=-_k*spd,dur=_dur,
                               yaw_ref=None,line_ref=y_ref,line_axis='bodyy')
            _pa=self.get_pos()
            if _pa is not None and _pb is not None and step>1e-6:
                try:
                    _dy=self.body_lateral(_pa)-self.body_lateral(_pb)
                except Exception:
                    _dy=None
                if _dy is not None:
                    _raw=max(-FWD_Y_COMP_MAX,min(FWD_Y_COMP_MAX,_dy/step))
                    _k=FWD_Y_COMP_ALPHA*_k+(1.0-FWD_Y_COMP_ALPHA)*_raw
                    self._fwd_y_comp=_k
                    if abs(_dy)>0.004:
                        log.info('  前进横向漂 %+.3f m（本步 %.3f m）-> 补偿系数 %.3f'
                                 ' -> 下一步 y=%+.4f',_dy,step,_k,-_k*spd)
            fwd+=step
            self.show(f,ds,t,'FWD %.2f/%.2f m'%(fwd,MAX_FORWARD_M),ratio,d_tof)
            log.info('it=%d %s fwd=%.2f/%.2f m box_w=%s ratio=%s conf=%s tof=%s  4路=[%s]',
                     it,label,fwd,MAX_FORWARD_M,
                     box_w if box_w is not None else '-',
                     ('%.2f'%ratio) if ratio is not None else '-',
                     ('%.2f'%t.conf) if t is not None else '-',
                     '%.1f'%d_tof if d_tof is not None else '-',_tof4)
            _r={'it':it,'label':label,
                'xe':(t.center[0]-f.shape[1]//2) if t is not None else None,
                'box_w':box_w,'tof_cm':d_tof,'fwd_m':fwd,'step_m':step}
            _r.update(csv_state(self,('finest' if finer else ('fine' if near else 'coarse'))
                                +' tof4='+_tof4))
            csv_row('fwd',**_r)
            time.sleep(.05)
        self.fix_heading(yaw_ref,' 前进结束')
        self.stop_drive(); time.sleep(.2)
        _ft,_=self.read_range_cm()
        if _ft is not None:
            self._stop_tof=_ft
        log.info('★ 停车读数 = %s cm（抓取路径按这个决定，不看抓取时的瞬时噪声）',
                 ('%.1f'%self._stop_tof) if getattr(self,'_stop_tof',None) is not None else '无数据')
        if stopped_by=='tof' and FINAL_CREEP_M>0:
            log.info('盲走补偿 %.0f mm（测距此段不可见）',FINAL_CREEP_M*1000)
            self.chassis_pulse(x=APPROACH_SPEED,dur=FINAL_CREEP_M/APPROACH_SPEED)
            fwd+=FINAL_CREEP_M
        log.info('前进阶段结束：累计 %.2f m（%d 轮 / %.1fs，停止原因=%s）',fwd,it,time.time()-t_start,stopped_by)
        _pe=self.get_pos()
        if _pe is not None and p0 is not None:
            _real=_pe[0]-p0[0]
            log.info('★ 前进量核对：代码计数 %.2f m  vs  底盘实际 %.2f m（差 %+.2f m）',
                     fwd,_real,_real-fwd)
            if abs(_real-fwd)>0.05:
                log.warning('★ 两者差 >5cm：还有别的动作在偷偷推车 —— '
                            '别用计数 fwd 判断距离，也别拿这一轮的落点去改阈值')
        _pe2=self.get_pos()
        _note='停止原因=%s（必须是 tof 才算受控停车）'%stopped_by
        if _pe2 is not None and p0 is not None:
            _note+=' 计数%.2fm/实际%.2fm'%(fwd,_pe2[0]-p0[0])
        _r={'fwd_m':fwd,'it':it,'tof_cm':self.read_range_cm()[0],'note':_note}
        _r.update(csv_state(self,'approach_end'))
        csv_row('approach_end',**_r)
        if stopped_by!='tof':
            log.warning('★ 注意：这次不是测距停的（%s）-> 停车距离不受控，'
                        '别拿这一轮的落点去改 APPROACH_STOP_CM',stopped_by)
        if self._align_mid_done and not self._align_mid_ok:
            log.info('中距对准没成功（目标丢失），最后再补一次大偏差校正')
        self.align_lateral(label,yaw_ref,ALIGN_FRAC_END,ALIGN_MAX_END,'ALIGN终',GRASP_XE_OFFSET_PX)
        try:
            f=self.frame(); ds=[d for d in self.detect(f) if d.label==label]
            t2=max(ds,key=lambda d:(d.area,d.conf),default=None)
        except Exception:
            t2=None
        if t2 is not None:
            try:
                self._pre_close_box=(t2.center[0],t2.bbox[2]-t2.bbox[0])
            except Exception: pass
        elif getattr(self,'_last_seen_box',None) is not None:
            self._pre_close_box=self._last_seen_box
            log.info('[自检] 夹取前那一帧没检到瓶子 -> 改用接近过程中最后一次检测（框宽%d）',
                     self._last_seen_box[1])
        if t2 is not None:
            xe2=t2.center[0]-f.shape[1]//2-GRASP_XE_OFFSET_PX
            bw2=max(1,t2.bbox[2]-t2.bbox[0])
            if self.center_ok(xe2,bw2,ALIGN_FRAC_END):
                log.info('★ 夹取前确认：紫线在瓶子中间 (xe=%d, 容差%d) 框=%s',
                         xe2,self.center_tol_px(bw2,ALIGN_FRAC_END),t2.bbox)
            else:
                log.warning('★ 夹取前紫线仍偏 (xe=%d, 容差%d) 框=%s -> 仍会尝试夹取',
                            xe2,self.center_tol_px(bw2,ALIGN_FRAC_END),t2.bbox)
            last_t=t2
        return last_t if last_t is not None else D(label,0.0,(0,0,0,0))
    def align_lateral(self,label,yaw_ref,frac,max_pulses,tag,xe_offset=0.0):
        pa=self.get_pos(); xa_ref=pa
        log.info('%s：要求紫线落在瓶子中间 ±%.0f%% 框宽以内%s',tag,frac*100,
                 ('（含固定偏移 %+dpx）'%xe_offset) if xe_offset else '')
        best_abs=None; scale=1.0; last=None; self._align_flips=0
        for k in range(1,max_pulses+1):
            f=self.frame(); ds=[d for d in self.detect(f) if d.label==label]
            t=max(ds,key=lambda d:(d.area,d.conf),default=None)
            if t is None:
                seen=''
                for _try in range(ALIGN_LOST_RETRY):
                    time.sleep(0.12)
                    f=self.frame(); ds_all=self.detect(f)
                    ds=[d for d in ds_all if d.label==label]
                    if ds:
                        t=max(ds,key=lambda d:(d.area,d.conf)); break
                    seen=(', '.join('%s(%.2f)'%(d.label,d.conf) for d in ds_all) or '什么都没检到')
                if t is None:
                    log.warning('★ %s：目标始终丢失 -> 跳过本次横向校正（重试 %d 次；最后检到：%s）',
                                tag,ALIGN_LOST_RETRY,seen)
                    return False,last
            last=t
            h,w=f.shape[:2]; xe=t.center[0]-w//2-xe_offset; box_w=max(1,t.bbox[2]-t.bbox[0])
            _x1,_y1,_x2,_y2=t.bbox
            if _y2>=h-2 or _y1<=1 or _x1<=1 or _x2>=w-2:
                log.warning('★ %s：目标框被画面边缘裁掉 (%d,%d,%d,%d / %dx%d) '
                            '-> 框中心不可信，跳过横向脉冲（避免照噪声左右乱蹭）',
                            tag,_x1,_y1,_x2,_y2,w,h)
                csv_row('align',xe=xe,box_w=box_w,label=label,note='%s 框被裁->跳过'%tag)
                return False,last
            tol=self.center_tol_px(box_w,frac)
            if abs(xe)<=tol:
                log.info('%s 已居中 xe=%d (容差%d, 框宽%d)',tag,xe,tol,box_w)
                csv_row('align',xe=xe,box_w=box_w,label=label,note='%s 居中成功'%tag)
                return True,last
            a=abs(xe)
            if (best_abs is not None and a>best_abs+LAT_FLIP_MARGIN_PX
                    and self._align_flips<1):
                self._align_flips+=1
                self._lat_sign=-self.lat_sign()
                log.warning('★ %s 偏差反而变大 (%.0f -> %d)：横移方向反了 -> '
                            '符号翻转为 %+d（第 %d 次）',
                            tag,best_abs,a,int(self._lat_sign),self._align_flips)
                best_abs=None; scale=1.0
                time.sleep(0.15)
                continue
            if best_abs is not None and a>=best_abs-2:
                scale=max(ALIGN_MIN_SCALE,scale*ALIGN_DAMP)
                log.info('%s 偏差没变小 (%.0f -> %d) -> 步子缩到 %.0f%%',tag,best_abs,a,scale*100)
            best_abs=a if best_abs is None else min(best_abs,a)
            dur=min(ALIGN_PULSE_MAX,max(ALIGN_PULSE_MIN,
                    self.center_pulse_dur(xe,box_w,ALIGN_CENTER_SPEED,ALIGN_PULSE_MIN,ALIGN_PULSE_MAX)*scale))
            lat=self.lat_sign()*(1 if xe>0 else -1)*ALIGN_CENTER_SPEED
            self.chassis_pulse(y=lat,dur=dur,yaw_ref=yaw_ref,line_ref=xa_ref,
                               line_axis='bodyx')
            self.show(f,ds,t,'%s %d/%d'%(tag,k,max_pulses))
            log.info('%s %d/%d xe=%d (容差%d) 脉冲%.2fs(%.0f%%) 朝%s（方向符号%+d）框=%s',
                     tag,k,max_pulses,xe,tol,dur,scale*100,
                     '右' if lat>0 else '左',int(self.lat_sign()),t.bbox)
            csv_row('align',xe=xe,box_w=box_w,label=label,
                    note='%s %d/%d 脉冲%.2fs 朝%s'%(tag,k,max_pulses,dur,
                                                    '右' if lat>0 else '左'))
        self.stop_drive()
        csv_row('align',label=label,note='%s 用满 %d 次仍未居中 -> 放弃'%(tag,max_pulses))
        return False,last
    def wait(self,a,name,timeout=20):
        self._feed_camera()
        t0=time.time(); ok=a.wait_for_completed(timeout=timeout); dt=time.time()-t0
        self._feed_camera()
        if not ok:
            log.error('  [动作超时] %s（等了 %.1fs）',name,dt); raise RuntimeError(name+' timeout')
        if hasattr(a,'has_succeeded') and not a.has_succeeded:
            log.error('  [动作失败] %s',name); raise RuntimeError(name+' failed')
        log.info('  [动作完成] %s（%.1fs）',name,dt)
        time.sleep(.4)
    def _on_grip_status(self,status):
        try:
            s=status[0] if isinstance(status,(tuple,list)) else status
        except Exception:
            s=None
        self._grip_status=str(s) if s else None
        self._grip_status_t=time.time()
    def grip_status(self,max_age=2.0):
        s=getattr(self,'_grip_status',None); t=getattr(self,'_grip_status_t',0.0)
        if s is None or (time.time()-t)>max_age: return None
        return s
    @staticmethod
    def grip_status_text(s):
        return {'normal':'normal(夹住了)','closed':'closed(夹空了)',
                'opened':'opened(没闭合)'}.get(s,('无数据' if s is None else str(s)))
    def grip(self,open_):
        act='张开' if open_ else '闭合'
        pw=GRIP_OPEN_POWER if open_ else GRIP_CLOSE_POWER
        log.info('  [夹爪] %s（power=%d）',act,pw)
        r=(self.gripper.open if open_ else self.gripper.close)(power=pw)
        if not r: raise RuntimeError('gripper command failed')
        self.sleep_feed(GRIP_SECONDS)
        self.gripper.pause()
        _st=self.grip_status()
        log.info('  [夹爪] %s 完成；实测状态=%s',act,self.grip_status_text(_st))
        if open_ and _st is not None and _st!='opened':
            log.warning('  [夹爪] 张开后实测=%s（不是 opened）-> 再张开一次',
                        self.grip_status_text(_st))
            try:
                (self.gripper.open)(power=pw)
                self.sleep_feed(GRIP_SECONDS); self.gripper.pause()
                log.info('  [夹爪] 重试后实测=%s',self.grip_status_text(self.grip_status()))
            except Exception as _e:
                log.warning('  [夹爪] 再张开失败(%s)',str(_e)[:80])
    def pick(self):
        if CALIBRATE:
            y=PICK_Y+CALIB_HOVER_MM
            log.info('【校准模式】机械臂下到抓取点上方 %dmm（x=%d, y=%d），不会闭合夹爪',CALIB_HOVER_MM,PICK_X,y)
            self.wait(self.arm.moveto(x=PICK_X,y=y),'calib hover')
            return False
        tof,_=self.read_range_cm()
        stop_tof=getattr(self,'_stop_tof',None)
        judge = stop_tof if stop_tof is not None else tof
        try:
            _t,_ = self.read_range_cm()
            _tgt = 3.0
            for _k in range(3):
                if _t is None or _t > 60.0: break
                _d = _t - _tgt
                if abs(_d) <= 0.5: break
                _m = min(abs(_d), 6.0) / 100.0
                if _d < 0: _m = -_m
                log.info('★ 夹取前距离校正 %d：实测 %.1fcm（目标 %.1f）-> %s %.1fcm',
                         _k+1, _t, _tgt, '前进' if _d > 0 else '后退', abs(_d))
                try:
                    self.wait_feed(self.chassis.move(x=_m, y=0, z=0, xy_speed=.5,
                                                     z_speed=HEADING_Z_SPEED),
                                   8, '夹取前距离校正')
                except Exception as _e:
                    log.warning('  距离校正动作失败(%s)', str(_e)[:60]); break
                self.stop_drive(); time.sleep(.2)
                _t,_ = self.read_range_cm()
            if _t is not None:
                log.info('★ 夹取前距离校正完成：实测 %.1fcm（目标 %.1f）', _t, _tgt)
        except Exception as _e:
            log.warning('夹取前距离校正跳过(%s)', str(_e)[:80])
        log.info('抓取路径判据：停车读数 %s cm，此刻读数 %s cm -> %s',
                 ('%.1f'%stop_tof) if stop_tof is not None else '无数据',
                 ('%.1f'%tof) if tof is not None else '无数据',
                 '就近夹' if (judge is not None and judge<=CLOSE_GRASP_CM) else '伸臂到 PICK_X')
        if judge is not None and judge<=CLOSE_GRASP_CM:
            log.warning('★ 停车读数只有 %.1fcm（<= %.0f）已经贴得很近 -> 不再伸机械臂，'
                        '就在当前 x 直接夹取',judge,CLOSE_GRASP_CM)
            _mode='close_range(不伸臂)'
            self.go_pose_y_only(PICK_Y,'close-range down')
        else:
            log.info('开始抓取：机械臂下到 PICK(x=%d, y=%d)（当前测距 %s）',
                     PICK_X,PICK_Y,('%.1fcm'%tof) if tof is not None else '无数据')
            _mode='extend(伸到PICK_X)'
            self.go_pose(PICK_X,PICK_Y,'down to object')
        _r={'tof_cm':tof,'note':'%s 停车读数=%s'%(_mode,
                ('%.1f'%stop_tof) if stop_tof is not None else '无')}
        _r.update(csv_state(self,'before_close'))
        csv_row('grasp',**_r)
        self.grip(False)
        _r={'tof_cm':self.read_range_cm()[0],
            'note':'夹爪已闭合(功率%d)'%GRIP_CLOSE_POWER}
        _r.update(csv_state(self,'after_close'))
        csv_row('grasp',**_r)
        self.go_pose(PICK_X,SAFE_Y,'lift object')
        log.info('运输前收臂 -> 运输姿态 (x=%d, y=%d)（x 回到回零位，高度保持抬起）',
                 CARRY_X,CARRY_Y)
        self.go_pose(CARRY_X,CARRY_Y,'carry home')
        _ok=self._grasp_selfcheck(getattr(self,'_round_label',None))
        try:
            _box=getattr(self,'_pre_close_box',None)
            _after=self.read_range_cm()[0]
            csv_row('grasp_attempt',
                    box_w=(_box[1] if _box else None),
                    tof_cm=getattr(self,'_stop_tof',None),
                    arm_x=(self.get_arm_pos() or (None,None))[0],
                    note='闭合后=%.1fcm 停车读数=%s 结果=%s' % (
                        _after if _after is not None else -1,
                        ('%.1f'%self._stop_tof) if getattr(self,'_stop_tof',None) is not None else '无',
                        {True:'夹到', False:'没夹到', None:'判不出'}[_ok]))
        except Exception: pass
        return True
    def _grasp_selfcheck(self,label=None):
        label=label or TARGET_CLASS
        log.info('[自检] 夹爪状态 = %s（normal=夹住了/closed=夹空/opened=没闭合）',
                 self.grip_status_text(self.grip_status()))
        before=getattr(self,'_pre_close_box',None)
        try:
            f=self.frame(); ds=[d for d in self.detect(f) if d.label==label]
            t=max(ds,key=lambda d:(d.area,d.conf),default=None)
        except Exception as e:
            log.warning('[自检] 取帧失败(%s)，跳过夹取结果判断',e); return None
        if before is None:
            log.info('[自检] 夹取前没记到瓶子位置，无法判断结果（本次%s）',
                     '画面里已无 '+label if t is None else '仍看得到 '+label)
            return None
        if t is None:
            log.info('★★ 夹取结果：成功（抬起来后画面里看不到 %s 了，说明被夹走了）',label)
            try: csv_row('grasp_result',note='成功(抬起后已看不到)')
            except Exception: pass
            return True
        dx=abs(t.center[0]-before[0]); bw=t.bbox[2]-t.bbox[0]; dw=abs(bw-before[1])
        if dx<40 and dw<max(30,0.25*before[1]):
            log.error('★★ 夹取结果：★失败★（%s 还在原地：横向只差 %d px、框宽只差 %d px）'
                      '-> 夹爪从瓶子旁边过去了，建议重试或检查停车距离',label,dx,dw)
            try: csv_row('grasp_result',note='失败(瓶子还在原地 dx=%d dw=%d)'%(dx,dw))
            except Exception: pass
            return False
        log.info('★★ 夹取结果：成功（%s 位置已明显变化：dx=%d px, d框宽=%d px）',label,dx,dw)
        try: csv_row('grasp_result',note='成功(位置变化 dx=%d dw=%d)'%(dx,dw))
        except Exception: pass
        return True
    def move_y(self,d):
        yaw_ref=self.yaw_anchor()
        log.info('  [底盘] 横向移动 %.2f m',d)
        _bx,_by=self.world_to_body(0.0,d)
        a=self.chassis.move(x=_bx,y=_by,z=0,xy_speed=.5,z_speed=20)
        if not self.wait_feed(a,25,'横向移动'):
            log.error('  [底盘] 横向移动超时'); raise RuntimeError('zone move timeout')
        self.stop_drive(); time.sleep(.3)
        self.fix_heading(yaw_ref,' 横移后')
        log.info('  [底盘] 横向移动完成')
    def return_to_scan_depth(self):
        ref=getattr(self,'_ref_pose',None)
        p=self.get_pos()
        if ref is None or p is None: return
        dx=-(self.body_forward(p,ref)+BACKOFF_EXTRA_M)
        if abs(dx)<=0.01:
            log.info('★ 已在原点深度（车体前向 %+.3f m），无需后退',-dx); return
        log.info('★ 后退 %.3f m 回到原点深度（当前车体前向 %+.3f m）',-dx,-dx)
        try:
            _bx,_by=self.world_to_body(dx,0.0)
            self.wait_feed(self.chassis.move(x=_bx,y=_by,z=0,xy_speed=.4,z_speed=20),
                           15,'退回扫描深度')
            self.stop_drive(); time.sleep(.25)
            self.fix_heading(self.yaw_anchor(),' 退回深度后')
        except Exception as e:
            log.warning('  退回扫描深度失败: %s',e)
        p2=self.get_pos()
        if p2 is not None:
            log.info('  退回后车体前向 %+.3f m（残差 %+.3f m）',
                     self.body_forward(p2,ref),self.body_forward(p2,ref))
    def drop_target_lat(self,label,cur_lat):
        L=getattr(self,'_pick_y1',None)
        if L is None: L=cur_lat
        if str(label or '').lower()==TISSUE_CLASS:
            ty=L+(ROW_OBJECTS-1)*ROW_SPACING_M+AREA_B_OFFSET_M
            log.info('★ 落点=区域B（纸巾）：最左端 %.3f + (%d-1)×%.2f + %.2f -> 目标 %.3f',
                     L,ROW_OBJECTS,ROW_SPACING_M,AREA_B_OFFSET_M,ty)
        else:
            ty=L-AREA_A_OFFSET_M
            log.info('★ 落点=区域A（水瓶）：最左端 %.3f - %.2f -> 目标 %.3f',
                     L,AREA_A_OFFSET_M,ty)
        _key=('B' if str(label or '').lower()==TISSUE_CLASS else 'A')
        _cnt=getattr(self,'_place_n',{})
        _k=_cnt.get(_key,0)
        if _k>0:
            _sh=(PLACE_STEP_M*_k) if _key=='B' else (-PLACE_STEP_M*_k)
            log.info('  落点错开：本区第 %d 次放置 -> 再偏 %+.2f m（目标 %.3f -> %.3f）',
                     _k+1,_sh,ty,ty+_sh)
            ty=ty+_sh
        _cnt[_key]=_k+1; self._place_n=_cnt
        return ty
    def go_to_drop_pose(self,label=None):
        cur=self.get_pos()
        if cur is None:
            _tis=(str(label or '').lower()==TISSUE_CLASS)
            _off=AREA_B_OFFSET_M if _tis else AREA_A_OFFSET_M
            _sgn=1.0 if _tis else -1.0
            log.warning('★ 没有位置反馈 -> 落点退回固定横移 %+.2f m（%s）',
                        _sgn*_off,'区域B' if _tis else '区域A')
            self.move_y(_sgn*RIGHT_SIGN*_off)
            return False
        ref=getattr(self,'_ref_pose',None) or cur
        bl=self.body_lateral(cur,ref)
        if getattr(self,'_pick_y1',None) is None:
            self._pick_y1=bl
            self._last_pick_lat=bl
            self._pick_x1=cur[0]
            log.info('★ 第 1 个被抓物体的横向位置 = %.3f（整排最左端）',bl)
        self._last_pick_lat=bl
        if getattr(self,'_row_max_lat',None) is None or bl>self._row_max_lat:
            self._row_max_lat=bl
        ty=self.drop_target_lat(label,bl)
        dy=ty-bl
        log.info('★ 落点计算（车体横向）：本轮拿起 %.3f（整排最左 %.3f / 最右 %.3f）'
                 '-> %s = %.3f（要横移 %+.3f m）',
                 bl,self._pick_y1,self._row_max_lat,
                 '区域B(纸巾)' if str(label or '').lower()==TISSUE_CLASS else '区域A(水瓶)',
                 ty,dy)
        if abs(dy)>0.02:
            self.move_lateral_straight(dy,'落点横移')
            time.sleep(.25)
            self.fix_heading(None,' 落点横移后')
        if DROP_ALIGN_X and getattr(self,'_pick_x1',None) is not None:
            p=self.get_pos()
            if p is not None and abs(p[0]-self._pick_x1)>0.03:
                dx=self._pick_x1-p[0]
                log.info('★ 落点前后对齐 %.3f m（DROP_ALIGN_X=True）',dx)
                self.chassis.move(x=dx,y=0,z=0,xy_speed=.4,z_speed=20).wait_for_completed(timeout=15)
                self.stop_drive(); time.sleep(.25)
        p=self.get_pos()
        if p is not None:
            bl2=self.body_lateral(p,ref)
            log.info('★ 到达落点：车体横向 %.3f（目标 %.3f，残差 %+.3f m）',bl2,ty,bl2-ty)
            try:
                csv_row('drop_pose',pos_x=p[0],pos_y=p[1],
                        note='拿起%.3f 目标%.3f 残差%+.3f'%(bl,ty,bl2-ty))
            except Exception: pass
        return True
    def place_return(self,label):
        self.return_to_scan_depth()
        self.go_to_drop_pose(label)
        log.info('放物流程开始：%s -> %s 放下',
                 label,('区域B（最右 +%.2f m）'%AREA_B_OFFSET_M)
                       if str(label or '').lower()==TISSUE_CLASS
                       else ('区域A（最左水瓶 -%.2f m）'%AREA_A_OFFSET_M))
        self.go_pose(PICK_X,SAFE_Y,'zone safe')
        self.go_pose(PICK_X,PICK_Y,'zone down')
        self.grip(True)
        self.go_pose(PICK_X,SAFE_Y,'zone up')
        log.info('arm recenter after release（避免碰倒刚放下的物体）')
        self.go_pose(HOME_X,HOME_Y,'post-place recenter')
        self.return_to_row_left()
        self.return_to_scan_depth()
        self.go_pose(SCAN_X,SCAN_Y,'return scan pose')
    def move_lateral_straight(self,d,label='横移'):
        if abs(d)<0.01: return 0.0
        ref=self.get_pos()
        if ref is None:
            log.warning('★ 没有位置反馈 -> %s 退回整段横移 %.2f m',label,d)
            self.move_y(d); return 0.0
        key='right' if d>0 else 'left'
        if not hasattr(self,'_lat_comp'): self._lat_comp={'right':0.0,'left':0.0}
        c=self._lat_comp.get(key,0.0)
        sgn=1.0 if d>0 else -1.0
        step=min(LAT_STEP_M,abs(d)); n=int(math.ceil(abs(d)/step-1e-9))
        log.info('★ %s：%s %.3f m，分 %d 段（每段 %.2fm，速度 %.2fm/s，'
                 '前馈补偿初值 %+.3f）',
                 label,'往右' if d>0 else '往左',abs(d),n,step,LAT_SPEED,c)
        done=0.0; worst=0.0
        for i in range(n):
            s=min(step,abs(d)-done)*sgn
            _bx,_by=self.world_to_body(0.0,s)
            _pb=self.get_pos()
            try:
                self.wait_feed(self.chassis.move(x=_bx-c, y=_by, z=0,
                                                 xy_speed=LAT_SPEED,
                                                 z_speed=HEADING_Z_SPEED),
                               LAT_STEP_TIMEOUT,'%s %d/%d'%(label,i+1,n))
            except Exception as e:
                log.warning('  %s 第 %d 段失败(%s) -> 停止横移',label,i+1,str(e)[:80]); break
            self.stop_drive()
            _pa=self.get_pos()
            if _pa is not None and _pb is not None:
                dbx=self.body_forward(_pa,ref)-self.body_forward(_pb,ref)
                if abs(dbx)>0.004:
                    c=max(-LAT_COMP_MAX,min(LAT_COMP_MAX,c+1.0*dbx))
                    self._lat_comp[key]=c
                    log.info('  第 %d/%d 段：前后漂 %+.3f m -> 下一段预补偿 %+.3f',
                             i+1,n,dbx,c)
            _ya=self.yaw_anchor(); _yy=self.get_yaw()
            _yerr=abs(_yy-_ya) if (_ya is not None and _yy is not None) else None
            if _yerr is not None and _yerr>25.0:
                log.error('★★ %s：航向已偏 %.1f°（>25°）-> 中止本次横移，'
                          '原地不动，请检查是否被卡住/打滑',label,_yerr)
                self.stop_drive(); break
            dx=self.body_forward(self.get_pos(),ref)
            worst=max(worst,abs(dx))
            done+=abs(s)
        self.stop_drive()
        self.fix_heading(self.yaw_anchor(),' '+label+' 结束')
        _extra=0
        while _extra<40:
            _p=self.get_pos()
            if _p is None: break
            _rem=d-self.body_lateral(_p,ref)
            if abs(_rem)<=0.02: break
            _extra+=1
            _s=max(-LAT_STEP_M,min(LAT_STEP_M,_rem))
            _bx,_by=self.world_to_body(0.0,_s)
            log.info('  补段 %d：实测横向还差 %+.3f m -> 再走 %.3f m',_extra,_rem,_s)
            try:
                self.wait_feed(self.chassis.move(x=_bx,y=_by,z=0,
                                                 xy_speed=LAT_SPEED,
                                                 z_speed=HEADING_Z_SPEED),
                               LAT_STEP_TIMEOUT,'%s 补段%d'%(label,_extra))
            except Exception as e:
                log.warning('  补段失败(%s) -> 停止补段',str(e)[:80]); break
            self.stop_drive()
            self.fix_heading(self.yaw_anchor(),' '+label)
        if _extra:
            _p=self.get_pos()
            log.info('  补段结束（共 %d 段）：实测横向 %.3f（目标 %.3f，残差 %+.3f）',
                     _extra,self.body_lateral(_p,ref),d,self.body_lateral(_p,ref)-d)
        fin=self.body_forward(self.get_pos(),ref)
        if abs(fin)>=0.04:
            log.info('★ %s 结束回正：前后残差 %+.3f m -> 位置回正一次',label,fin)
            try:
                self.wait_feed(self.chassis.move(x=-fin,y=0,z=0,xy_speed=.5,
                                                 z_speed=HEADING_Z_SPEED),
                               8,'%s 结束回正'%label)
                self.stop_drive()
                fin=self.body_forward(self.get_pos(),ref)
            except Exception as e:
                log.warning('  结束回正失败(%s)',str(e)[:80])
        log.info('★ %s 完成：横移 %.2f m，前后残差 %+.3f m（过程中最大 %+.3f m）',
                 label,abs(d),fin,worst)
        return fin
    def return_to_row_left(self):
        cur=self.get_pos(); ref=getattr(self,'_ref_pose',None)
        L=getattr(self,'_last_pick_lat',None)
        if L is not None:
            L=L+ROW_SPACING_M
        else:
            L=getattr(self,'_pick_y1',None)
        if cur is None or ref is None or L is None:
            log.warning('★ 位置反馈不全 -> 放完退回固定右移 %.2f m',PLACE_BACKOFF_M)
            self.move_y(RIGHT_SIGN*PLACE_BACKOFF_M); return
        bl=self.body_lateral(cur,ref)
        d=L-bl
        log.info('★ 放完回位：当前横向 %.3f -> 下一个待夹取位置 %.3f（%.3f m，%s）',
                 bl,L,abs(d),'往右' if d>0 else '往左')
        if abs(d)<=0.02:
            log.info('  已在最左端线上，无需回位'); return
        self.move_lateral_straight(d,'放完回位')
    def stop(self):
        try:self.stop_drive()
        except Exception:pass
        try:
            if getattr(self,'_tsock',None) is not None:
                self._tsock.close(); self._tsock=None
        except Exception:pass
        if self.sensor:
            try:self.sensor.unsub_distance()
            except Exception:pass
        try:self.chassis.unsub_attitude()
        except Exception:pass
        try:self.chassis.unsub_position()
        except Exception:pass
        try:self.arm.unsub_position()
        except Exception:pass
        if self.gripper:
            try:self.gripper.pause()
            except Exception:pass
        if self.camera:
            self._grab_stop=True
            time.sleep(0.3)
            try:
                with self._cam_io_lock:
                    self.camera.stop_video_stream()
            except Exception:pass
        if self.connected:
            try:self.ep.close()
            except Exception:pass
        cv2.destroyAllWindows()
def main():
    s=Sorter()
    try:
        log.info('='*72)
        log.info('本轮参数：停车阈值=%.1fcm  视觉停车=框宽%dpx  落点左移=%.2fm  盲走=%.0fmm  '
                 '粗步=%.0fmm  细步=%.0fmm(%.0fcm内)  夹取前居中容差=±%.0f%%框宽  置信度阈值=%.2f',
                 APPROACH_STOP_CM,VISION_STOP_PX,DROP_LEFT_M,FINAL_CREEP_M*1000,FWD_STEP_M*1000,
                 FWD_STEP_FINE_M*1000,FWD_FINE_CM,GRASP_CENTER_FRAC*100,CONF)
        log.info('★ 判据：日志里要出现 "测距 xx.xcm <= %.1fcm -> 已到位" 且 '
                 '"前进阶段结束…停止原因=tof"，停车距离才算受控。',APPROACH_STOP_CM)
        log.info('★ 摄像头自愈：%s（连续 %d 帧相同或取帧超时 -> 自动重启视频流，最多 %d 次/轮）',
                 '开' if CAMERA_AUTO_RESTART else '关',CAMERA_FREEZE_FRAMES,CAMERA_MAX_RESTARTS)
        log.info('★ 日志文件：%s',LOG_DIR / ('run_%s.log' % RUN_TS))
        log.info('★ 埋点CSV ：%s',CSV_PATH)
        log.info('='*72)
        csv_row('start',note='stop=%.1fcm vision=%dpx creep=%.0fmm fine=%.0fmm/%.0fcm conf=%.2f'
                %(APPROACH_STOP_CM,VISION_STOP_PX,FINAL_CREEP_M*1000,
                  FWD_STEP_FINE_M*1000,FWD_FINE_CM,CONF))
        s.connect()
        log.info('initial arm recenter')
        s.home_arm()
        if getattr(s,'_arm_stuck',False) and ARM_STUCK_ABORT:
            log.error('='*72)
            log.error('★★★ 机械臂回不了零 -> 本轮中止，不往下跑。')
            log.error('★★★ 手臂停在镜头前，所有视觉结论都不可信，跑了也是白跑。')
            log.error('★★★ 软件层已经试过三招了（①先抬后收 ②直接到家 ③相对 move），'
                      '全部无效 -> 这不是代码能解决的了。')
            log.error('★★★ 现在只剩两条路：')
            log.error('★★★   ① 给机器人**断电重启**（真关机再开机）—— 堵转保护/零位'
                      '在机器人固件里，只有断电才复位；重跑程序没用。')
            log.error('★★★   ② 若重启后仍卡在 x≈179：先**目视**手臂是伸着还是缩着、'
                      '用手轻推有没有阻力 -> 有阻力就是机械干涉/舵机问题，要查线缆、'
                      '云台或送修。')
            log.error('★★★ 想强行跑（视觉几何是错的，不推荐）把 ARM_STUCK_ABORT 改 False。')
            log.error('★★★ （确实想强行跑就把 ARM_STUCK_ABORT 改成 False）')
            log.error('='*72)
            csv_row('abort',note='机械臂回不了零 -> 本轮中止')
            return
        s.grip(True)
        log.info('回零完成 -> 摆到识别姿态 (x=%d, y=%d)',SCAN_X,SCAN_Y)
        s.go_pose(SCAN_X,SCAN_Y,'scan pose')
        s._yaw0=s.get_yaw()
        log.info('★ 航向基准锁定 = %s（全程所有动作都以它为准）',
                 ('%.1f°'%s._yaw0) if s._yaw0 is not None else '无IMU')
        round_no=0
        while s.scanned<MAX_SCAN:
            round_no+=1
            _p=s.get_pos()
            log.info('='*72)
            log.info('★★ 第 %d 个瓶子：开始（当前位姿 %s）',round_no,
                     ('(x=%.3f, y=%.3f)'%_p) if _p else '无')
            log.info('='*72)
            t=s.scan()
            if t is None:
                log.info('第 %d 轮没找到目标 -> 本次任务结束（共处理 %d 个）',
                         round_no,round_no-1); break
            if not s.pick():
                log.info('校准模式：已停在抓取点上方，本次结束（未抓取、未放置）'); break
            s.place_return(t.label)
            log.info('★★ 第 %d 个瓶子已完成（已放到同一个落点）',round_no)
    except KeyboardInterrupt: log.warning('stopped by user')
    finally:
        try: csv_row('run_end',note='本轮结束')
        except Exception: pass
        s.stop()
if __name__=='__main__': main()
