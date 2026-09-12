"""RoboMaster EP + 双 YOLO：六格定点夹取，单文件实机版。

使用：在原来可运行 RoboMaster SDK / YOLO 的 Python 环境里打开本文件，
确认权重路径和下面的场地初值，直接点击 IDE 的运行按钮。没有命令行参数或运行模式。
两个 .pt 权重仍是独立的模型数据文件，不需要其他 Python 文件或配置文件。

任务：逐格平移 -> 识别居中 -> 向前接近 -> 夹取/抬升/收臂 -> 退回基准线
      -> 平移至对应料箱 -> 前进放置 -> 退回基准线 -> 下一格，共六格。

启动条件：车在平移基准线上、车头正对物体排、夹爪空载，投放点未占用。
停止：预览窗口按 Q / ESC，或终端 Ctrl+C。退出不会自动松开手中物体。
机械臂 Action 等待期间窗口按键可能延后处理；SDK 断连和原生崩溃需现场停止。

参数已按原脚本补回，没有要求填写的None。注释区分原值、换算值与估算初值。
未采用示意图尺寸。向料箱前进默认0m，保持原流程；不表示实际箱壁距离已测量。
新增箱内落点按物宽估算：左箱至少覆盖26cm、右箱至少覆盖36.4cm的横向范围，
还需留定位误差余量；箱底高度、可用范围和伸臂路径应现场核对。
持物区域默认从抓取前后新出现的同类检测框自动确定，不能定位时停止，不猜成功。
持物验证用夹爪状态与相机证据，不能保证力学夹持或物体在箱底站稳。
轮式里程计/IMU不能消除打滑造成的绝对位置漂移，长距离误差需实测核对。
"""

import copy
from dataclasses import asdict, dataclass
import faulthandler
import hashlib
import json
import logging
import math
from pathlib import Path
import threading
import time
from typing import List, Tuple


# ======================== 参数区：通常只修改这里 ========================
SCRIPT_DIR = Path(__file__).resolve().parent
SHOW_PREVIEW = True
LOG_DIR = SCRIPT_DIR / 'logs'  # 自动生成运行日志，不需要事先准备文件

CONFIG = {
    'connection': {'type': 'ap'},  # 沿用原代码的 AP 连接方式

    'layout': {
        'slot_count': 6,
        'slot_spacing_m': 0.60,  # 六格中心：0、0.6、1.2、1.8、2.4、3.0m
        'start_y_m': 0.0,        # 默认开始时正对第1格，位于 x=0 平移线上
        # 场地前向在SDK固定里程计中的方向角 atan2(dy,dx)。
        # 原代码USE_FRAME_CONVERT=False：此处以轴对齐的0°为初值；必须上电摆正。
        'lane_angle_in_odom_deg': 0.0,
        # 原代码符号在运行中自学，并无本轮最终读数；以下是方向初值，不是实测结论。
        'yaw_feedback_to_clockwise': 1,  # 原始yaw增大为顺时针填+1，否则-1
        'sdk_z_to_clockwise': 1,         # 正z指令为顺时针填+1，否则-1
    },

    'vision': {
        # 相对路径以本.py所在目录为准；也可以填写本机的绝对路径。
        # 两个模型启动时同时加载，每张图依次送给两个模型，再合并检测结果。
        'models': [
            {'path': 'yolov8m.pt', 'labels': {'bottle': 'bottle'}},
            {'path': 'tissue.pt', 'labels': {'tissue': 'tissue'}},
        ],
        # 若第一个模型输出叫cup，修改上面的映射为 {'cup': 'bottle'}。
        # 左侧是权重里的真实类别名，右侧是程序内部类别名；启动会核对。
        'device': 'cuda:0',     # GPU预热失败在连接机器人前尝试CPU
        'imgsz': 1280,          # 原代码coco_m的推理尺寸
        'resolution': '540p',   # 恢复原代码实际使用的视频分辨率
        'reference_width_px': 960,  # 仅供启动校验预计算；连接后按真实帧宽重新换算
        # ROI格式：[左/图宽，上/图高，右/图宽，下/图高]，每项0..1。
        # scan_roi须排除邻格与背景误检，并在整个接近过程覆盖当前目标。
        'scan_roi': [0.30, 0.05, 0.70, 0.95],  # 待按本机画面复核
        # auto：抬升收臂后，寻找抓取前不存在/明显改变且多帧稳定的同类框，
        # 自动建立本次持物ROI；携带物不可见或候选不唯一时停车，不用“消失=成功”。
        # 如已测好固定ROI，也可填写[左,上,右,下]。相机随臂移动时需特别核对。
        'held_roi': 'auto',
        'image_right_to_lane_y': 1,  # 图中目标偏右时车应右移填+1，镜像时复核
        'observation_max_age_s': 1.0,
        'scan_timeout_s': 3.0,
        'confirm_frames': 3,
        'confirm_duration_s': 0.25,
        'association_iou': 0.4,
    },

    'classes': {
        'bottle': {
            'confidence': 0.75,
            'bin': 'left',
            'width_m': 0.065,           # 原OBJ_WIDTH_CM=6.5cm
            'grasp_width_px': 300,      # 原VISION_STOP_PX；按实际帧宽自动转换为比例
            'aim_x_ratio': 0.5,         # 原画面中线 + GRASP_XE_OFFSET_PX=0
            'min_pick_x_m': 0.0,
            'max_pick_x_m': 2.50,       # 原MAX_FORWARD_M，仅为上限，不是固定前进2.5m
            'center_fraction': 0.05,    # 原GRASP_CENTER_FRAC：终点框宽的5%
            'width_tolerance_ratio': 0.06,
        },
        'tissue': {
            'confidence': 0.60,
            'bin': 'right',
            'width_m': 0.091,           # 原OBJ_WIDTH_CM=9.1cm
            'grasp_width_px': 420,      # 原stop_px():300*9.1/6.5
            'aim_x_ratio': 0.5,
            'min_pick_x_m': 0.0,
            'max_pick_x_m': 2.50,
            'center_fraction': 0.05,
            'width_tolerance_ratio': 0.06,
        },
    },

    'bins': {
        # 位置均以同一底盘参考点测量，例如底盘中心。
        # 每箱center_y + 各offset 是底盘横向投放停车点；前向再走approach_x。
        # 左侧全部落点应在第1格左边；右侧全部落点应在第6格右边。
        'left': {
            'center_y_m': -0.60,        # 原L-AREA_A_OFFSET_M；新固定第1格L=0
            'approach_x_m': 0.0,        # 原版在基准线上伸臂放置；真实箱位需要前进再改
            'release_pose_mm': [200, 0],  # 原place_return复用PICK_X/PICK_Y
            'drop_offsets_y_m': [0.0, -0.0975, 0.0975],  # 估算：相邻间距=1.5*瓶宽
            'min_drop_spacing_m': 0.085,              # 估算：瓶宽+2cm
        },
        'right': {
            'center_y_m': 3.60,         # 原L+(6-1)*0.6+0.6；固定L=0
            'approach_x_m': 0.0,
            'release_pose_mm': [200, 0],
            'drop_offsets_y_m': [0.0, -0.1365, 0.1365],  # 估算：相邻间距=1.5*纸巾宽
            'min_drop_spacing_m': 0.111,               # 估算：纸巾宽+2cm
        },
        # 若不是每类3个，可按实际容量增加偏移；箱满会在夹取前跳过，不强行放置。
    },

    'arm': {
        # 以下姿态/范围来自原脚本历史值，不是本次实机标定。单位均为mm。
        # 每次变更伸出量都先抬升，再高处伸缩，最后下降，须验证整段路径无碰撞。
        'x_min_mm': 70, 'x_max_mm': 205,
        'y_min_mm': 0, 'y_max_mm': 120,
        'low_pose_min_x_mm': 150, 'inner_min_y_mm': 30,
        'travel_y_mm': 90,
        'scan': [120, 30],
        'carry': [70, 90],
        'pick': [200, 0],
        'tolerance_mm': 8,
        'timeout_s': 15,
        'open_power': 40, 'close_power': 45, 'grip_seconds': 3.0,
        # normal为未完全闭合；不是受力证明，还必须通过相机持物验证。
        'holding_statuses': ['normal'],
    },

    'tof': {
        'enabled': True,
        'index': 0,                # 唯一选定的前向传感器通道0..3，不盲取四路最小值
        'min_clearance_m': 0.070,  # 沿用原7cm阈值；本版触发后停机，不据此单独判夹取成功
        'max_valid_m': 3.0,
        'max_age_s': 0.5,
        # 启用时测距无效/过期则停机。无可用传感器才设False，不会获得防撞能力。
        # 料箱接近与基准线运输依靠实测空闲路径，本代码没有全向避障。
    },

    'motion': {
        # 底盘单位m、m/s；角度deg。先用这些低速初值，不自动放大脉冲。
        'hz': 20,
        'telemetry_max_age_s': 0.5,
        'pose_tolerance_m': 0.003,  # 接收阈值，不等于承诺实机能达到3mm绝对精度
        'heading_tolerance_deg': 1.0,
        'max_heading_error_deg': 8.0,
        'max_cross_track_m': 0.035,
        'kp_position': 1.7, 'kp_heading': 2.0,
        'max_speed_mps': 0.18, 'approach_speed_mps': 0.045,
        'max_correction_mps': 0.025,
        'max_yaw_dps': 12,
        'max_accel_mps2': 0.25,
        'segment_timeout_s': 12, 'stall_timeout_s': 3.0,
        'settle_s': 0.25, 'velocity_lease_s': 0.3,
        'approach_step_m': 0.03, 'fine_step_m': 0.012,
        'approach_timeout_s': 120,
        'max_alignment_offset_m': 0.12,
        'alignment_step_limit_m': 0.03,
        'max_alignment_steps': 8, 'max_approach_attempts': 2,
        'workspace_margin_m': 0.15,
    },
}
# ====================== 参数区结束：下方为控制逻辑 ======================

log = logging.getLogger(__name__)


# ======================== 任务数据与异常 ========================

class SafetyFault(RuntimeError):
    """位置、动作或抓取证据不可靠：停机，禁止自动恢复运动。"""


class SkipSlot(RuntimeError):
    """尚未闭爪时的识别/可达性失败：有有效定位时可沿原线退出。"""


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: Tuple[float, float, float, float]  # 原图坐标分别除以宽/高

    @property
    def cx(self):
        return (self.box[0] + self.box[2]) / 2

    @property
    def cy(self):
        return (self.box[1] + self.box[3]) / 2

    @property
    def width(self):
        return self.box[2] - self.box[0]

    def inside(self, roi):
        return roi[0] <= self.cx <= roi[2] and roi[1] <= self.cy <= roi[3]


@dataclass(frozen=True)
class Observation:
    seq: int
    stamp: float
    detections: List[Detection]


def wrap(angle):
    return (angle + 180) % 360 - 180


def clamp(value, limit):
    return max(-limit, min(limit, value))


def iou(a, b):
    ax, ay, bx, by = a.box
    cx, cy, dx, dy = b.box
    overlap = max(0, min(bx, dx)-max(ax, cx)) * max(0, min(by, dy)-max(ay, cy))
    union = (bx-ax)*(by-ay) + (dx-cx)*(dy-cy) - overlap
    return overlap / union if union > 0 else 0


# ======================== 实机参数检查 ========================

class ConfigError(ValueError):
    pass


def number(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f"{name}: 请填写有限数值，不能留 None")
    if low is not None and value < low or high is not None and value > high:
        raise ConfigError(f"{name}: {value} 超出 [{low}, {high}]")
    return value


def roi(value, name):
    if not isinstance(value, list) or len(value) != 4:
        raise ConfigError(f"{name}: 应为 [左,上,右,下]，坐标除以图像宽高")
    for v in value:
        number(v, name, 0, 1)
    if value[0] >= value[2] or value[1] >= value[3]:
        raise ConfigError(f"{name}: ROI 面积必须大于零")


def arm_target(c, target):
    a = c['arm']
    if not isinstance(target, (list, tuple)) or len(target) != 2:
        raise ConfigError('机械臂姿态必须是 [x_mm, y_mm]')
    x, y = target
    number(x, 'arm.x', a['x_min_mm'], a['x_max_mm'])
    number(y, 'arm.y', a['y_min_mm'], a['y_max_mm'])
    if x < a['low_pose_min_x_mm'] and y < a['inner_min_y_mm']:
        raise ConfigError(f'机械臂内收低位不可达: {target}')


def validate(c):
    try:
        _validate(c)
    except (KeyError, TypeError) as e:
        raise ConfigError(f'配置字段缺失或类型错误: {e}') from e


def _validate(c):
    l, m, v, a, t = (c[k] for k in ('layout', 'motion', 'vision', 'arm', 'tof'))
    if l['slot_count'] != 6 or l['slot_spacing_m'] != 0.6:
        raise ConfigError('本任务固定六格、中心间距 0.60 m')
    number(l['start_y_m'], 'layout.start_y_m')
    number(l['lane_angle_in_odom_deg'], 'layout.lane_angle_in_odom_deg', -180, 180)
    for key in ('yaw_feedback_to_clockwise', 'sdk_z_to_clockwise'):
        if l[key] not in (-1, 1):
            raise ConfigError(f'{key} 必须是 +1 或 -1')
    positive = ('hz', 'telemetry_max_age_s', 'pose_tolerance_m', 'heading_tolerance_deg',
                'max_heading_error_deg', 'max_cross_track_m', 'kp_position', 'kp_heading',
                'max_speed_mps', 'approach_speed_mps', 'max_correction_mps', 'max_yaw_dps',
                'max_accel_mps2', 'segment_timeout_s', 'stall_timeout_s', 'settle_s',
                'velocity_lease_s', 'approach_step_m', 'fine_step_m', 'approach_timeout_s',
                'max_alignment_offset_m', 'alignment_step_limit_m', 'workspace_margin_m')
    for key in positive:
        number(m[key], 'motion.' + key, 0.0001)
    if m['velocity_lease_s'] <= 2 / m['hz']:
        raise ConfigError('velocity_lease_s 必须大于两个控制周期')
    if m['max_speed_mps'] > 0.4 or m['approach_speed_mps'] > 0.10:
        raise ConfigError('底盘限速：平移 <=0.4 m/s，接近 <=0.10 m/s')
    if not 0 < m['pose_tolerance_m'] < m['fine_step_m'] <= m['approach_step_m'] <= 0.05:
        raise ConfigError('要求 0 < 到位容差 < 细步 <= 粗步 <= 0.05 m')
    if m['max_alignment_offset_m'] >= l['slot_spacing_m'] / 2:
        raise ConfigError('视觉调整范围必须小于半个格子间距')
    if m['max_correction_mps'] > m['max_speed_mps']:
        raise ConfigError('纠偏速度不能超过行驶速度')
    for key in ('max_approach_attempts', 'max_alignment_steps'):
        if not isinstance(m[key], int) or not 1 <= m[key] <= 30:
            raise ConfigError(f'{key} 必须为 1..30 的整数')
    for key in ('observation_max_age_s', 'scan_timeout_s', 'confirm_duration_s', 'association_iou'):
        number(v[key], 'vision.' + key, 0.001)
    if not isinstance(v['confirm_frames'], int) or not 2 <= v['confirm_frames'] <= 30:
        raise ConfigError('confirm_frames 必须为 2..30 整数')
    if v['association_iou'] > 1:
        raise ConfigError('association_iou 不能大于 1')
    if v['image_right_to_lane_y'] not in (-1, 1):
        raise ConfigError('image_right_to_lane_y 必须是 +1 或 -1')
    roi(v['scan_roi'], 'vision.scan_roi')
    if v['held_roi'] != 'auto':
        roi(v['held_roi'], 'vision.held_roi')
    if v['resolution'] not in ('360p', '540p', '720p'):
        raise ConfigError('请使用 SDK 支持的视频分辨率')
    if not v['models'] or len(c['classes']) < 2 or len(c['bins']) != 2:
        raise ConfigError('需要模型、至少两类物体、两个料箱')
    mapped = set()
    for model in v['models']:
        if not model['path'] or not model['labels']:
            raise ConfigError('模型路径及类别映射不能为空')
        mapped.update(model['labels'].values())
    if mapped != set(c['classes']):
        raise ConfigError('模型 labels 的目标类别必须完整对应 classes')
    for name, p in c['classes'].items():
        if p['bin'] not in c['bins']:
            raise ConfigError(f'{name} 的目标料箱不存在')
        for key in ('confidence', 'grasp_width_ratio', 'aim_x_ratio'):
            number(p[key], name + '.' + key, 0.001, 0.999)
        number(p['width_m'], name + '.width_m', 0.001, 0.5)
        number(p['center_tolerance_ratio'], name + '.center_tolerance_ratio', 0.001, 0.2)
        number(p['width_tolerance_ratio'], name + '.width_tolerance_ratio', 0.001, 0.5)
        number(p['min_pick_x_m'], name + '.min_pick_x_m', 0)
        number(p['max_pick_x_m'], name + '.max_pick_x_m', 0.001, 2.5)
        if p['min_pick_x_m'] >= p['max_pick_x_m']:
            raise ConfigError('最小夹取前进距离必须小于最大距离')
    for key in ('x_min_mm', 'x_max_mm', 'y_min_mm', 'y_max_mm', 'low_pose_min_x_mm',
                'inner_min_y_mm', 'travel_y_mm', 'tolerance_mm', 'timeout_s', 'grip_seconds'):
        number(a[key], 'arm.' + key)
    if not 0 < a['tolerance_mm'] <= 20 or a['timeout_s'] <= 0 or a['grip_seconds'] <= 0:
        raise ConfigError('机械臂容差/超时/夹爪时间无效')
    if a['travel_y_mm'] < a['inner_min_y_mm']:
        raise ConfigError('机械臂横向伸缩必须在已标定的抬升高度')
    for key in ('scan', 'carry', 'pick'):
        arm_target(c, a[key])
    arm_target(c, [a['x_min_mm'], a['travel_y_mm']])
    arm_target(c, [a['x_max_mm'], a['travel_y_mm']])
    if a['carry'][1] < a['travel_y_mm']:
        raise ConfigError('运输姿态必须达到抬升高度')
    for key in ('open_power', 'close_power'):
        number(a[key], key, 1, 100)
    if not a['holding_statuses'] or not set(a['holding_statuses']) <= {'normal', 'closed'}:
        raise ConfigError('holding_statuses 只能是标定后的 normal/closed')
    all_y = [0, 3, l['start_y_m']]
    for name, b in c['bins'].items():
        number(b['center_y_m'], name + '.center_y_m')
        number(b['approach_x_m'], name + '.approach_x_m', 0, 2)
        number(b['min_drop_spacing_m'], name + '.min_drop_spacing_m', 0.001)
        arm_target(c, b['release_pose_mm'])
        if not b['drop_offsets_y_m']:
            raise ConfigError('每个料箱至少配置一个可达投放点')
        ys = []
        for offset in b['drop_offsets_y_m']:
            number(offset, name + '.drop_offsets_y_m')
            y = b['center_y_m'] + offset
            if not (y < -m['max_alignment_offset_m'] or y > 3 + m['max_alignment_offset_m']):
                raise ConfigError('料箱投放点必须在六格区域两侧')
            ys.append(y)
        if any(abs(x-y) < b['min_drop_spacing_m'] for i,x in enumerate(ys) for y in ys[i+1:]):
            raise ConfigError('同一料箱投放点重复或间距太小')
        all_y.extend(ys)
    centers = sorted(b['center_y_m'] for b in c['bins'].values())
    if not centers[0] < 0 < 3 < centers[1]:
        raise ConfigError('两个料箱应分居六格左右两侧')
    if t['enabled']:
        if t['index'] not in range(4):
            raise ConfigError('tof.index 是唯一选定的前向传感器通道 0..3')
        number(t['min_clearance_m'], 'tof.min_clearance_m', 0.001)
        number(t['max_valid_m'], 'tof.max_valid_m', t['min_clearance_m'])
        number(t['max_age_s'], 'tof.max_age_s', 0.001)
    c['_bounds'] = (min(all_y)-m['workspace_margin_m'], max(all_y)+m['workspace_margin_m'],
                    max([p['max_pick_x_m'] for p in c['classes'].values()] +
                        [b['approach_x_m'] for b in c['bins'].values()]) + m['workspace_margin_m'])


def prepare_visual_geometry(c, frame_width):
    """保持原300/420像素阈值；只转换表示方式，不拿YOLO输入尺寸当图像宽度。"""
    number(frame_width, '原始相机帧宽', 1)
    for label, profile in c['classes'].items():
        if 'grasp_width_px' in profile:
            pixels = number(profile['grasp_width_px'], label + '.grasp_width_px', 1, frame_width-1)
            fraction = number(profile['center_fraction'], label + '.center_fraction', 0.001, 0.5)
            profile['grasp_width_ratio'] = pixels / frame_width
            profile['center_tolerance_ratio'] = pixels / frame_width * fraction


# ======================== 底盘直线闭环 ========================

class Navigator:
    def __init__(self, io, config, emit):
        self.io, self.c, self.emit = io, config, emit
        self.m = config['motion']

    def pose(self):
        p = self.io.pose()
        if not all(math.isfinite(v) for v in (p.x, p.y, p.yaw, p.stamp)):
            raise SafetyFault('里程计包含非有限数值')
        if not 0 <= self.io.now() - p.stamp <= self.m['telemetry_max_age_s']:
            raise SafetyFault('位置或航向反馈过期')
        lo, hi, front = self.c['_bounds']
        if not (-self.m['workspace_margin_m'] <= p.x <= front and lo <= p.y <= hi):
            raise SafetyFault(f'机器人超出已配置工作区: {p}')
        if abs(p.yaw) > self.m['max_heading_error_deg']:
            raise SafetyFault(f'航向偏差过大 {p.yaw:.1f} deg；检查坐标方向和车轮')
        return p

    def move_y(self, y):
        p = self.pose()
        if abs(p.x) > self.m['pose_tolerance_m']:
            raise SafetyFault(f'未退回平移线，拒绝横移: x={p.x:.3f}')
        self.io.require_transport_pose()
        self._move(0.0, y, 'y', self.m['max_speed_mps'])

    def move_x(self, x, guard=False):
        p = self.pose()
        self._move(x, p.y, 'x', self.m['approach_speed_mps'] if guard else self.m['max_speed_mps'], guard)

    def _move(self, x, y, axis, speed, guard=False):
        m, io = self.m, self.io
        lo, hi, front = self.c['_bounds']
        if not all(math.isfinite(v) for v in (x, y)) or not (0 <= x <= front and lo <= y <= hi):
            raise SafetyFault('运动目标超出工作区')
        start = self.pose()
        if axis == 'y' and abs(x) > m['pose_tolerance_m']:
            raise SafetyFault('横移目标必须在 x=0 基准线上')
        io.stop()
        self.emit('move_start', axis=axis, x=x, y=y)
        beginning = last_progress = io.now()
        distance = math.hypot(x-start.x, y-start.y)
        deadline = beginning + max(m['segment_timeout_s'], distance/max(speed*0.35, 0.005)+3)
        progress_error = float('inf')
        stable_since = None
        last_vx = last_vy = 0.0
        previous_time = io.now()
        last_log = previous_time-1
        try:
            while True:
                io.check_abort()
                p = self.pose()
                now = io.now()
                if now-last_log >= 0.25:
                    self.emit('motion_sample', x=p.x,y=p.y,yaw=p.yaw,target_x=x,target_y=y,axis=axis)
                    last_log = now
                ex, ey, eh = x-p.x, y-p.y, -wrap(p.yaw)
                if guard:
                    io.check_front_clearance()
                cross = ex if axis == 'y' else ey
                if abs(cross) > m['max_cross_track_m']:
                    raise SafetyFault(f'偏离当前直线 {cross:.3f} m')
                at_target = (abs(ex) <= m['pose_tolerance_m'] and abs(ey) <= m['pose_tolerance_m']
                             and abs(eh) <= m['heading_tolerance_deg'])
                if at_target:
                    io.stop()
                    last_vx = last_vy = 0.0
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= m['settle_s']:
                        self.emit('move_done', axis=axis, x=p.x, y=p.y, yaw=p.yaw)
                        return
                else:
                    stable_since = None
                    vx = clamp(m['kp_position']*ex, speed if axis == 'x' else m['max_correction_mps'])
                    vy = clamp(m['kp_position']*ey, speed if axis == 'y' else m['max_correction_mps'])
                    # 仅作为轨迹误差纠正使用第二轴；不安排斜穿目标点。
                    dt = max(1/m['hz'], min(0.15, now-previous_time))
                    dv = m['max_accel_mps2']*dt
                    vx, vy = last_vx+clamp(vx-last_vx, dv), last_vy+clamp(vy-last_vy, dv)
                    # 速度不会因最小脉冲时间被强制放大；到位后立即发零速。
                    theta = math.radians(p.yaw)
                    bx = math.cos(theta)*vx + math.sin(theta)*vy
                    by = -math.sin(theta)*vx + math.cos(theta)*vy
                    io.velocity(bx, by, clamp(m['kp_heading']*eh, m['max_yaw_dps']))
                    last_vx, last_vy = vx, vy
                error = math.hypot(ex, ey) + abs(eh)*0.001
                if error < progress_error - 0.002:
                    progress_error, last_progress = error, now
                if not at_target and now-last_progress > m['stall_timeout_s']:
                    raise SafetyFault('底盘未收敛/堵转：不放大脉冲、不忽略超时')
                if now > deadline:
                    raise SafetyFault('底盘运动超时')
                previous_time = now
                io.sleep(1/m['hz'])
        finally:
            io.stop()


# ======================== 双模型与实机接口 ========================

class Samples:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {}

    def put(self, key, value):
        with self.lock:
            self.data[key] = (value, time.monotonic())

    def get(self, key, max_age):
        with self.lock:
            item = self.data.get(key)
        if item is None or time.monotonic()-item[1] > max_age:
            raise SafetyFault(f'{key} 遥测缺失或过期')
        return item


class YoloDetector:
    def __init__(self, c):
        from ultralytics import YOLO
        import numpy as np
        self.c = c
        self.models = []
        for spec in c['vision']['models']:
            if not Path(spec['path']).is_file():
                raise ConfigError(f"权重不存在: {spec['path']}；不会自动下载或换模型")
            model = YOLO(spec['path'])
            names = {str(n).lower() for n in model.names.values()}
            missing = set(spec['labels'])-names
            if missing:
                raise ConfigError(f"{spec['path']} 缺少类别 {sorted(missing)}，实际为 {model.names}")
            self.models.append((model, spec['labels']))
        self.device = c['vision']['device']
        # GPU 是否实际可执行由完整预热检验；不依据显卡字符串猜测。
        blank = np.zeros((720, 1280, 3), np.uint8)
        try:
            self.detect(blank)
        except Exception:
            if self.device == 'cpu':
                raise
            log.exception('GPU 预热失败；连接机器人之前尝试 CPU')
            self.device = 'cpu'
            self.detect(blank)
        log.info('模型预热完成，device=%s', self.device)

    def detect(self, frame):
        out = []
        height, width = frame.shape[:2]
        for model, mapping in self.models:
            floor = min(0.25, min(p['confidence'] for p in self.c['classes'].values()))
            r = model.predict(frame, imgsz=self.c['vision']['imgsz'], conf=floor,
                              device=self.device, max_det=30, verbose=False)[0]
            if r.boxes is None:
                continue
            for raw_box, cls, conf in zip(r.boxes.xyxy.cpu().tolist(),
                                          r.boxes.cls.cpu().tolist(), r.boxes.conf.cpu().tolist()):
                raw_name = str(r.names[int(cls)]).lower()
                label = mapping.get(raw_name, 'unknown:' + raw_name)
                x1, y1, x2, y2 = raw_box
                box = (x1/width, y1/height, x2/width, y2/height)
                if x2 <= x1 or y2 <= y1 or not all(math.isfinite(v) for v in (*box, conf)):
                    continue
                out.append(Detection(label, float(conf), box))
        # 同类跨模型重复框去重；不同类重叠框保留，交给状态机拒绝歧义。
        kept = []
        for d in sorted(out, key=lambda d: d.confidence, reverse=True):
            if not any(d.label == k.label and iou(d, k) > 0.6 for k in kept):
                kept.append(d)
        return kept


class CameraReader:
    def __init__(self, camera, samples):
        self.camera, self.samples = camera, samples
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._run, name='camera-reader', daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        seq = 0
        last_digest = None
        last_changed_position = None
        last_change = time.monotonic()
        try:
            while not self.stop_event.is_set():
                # 用调用前时间标记，避免把阻塞读取前的帧伪装成动作后的帧。
                stamp = time.monotonic()
                try:
                    frame = self.camera.read_cv2_image(strategy='newest', timeout=0.3)
                except Exception as e:
                    # SDK 队列超时可短暂发生；持续无帧由 get 的总超时处理。
                    import queue
                    if isinstance(e, queue.Empty):
                        continue
                    raise
                if frame is None:
                    continue
                digest = hashlib.blake2s(frame.tobytes(), digest_size=8).digest()
                try:
                    pos, _ = self.samples.get('position', 0.5)
                except SafetyFault:
                    pos = None
                if digest != last_digest:
                    last_digest, last_change, last_changed_position = digest, stamp, pos
                elif (pos is not None and last_changed_position is not None
                      and stamp-last_change > 1.5
                      and math.hypot(pos[0]-last_changed_position[0], pos[1]-last_changed_position[1]) > 0.02):
                    raise SafetyFault('机器人已移动但视频持续逐字节相同，疑似冻结')
                seq += 1
                with self.condition:
                    self.latest = (seq, stamp, frame)
                    self.condition.notify_all()
        except BaseException as e:
            with self.condition:
                self.error = e
                self.condition.notify_all()

    def get(self, after, timeout=2.0):
        deadline = time.monotonic()+timeout
        with self.condition:
            while True:
                if self.error is not None:
                    raise SafetyFault(f'相机读取失败: {self.error}') from self.error
                if self.latest is not None and self.latest[1] >= after:
                    seq, stamp, frame = self.latest
                    return seq, stamp, frame.copy()
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise SafetyFault('相机未提供新的有效帧')
                self.condition.wait(min(0.1, remaining))

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        return not self.thread.is_alive()


class RealRobot:
    def __init__(self, c, emit, preview=True):
        self.c, self.emit, self.preview = c, emit, preview
        self.samples = Samples()
        self.ep = None
        self.reader = None
        self.initialized = self.streaming = False
        self.subscriptions = []
        self.origin = None
        self.detector = None
        self.last_frame = None

    now = staticmethod(time.monotonic)

    def connect(self):
        self.check_abort()
        self.detector = YoloDetector(self.c)  # 连接/运动前检查全部模型并预热
        from robomaster import robot
        self.ep = robot.Robot()
        try:
            self.ep.initialize(conn_type=self.c['connection']['type'])
            self.initialized = True
            self.stop()
            if self.ep.set_robot_mode(mode=robot.FREE) is False:
                raise SafetyFault('无法设置底盘自由模式')
            def subscribe(module, method, key, **kw):
                callback = lambda value: self.samples.put(key, value)
                if not getattr(module, method)(callback=callback, **kw):
                    raise SafetyFault(f'订阅 {key} 失败')
                self.subscriptions.append((module, method.replace('sub_', 'unsub_', 1)))
            subscribe(self.ep.chassis, 'sub_position', 'position', cs=0, freq=20)
            subscribe(self.ep.chassis, 'sub_attitude', 'attitude', freq=20)
            subscribe(self.ep.robotic_arm, 'sub_position', 'arm', freq=10)
            subscribe(self.ep.gripper, 'sub_status', 'gripper', freq=10)
            if self.c['tof']['enabled']:
                subscribe(self.ep.sensor, 'sub_distance', 'distance', freq=10)
            deadline = self.now()+5
            while True:
                try:
                    pos, _ = self.samples.get('position', 0.5)
                    att, _ = self.samples.get('attitude', 0.5)
                    self.arm_position()
                    self.samples.get('gripper', 0.5)
                    self.origin = (float(pos[0]), float(pos[1]), float(att[0]))
                    break
                except SafetyFault:
                    if self.now() > deadline:
                        raise
                    self.sleep(0.05)
            if self.ep.camera.start_video_stream(display=False, resolution=self.c['vision']['resolution']) is False:
                raise SafetyFault('视频流启动失败')
            self.streaming = True
            self.reader = CameraReader(self.ep.camera, self.samples)
            self.reader.start()
            _, _, first_frame = self.reader.get(self.now(), timeout=5)
            self.frame_shape = first_frame.shape[:2]
            prepare_visual_geometry(self.c, first_frame.shape[1])
            validate(self.c)
            self.emit('connected', origin=self.origin, device=self.detector.device,
                      frame_height=first_frame.shape[0], frame_width=first_frame.shape[1],
                      class_geometry=self.c['classes'])
        except BaseException:
            self.close()
            raise

    def pose(self):
        age = self.c['motion']['telemetry_max_age_s']
        pos, pt = self.samples.get('position', age)
        att, at = self.samples.get('attitude', age)
        if self.origin is None:
            raise SafetyFault('场地原点未建立')
        ox, oy, yaw0 = self.origin
        angle = math.radians(self.c['layout']['lane_angle_in_odom_deg'])
        dx, dy = float(pos[0])-ox, float(pos[1])-oy
        return Pose(math.cos(angle)*dx+math.sin(angle)*dy,
                    -math.sin(angle)*dx+math.cos(angle)*dy+self.c['layout']['start_y_m'],
                    wrap(float(att[0])-yaw0)*self.c['layout']['yaw_feedback_to_clockwise'], min(pt, at))

    def check_abort(self):
        if self.reader and self.reader.error:
            raise SafetyFault(f'相机故障: {self.reader.error}')
        if self.preview and self.last_frame is not None:
            import cv2
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                raise KeyboardInterrupt('preview stop')

    def sleep(self, seconds):
        end = self.now()+seconds
        while self.now() < end:
            self.check_abort()
            time.sleep(min(0.04, max(0, end-self.now())))

    def velocity(self, x, y, z):
        self.check_abort()
        result = self.ep.chassis.drive_speed(x=x, y=y,
                    z=z*self.c['layout']['sdk_z_to_clockwise'],
                    timeout=self.c['motion']['velocity_lease_s'])
        if result is False:
            raise SafetyFault('底盘速度命令失败')

    def stop(self, strict=True):
        if self.initialized:
            try:
                if self.ep.chassis.drive_speed(x=0, y=0, z=0) is False:
                    raise RuntimeError('零速命令未被确认')
            except Exception as e:
                log.exception('零速命令发送失败')
                if strict:
                    raise SafetyFault('停止指令失败，禁止继续执行任务') from e

    def check_front_clearance(self):
        t = self.c['tof']
        if not t['enabled']:
            return
        raw, _ = self.samples.get('distance', t['max_age_s'])
        try:
            value = float(raw[t['index']])/1000.0
        except (TypeError, ValueError, IndexError) as e:
            raise SafetyFault('前向测距格式异常') from e
        if not math.isfinite(value) or not 0 < value <= t['max_valid_m']:
            raise SafetyFault('前向测距无效；不拿其他通道/旧值冒充当前距离')
        if value <= t['min_clearance_m']:
            raise SafetyFault(f'前向净距不足: {value:.3f} m')

    def arm_position(self):
        value, stamp = self.samples.get('arm', self.c['motion']['telemetry_max_age_s'])
        def signed(v):
            v = float(v)
            return v-4294967296 if v >= 2147483648 else v
        xy = tuple(signed(v) for v in value[:2])
        if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
            raise SafetyFault('机械臂位置数据无效')
        return xy, stamp

    def require_transport_pose(self):
        xy, _ = self.arm_position()
        if any(abs(u-v) > self.c['arm']['tolerance_mm'] for u,v in zip(xy, self.c['arm']['carry'])):
            raise SafetyFault('机械臂不在运输姿态，拒绝横移')

    def arm_pose(self, target):
        self.stop()
        self.check_abort()
        arm_target(self.c, target)
        a = self.c['arm']
        (cx, cy), _ = self.arm_position()
        if not (a['x_min_mm']-a['tolerance_mm'] <= cx <= a['x_max_mm']+a['tolerance_mm']
                and a['y_min_mm']-a['tolerance_mm'] <= cy <= a['y_max_mm']+a['tolerance_mm']):
            raise SafetyFault('当前机械臂位置超出已标定范围，停止自动调整')
        lift_x = min(a['x_max_mm'], max(a['x_min_mm'], cx))
        tx, ty = target
        # 先竖直抬升，再在高处伸缩，最后下降。避免低位内收撞到车体。
        route = [(lift_x, a['travel_y_mm']), (tx, a['travel_y_mm']), (tx, ty)]
        for x, y in route:
            arm_target(self.c, (x, y))
            xy, _ = self.arm_position()
            if max(abs(xy[0]-x), abs(xy[1]-y)) <= a['tolerance_mm']:
                continue
            began = self.now()
            action = self.ep.robotic_arm.moveto(x=int(round(x)), y=int(round(y)))
            # 一次等待覆盖动作时长，不反复用短 timeout 破坏 SDK Action 状态。
            ok = action.wait_for_completed(timeout=a['timeout_s'])
            self.check_abort()
            if not ok or not action.has_succeeded:
                raise SafetyFault(f'机械臂动作失败/超时: {(x,y)}')
            deadline = self.now()+1.5
            stable = 0
            last_stamp = -1
            while self.now() < deadline:
                xy, stamp = self.arm_position()
                if stamp > max(began, last_stamp):
                    stable = stable+1 if max(abs(xy[0]-x), abs(xy[1]-y)) <= a['tolerance_mm'] else 0
                    last_stamp = stamp
                    if stable >= 3:
                        break
                self.sleep(0.04)
            else:
                raise SafetyFault(f'机械臂未到位: target={(x,y)}, actual={xy}')
            self.emit('arm_done', target=[x,y], actual=list(xy))

    def grip(self, opened):
        self.stop()
        a = self.c['arm']
        fn = self.ep.gripper.open if opened else self.ep.gripper.close
        began = self.now()
        try:
            if not fn(power=a['open_power'] if opened else a['close_power']):
                raise SafetyFault('夹爪命令失败')
            self.sleep(a['grip_seconds'])
        finally:
            if self.ep.gripper.pause() is False:
                raise SafetyFault('夹爪暂停命令失败')
        statuses = []
        last_stamp = began
        deadline = self.now()+1.5
        while len(statuses) < 3 and self.now() < deadline:
            status, stamp = self.samples.get('gripper', self.c['motion']['telemetry_max_age_s'])
            if isinstance(status, (tuple, list)) and len(status) == 1:
                status = status[0]
            if stamp > last_stamp:
                statuses.append(str(status))
                last_stamp = stamp
            self.sleep(0.04)
        if len(statuses) < 3 or len(set(statuses)) != 1:
            raise SafetyFault('夹爪状态未稳定')
        status = statuses[-1]
        if status not in ('opened', 'closed', 'normal') or opened and status != 'opened':
            raise SafetyFault(f'夹爪状态不符: {status}')
        self.emit('gripper', opened=opened, status=status)
        return status

    def observe(self, after):
        self.stop()
        self.check_abort()
        seq, stamp, frame = self.reader.get(after)
        if frame.shape[:2] != self.frame_shape:
            raise SafetyFault('运行中相机分辨率改变，停止以避免停车比例错误')
        detections = self.detector.detect(frame)
        self.last_frame = frame
        if self.preview:
            import cv2
            display = frame.copy()
            h,w = frame.shape[:2]
            for d in detections:
                x1,y1,x2,y2 = (int(d.box[0]*w), int(d.box[1]*h), int(d.box[2]*w), int(d.box[3]*h))
                cv2.rectangle(display,(x1,y1),(x2,y2),(0,220,0),2)
                cv2.putText(display, f'{d.label} {d.confidence:.2f} width={d.width:.3f}',
                            (x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,220,0),1)
            for name,color in (('scan_roi',(255,180,0)),('held_roi',(200,0,255))):
                r = self.c['vision'][name]
                if isinstance(r, (list, tuple)):
                    cv2.rectangle(display,(int(r[0]*w),int(r[1]*h)),(int(r[2]*w),int(r[3]*h)),color,2)
            cv2.imshow('EP sorter - q/ESC to stop', display)
            self.check_abort()
        return Observation(seq, stamp, detections)

    def close(self):
        self.stop(strict=False)
        if self.initialized:
            try:
                self.ep.gripper.pause()
            except Exception:
                log.exception('退出时夹爪暂停失败')
        reader_stopped = self.reader is None or self.reader.close()
        if not reader_stopped:
            log.error('相机读取线程未退出，避免并发销毁解码器；请结束进程并检查连接')
            return
        if self.streaming:
            try:
                self.ep.camera.stop_video_stream()
            except Exception:
                log.exception('停止视频流失败')
            self.streaming = False
        for module, method in reversed(self.subscriptions):
            try:
                getattr(module, method)()
            except Exception:
                log.exception('取消订阅失败: %s', method)
        self.subscriptions.clear()
        if self.ep is not None:
            try:
                self.ep.close()
            except Exception:
                log.exception('关闭 SDK 失败')
        self.initialized = False
        if self.preview:
            import cv2
            cv2.destroyAllWindows()


# ======================== 六格任务状态机 ========================

class SortTask:
    def __init__(self, io, config, emit):
        self.io, self.c, self.emit = io, config, emit
        self.nav = Navigator(io, config, emit)
        self.state = 'IDLE'
        self.slot = None
        self.payload = None
        self.used = {name: 0 for name in config['bins']}
        self.results = []
        self.last_seq = -1
        self.last_observation = None
        self.before_pick_detections = []
        self.before_pick_target = None
        self.held_roi = None

    def enter(self, state, **data):
        self.state = state
        self.emit('state', state=state, slot=self.slot, payload=self.payload, **data)

    def observe(self):
        # 后端必须提供本次调用之后取得的新帧；推理期间底盘处于停止状态。
        after = self.io.now()
        o = self.io.observe(after=after)
        if o.seq <= self.last_seq or o.stamp < after:
            raise SafetyFault('重复帧/动作前旧帧不能参与连续确认')
        if not 0 <= self.io.now()-o.stamp <= self.c['vision']['observation_max_age_s']:
            raise SafetyFault('视觉结果过期，请降低推理耗时或检查视频流')
        self.last_seq = o.seq
        self.last_observation = o
        self.emit('detections', slot=self.slot, state=self.state, frame=o.seq,
                  detections=[asdict(d) for d in o.detections])
        return o

    def candidates(self, observation, roi):
        return [d for d in observation.detections if d.inside(roi)
                and d.label in self.c['classes']
                and d.confidence >= self.c['classes'][d.label]['confidence']]

    def stable_target(self, label=None, previous=None):
        """多帧同一目标。冲突类别/多个目标不按最大框随意选一个。"""
        v = self.c['vision']
        deadline = self.io.now()+v['scan_timeout_s']
        count, first = 0, None
        candidate = None
        reason = 'empty_or_unrecognized'
        while self.io.now() < deadline:
            self.io.check_abort()
            o = self.observe()
            ds = self.candidates(o, v['scan_roi'])
            if len(ds) != 1:
                count, first, candidate = 0, None, None
                if len(ds) > 1:
                    reason = 'ambiguous_targets'
                elif any(d.inside(v['scan_roi']) for d in o.detections):
                    reason = 'unrecognized_or_low_confidence'
                self.io.sleep(0.02)
                continue
            d = ds[0]
            if label is not None and d.label != label:
                raise SkipSlot('target_class_changed')
            # 移动前后关联更宽松；静止确认仍要求稳定 IoU。
            if previous is not None and (abs(d.cx-previous.cx) > 0.18
                                        or not 0.6 <= d.width/previous.width <= 1.7):
                raise SkipSlot('target_identity_changed')
            if candidate is None or d.label != candidate.label or iou(d, candidate) < v['association_iou']:
                count, first = 1, o.stamp
            else:
                count += 1
            candidate = d
            if count >= v['confirm_frames'] and o.stamp-first >= v['confirm_duration_s']:
                return d
            self.io.sleep(0.02)
        raise SkipSlot(reason if label is None else 'target_lost')

    def center_on_lane(self, target, slot_y):
        self.enter('ALIGN_ON_LANE')
        m, v = self.c['motion'], self.c['vision']
        for _ in range(m['max_alignment_steps']):
            profile = self.c['classes'][target.label]
            error = target.cx-profile['aim_x_ratio']
            # 远处同样的像素偏差会在近处放大；用终点框宽折算远处容差。
            tolerance = profile['center_tolerance_ratio'] * min(1, target.width/profile['grasp_width_ratio'])
            if abs(error) <= tolerance:
                return target
            # 单目宽度只估算横向小调整；前进距离使用独立标定的终点图像比例。
            dy = v['image_right_to_lane_y'] * error/target.width * profile['width_m']
            dy = clamp(dy, m['alignment_step_limit_m'])
            desired = self.nav.pose().y + dy
            if abs(desired-slot_y) > m['max_alignment_offset_m']:
                raise SkipSlot('outside_current_slot')
            self.io.arm_pose(self.c['arm']['carry'])
            self.nav.move_y(desired)
            self.io.arm_pose(self.c['arm']['scan'])
            target = self.stable_target(target.label, target)
        raise SkipSlot('alignment_did_not_converge')

    def approach(self, target, slot_y):
        """只在基准线调整横向；近处偏心则先原路退回，再有限次重对准。"""
        m = self.c['motion']
        overall_deadline = self.io.now()+m['approach_timeout_s']
        for attempt in range(m['max_approach_attempts']):
            target = self.center_on_lane(target, slot_y)
            self.enter('APPROACH', attempt=attempt+1, label=target.label)
            p = self.c['classes'][target.label]
            while self.io.now() < overall_deadline:
                pose = self.nav.pose()
                self.io.check_front_clearance()
                ratio = target.width/p['grasp_width_ratio']
                if ratio > 1+p['width_tolerance_ratio']:
                    raise SkipSlot('too_close_or_wrong_box')
                if ratio >= 1-p['width_tolerance_ratio']:
                    if not p['min_pick_x_m'] <= pose.x <= p['max_pick_x_m']:
                        raise SkipSlot('visual_stop_outside_motion_bounds')
                    if abs(target.cx-p['aim_x_ratio']) <= p['center_tolerance_ratio']:
                        return target
                    break
                # 偏心明显时也退出，避免靠近时丢到视野边缘。
                if abs(target.cx-p['aim_x_ratio']) > 3*p['center_tolerance_ratio']:
                    break
                step = m['fine_step_m'] if ratio > 0.65 else m['approach_step_m']
                next_x = min(pose.x+step, p['max_pick_x_m'])
                if next_x-pose.x <= m['pose_tolerance_m']:
                    raise SkipSlot('approach_distance_limit')
                self.nav.move_x(next_x, guard=True)
                # 丢失目标时只在此处静止重识别，绝不使用上次目标继续前进。
                target = self.stable_target(target.label, target)
            if self.io.now() >= overall_deadline:
                raise SkipSlot('approach_timeout')
            # 不在夹取位附近横扫；先退到 x=0，再重新识别与调整。
            near_correction = self.c['vision']['image_right_to_lane_y'] * (target.cx-p['aim_x_ratio'])/target.width*p['width_m']
            realign_y = self.nav.pose().y + clamp(near_correction, m['alignment_step_limit_m'])
            if abs(realign_y-slot_y) > m['max_alignment_offset_m']:
                raise SkipSlot('outside_current_slot')
            self.enter('REALIGN_RETREAT')
            self.io.arm_pose(self.c['arm']['carry'])
            self.nav.move_x(0)
            self.nav.move_y(realign_y)
            self.io.arm_pose(self.c['arm']['scan'])
            target = self.stable_target(target.label)
        raise SkipSlot('final_alignment_failed')

    @staticmethod
    def unchanged_box(current, previous):
        # 沿用原自检“中心约40px、框宽约25%”思路，改为图像归一化尺度。
        return (current.label == previous.label and abs(current.cx-previous.cx) < 0.04
                and abs(current.cy-previous.cy) < 0.04
                and 0.75 <= current.width/previous.width <= 1.25)

    def learn_held_roi(self, label):
        """仅从实际出现的新同类框建立持物区，不把看不到目标当作夹取成功。"""
        if self.before_pick_target is None:
            raise SafetyFault('缺少抓取前图像，不能自动建立持物区域')
        v = self.c['vision']
        deadline = self.io.now()+v['scan_timeout_s']
        candidate = None
        first = None
        count = 0
        while self.io.now() < deadline:
            observation = self.observe()
            detections = [d for d in self.candidates(observation, [0,0,1,1]) if d.label == label]
            if any(self.unchanged_box(d, self.before_pick_target) for d in detections):
                raise SafetyFault('抓取后目标仍在原位置，拒绝搬运')
            new = [d for d in detections if not any(self.unchanged_box(d, old)
                   for old in self.before_pick_detections)]
            if len(new) != 1:
                candidate, first, count = None, None, 0
            else:
                d = new[0]
                if candidate is None or iou(candidate, d) < v['association_iou']:
                    first, count = observation.stamp, 1
                else:
                    count += 1
                candidate = d
                if count >= v['confirm_frames'] and observation.stamp-first >= v['confirm_duration_s']:
                    x1,y1,x2,y2 = d.box
                    padx, pady = (x2-x1)*0.2, (y2-y1)*0.2
                    self.held_roi = [max(0,x1-padx), max(0,y1-pady), min(1,x2+padx), min(1,y2+pady)]
                    self.emit('held_roi_learned', slot=self.slot, label=label, roi=self.held_roi)
                    return
            self.io.sleep(0.02)
        raise SafetyFault('无法自动定位携带物：检查相机能否看到抬升目标，或填写已标定held_roi')

    def held_evidence(self, label, present):
        v = self.c['vision']
        if v['held_roi'] == 'auto':
            if self.held_roi is None:
                if not present:
                    raise SafetyFault('没有持物区域，无法确认释放')
                self.learn_held_roi(label)
            region = self.held_roi
        else:
            region = v['held_roi']
        first = None
        count = 0
        deadline = self.io.now()+v['scan_timeout_s']
        while self.io.now() < deadline:
            o = self.observe()
            ds = self.candidates(o, region)
            matched = (len(ds) == 1 and ds[0].label == label) if present else not any(
                d.inside(region) for d in o.detections)
            if matched:
                if first is None:
                    first = o.stamp
                count += 1
                if count >= v['confirm_frames'] and o.stamp-first >= v['confirm_duration_s']:
                    return
            else:
                count, first = 0, None
            self.io.sleep(0.02)
        raise SafetyFault('无法确认持物' if present else '张爪后仍疑似持物，禁止计为已放置')

    def pick(self, target):
        self.enter('PICK', label=target.label)
        self.io.stop()
        if self.last_observation is None:
            raise SafetyFault('缺少抓取前视觉记录')
        self.before_pick_detections = list(self.last_observation.detections)
        self.before_pick_target = target
        self.held_roi = None  # 每次重新学习，不沿用另一类物体的位置
        self.io.arm_pose(self.c['arm']['pick'])
        self.payload = 'uncertain:' + target.label  # 从发出闭爪起，任何异常均不自动张爪。
        status = self.io.grip(opened=False)
        if status not in self.c['arm']['holding_statuses']:
            raise SafetyFault(f'闭爪状态 {status} 不符合已标定持物状态')
        self.io.arm_pose(self.c['arm']['carry'])
        self.enter('VERIFY_PICK')
        self.held_evidence(target.label, present=True)
        self.payload = target.label

    def deliver(self, label, bin_name):
        b = self.c['bins'][bin_name]
        offset_index = self.used[bin_name]
        drop_y = b['center_y_m']+b['drop_offsets_y_m'][offset_index]
        self.enter('RETREAT_TO_LANE')
        self.nav.move_x(0)
        self.enter('TRANSIT_TO_BIN', bin=bin_name, drop_index=offset_index)
        self.nav.move_y(drop_y)
        # 再看运输后物体是否仍在夹爪中。
        self.held_evidence(label, present=True)
        self.enter('APPROACH_BIN')
        self.nav.move_x(b['approach_x_m'])
        self.held_evidence(label, present=True)
        self.enter('PLACE')
        self.io.arm_pose(b['release_pose_mm'])
        self.io.grip(opened=True)
        self.io.arm_pose(self.c['arm']['carry'])
        self.enter('VERIFY_RELEASE')
        self.held_evidence(label, present=False)
        self.payload = None
        # 只有到达料箱+张开确认+持物区清空，才消耗投放点。
        self.used[bin_name] += 1
        self.enter('RETURN_TO_LANE')
        self.nav.move_x(0)
        return offset_index

    def run(self):
        try:
            self.enter('INITIALIZE')
            self.io.stop()
            self.nav.pose()
            self.io.arm_pose(self.c['arm']['carry'])
            self.io.grip(opened=True)
            for self.slot in range(1, 7):
                result = dict(slot=self.slot, label=None, pick='not_attempted', place='not_attempted')
                self.results.append(result)
                slot_y = (self.slot-1)*self.c['layout']['slot_spacing_m']
                try:
                    self.enter('TRANSIT_TO_SLOT')
                    self.nav.move_y(slot_y)
                    self.io.arm_pose(self.c['arm']['scan'])
                    self.enter('OBSERVE_SLOT')
                    target = self.stable_target()
                    result['label'] = target.label
                    bin_name = self.c['classes'][target.label]['bin']
                    if self.used[bin_name] >= len(self.c['bins'][bin_name]['drop_offsets_y_m']):
                        raise SkipSlot('bin_full')
                    target = self.approach(target, slot_y)
                    result['pick'] = 'attempting'
                    self.pick(target)
                    result['pick'] = 'gripper_and_visual_verified'
                    result['place'] = 'attempting'
                    drop_index = self.deliver(target.label, bin_name)
                    result.update(status='placed', place='opened_and_held_roi_clear',
                                  bin=bin_name, drop_index=drop_index)
                except SkipSlot as e:
                    if self.payload is not None:
                        raise SafetyFault('持物状态不明，禁止按空爪恢复') from e
                    self.enter('SKIP_RECOVERY', reason=str(e))
                    self.io.arm_pose(self.c['arm']['carry'])
                    self.nav.move_x(0)
                    result.update(status='skipped', reason=str(e))
                self.emit('slot_result', **result)
            self.enter('DONE', placed=sum(r.get('status') == 'placed' for r in self.results))
            return self.results
        except BaseException as e:
            try:
                self.io.stop()
            except Exception as stop_error:
                self.emit('stop_error', reason=str(stop_error))
            if self.results and 'status' not in self.results[-1]:
                self.results[-1].update(status='fault', reason=str(e), failed_state=self.state)
                self.emit('slot_result', **self.results[-1])
            self.enter('FAULT', reason=str(e), error_type=type(e).__name__)
            raise
        finally:
            self.io.stop()


def build_config():
    """只读本文件顶部参数；一次列出缺少的实测值，连接实机前完成校验。"""
    c = copy.deepcopy(CONFIG)
    missing = []

    def inspect(value, path='CONFIG'):
        if isinstance(value, dict):
            for key, item in value.items():
                if path == 'CONFIG.tof' and not value['enabled'] and key == 'min_clearance_m':
                    continue
                inspect(item, path + '.' + str(key))
        elif isinstance(value, (list, tuple)):
            for index, item in enumerate(value):
                inspect(item, path + '[' + str(index) + ']')
        elif value is None:
            missing.append(path)

    inspect(c)
    if missing:
        raise ConfigError('请在文件顶部参数区填写以下实测值，填写后直接再次运行：\n  '
                          + '\n  '.join(missing))
    prepare_visual_geometry(c, c['vision'].get('reference_width_px', 960))
    validate(c)
    for model in c['vision']['models']:
        path = Path(model['path']).expanduser()
        model['path'] = str(path if path.is_absolute() else SCRIPT_DIR / path)
        # 同名权重也兼容原工程的models目录，不更换模型类别或忽略缺失权重。
        if not path.is_absolute() and not Path(model['path']).is_file():
            in_models = SCRIPT_DIR / 'models' / path
            if in_models.is_file():
                model['path'] = str(in_models)
    return c


def main():
    """唯一入口：连接真实EP，自动处理六格。无命令行参数。"""
    robot_io = task = None
    event_file = None
    exit_code = 0
    run_dir = None
    try:
        # IDE某些控制台没有可用的stderr文件描述符，不影响普通日志。
        try:
            faulthandler.enable()
        except (OSError, RuntimeError, AttributeError):
            pass
        run_dir = LOG_DIR / (time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns()%1000000000:09d}')
        run_dir.mkdir(parents=True, exist_ok=False)
        logging.basicConfig(level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(message)s',
                            handlers=[logging.StreamHandler(),
                                      logging.FileHandler(run_dir / 'run.log', encoding='utf-8')],
                            force=True)
        event_file = (run_dir / 'events.jsonl').open('w', encoding='utf-8')

        def emit(event, **data):
            record = dict(event=event, monotonic_s=robot_io.now() if robot_io else time.monotonic(), **data)
            event_file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
            event_file.flush()
            if event in ('state', 'slot_result'):
                log.info('%s %s', event, json.dumps(data, ensure_ascii=False))

        config = build_config()
        # 此文件仅为本轮日志快照；下次启动仍从代码顶部读取参数。
        (run_dir / 'parameters.used.json').write_text(
            json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
        log.info('开始实机任务：六格、中心间距0.60m；先加载两个模型，再连接EP。')
        robot_io = RealRobot(config, emit, preview=SHOW_PREVIEW)
        robot_io.connect()
        # 连接后已按实际帧宽换算，更新日志快照。
        (run_dir / 'parameters.used.json').write_text(
            json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
        task = SortTask(robot_io, config, emit)
        results = task.run()
        placed = sum(result.get('status') == 'placed' for result in results)
        exit_code = 0 if placed == 6 else 2
        log.info('六格处理结束：已放置%d个，跳过%d格。', placed, 6-placed)
    except KeyboardInterrupt:
        exit_code = 130
        log.warning('用户停止；不自动松开携带物。')
    except ConfigError as error:
        exit_code = 1
        log.error('%s', error)
    except Exception:
        exit_code = 1
        log.exception('实机任务停止')
    finally:
        if robot_io is not None:
            try:
                robot_io.close()
            except Exception:
                exit_code = 1
                log.exception('关闭机器人连接时发生异常')
        if run_dir is not None and run_dir.exists():
            summary = dict(exit_code=exit_code,
                           results=task.results if task is not None else [],
                           bins_used=task.used if task is not None else {},
                           payload=task.payload if task is not None else None)
            try:
                (run_dir / 'summary.json').write_text(
                    json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
            except OSError:
                exit_code = 1
                log.exception('保存结果日志失败')
            log.info('本轮日志：%s', run_dir)
        if event_file is not None:
            event_file.close()
    return exit_code


if __name__ == '__main__':
    main()
