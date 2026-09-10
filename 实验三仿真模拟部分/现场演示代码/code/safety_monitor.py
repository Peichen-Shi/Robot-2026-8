# -*- coding: utf-8 -*-
"""
safety_monitor.py —— 安全监控（验收标准：无碰撞、不超关节限位）
检查项：
  1) 关节限位：servo_motor_0 / servo_motor_1 / Prismatic_joint 的实际位置是否在关节区间内
  2) 与桌面/地面碰撞：机械臂与夹爪各连杆 与 Floor/桌面 是否接触
  3) 与目标物碰撞：机械臂与夹爪 与“非当前目标”的物体是否接触（当前目标在抓取瞬间会接触，属正常）
  4) 自碰撞：机械臂连杆 与 车体底座 是否接触
说明：料盒为 static+respondable=0 的视觉件，CoppeliaSim 无法对其做碰撞检测，
      因此“是否碰到料盒”改用距离余量判据（车头前缘 vs 箱壁外面，见主程序日志）。
"""
import math


class SafetyMonitor:
    ARM_LINKS = ["rod_link_respondable", "rod_1_link_respondable", "rod_2_link_respondable",
                 "rod_3_link_respondable", "arm_1_link_respondable", "arm_2_link_respondable",
                 "triangle_link_respondable", "endpoint_bracket_link_respondable",
                 "gripper_link_respondable"]
    FINGERS = ["left_gripper_5_respondable", "right_gripper_5_respondable"]
    JOINTS = ["servo_motor_0", "servo_motor_1"]

    def __init__(self, sim, find):
        self.sim = sim
        self.find = find
        self.robot = find("RoboMaster")
        self.prismatic = sim.getObject("/Prismatic_joint")
        self.joints = {n: find(n) for n in self.JOINTS}
        self.joints["Prismatic_joint"] = self.prismatic
        self.links = [find(n) for n in self.ARM_LINKS + self.FINGERS]
        self.links = [h for h in self.links if h is not None]
        self.grip_links = [h for h in (find(n) for n in ["gripper_link_respondable"] + self.FINGERS)
                           if h is not None]
        self.floor = find("Floor")
        self.base = find("base_link_visual")
        self.allow = set()          # 允许接触的物体（当前目标 / 手持物体）

    def set_allow(self, *objs):
        """设置允许接触的物体句柄（抓取瞬间手指会接触目标物）"""
        self.allow = set()
        for o in objs:
            if o is not None:
                self.allow.add(o)

    def _hit(self, a, b):
        if a is None or b is None:
            return False
        try:
            r = self.sim.checkCollision(a, b)
            v = r[0] if isinstance(r, (list, tuple)) else r
            return bool(v)
        except Exception:
            return False

    def check_joint_limits(self, margin=0.0):
        out = []
        for name, h in self.joints.items():
            if h is None:
                continue
            try:
                p = self.sim.getJointPosition(h)
                cyc, iv = self.sim.getJointInterval(h)
                if cyc:
                    continue
                lo, hi = min(iv), max(iv)
                if p < lo - 1e-6 - margin or p > hi + 1e-6 + margin:
                    out.append(("JOINT_LIMIT", "%s=%.4f 超出 [%.4f, %.4f]" % (name, p, lo, hi)))
            except Exception as e:
                out.append(("JOINT_LIMIT_ERR", "%s: %s" % (name, e)))
        return out

    def check_collisions(self, scene_objects):
        """手臂连杆：与任何物体/自身的接触都算违规；
           夹爪（腕部+手指）：只把“与非目标物体”的接触算违规（抓取瞬间接触目标是正常的）"""
        out = []
        arm_links = [h for h in self.links if h not in self.grip_links]
        # 1) 手臂/夹爪 与 桌面地面
        for h in self.links:
            if self._hit(h, self.floor):
                out.append(("COLLISION_TABLE", "link %d 与地面/桌面接触" % h))
                break
        # 2) 手臂连杆 与 任意物体（含目标物）——不允许
        for obj in scene_objects:
            for h in arm_links:
                if self._hit(h, obj):
                    out.append(("COLLISION_TARGET", "手臂 link %d 与物体 %d 接触" % (h, obj)))
                    break
        # 3) 夹爪 与 非当前目标物体——不允许
        for obj in scene_objects:
            if obj in self.allow:
                continue
            for h in self.grip_links:
                if self._hit(h, obj):
                    out.append(("COLLISION_GRIPPER", "夹爪 link %d 与非目标物体 %d 接触" % (h, obj)))
                    break
        # 4) 自碰撞（手臂/夹爪 与 车体底座）
        for h in self.links:
            if self._hit(h, self.base):
                out.append(("SELF_COLLISION", "link %d 与车体底座接触" % h))
                break
        return out

    def check(self, scene_objects, phase=""):
        return self.check_joint_limits() + self.check_collisions(scene_objects)
