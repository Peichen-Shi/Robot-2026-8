# -*- coding: utf-8 -*-
"""
sim_logger.py —— 数据记录：机械臂轨迹 / 执行结果 / 错误日志
输出（默认 logs/ 目录）：
    trajectory_<tag>_<时间戳>.csv   轨迹（时间、关节角、夹爪、车体与物体位置、阶段）
    results_<tag>_<时间戳>.json     执行结果（每个物体：位置、类别、置信度、落点误差、成功与否）
    errors_<tag>_<时间戳>.log       错误日志（含时间戳与阶段）
    summary_<tag>_<时间戳>.json     汇总（成功率、耗时、碰撞/限位统计）
"""
import csv
import json
import os
import time


class SimLogger:
    def __init__(self, logdir, tag):
        os.makedirs(logdir, exist_ok=True)
        self.tag = tag
        self.t0 = time.time()
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(logdir, "trajectory_%s_%s.csv" % (tag, ts))
        self.res_path = os.path.join(logdir, "results_%s_%s.json" % (tag, ts))
        self.err_path = os.path.join(logdir, "errors_%s_%s.log" % (tag, ts))
        self.sum_path = os.path.join(logdir, "summary_%s_%s.json" % (tag, ts))
        self.f = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_sim", "t_wall", "phase", "s0_deg", "s1_deg", "prism_m",
                         "gripper_state", "robot_x", "robot_y", "obj_x", "obj_y", "obj_z"])
        self.results = []
        self.errors = []
        self.violations = []
        self._ef = open(self.err_path, "w", encoding="utf-8")
        self._ef.write("# 错误日志  tag=%s  开始时间=%s\n" % (tag, time.strftime("%Y-%m-%d %H:%M:%S")))

    # ---------- 轨迹 ----------
    def traj(self, sim_time, phase, s0, s1, prism, grip, robot_xy, obj_xyz=None):
        obj = obj_xyz or ("", "", "")
        self.w.writerow(["%.3f" % sim_time, "%.3f" % (time.time() - self.t0), phase,
                         "%.3f" % s0, "%.3f" % s1, "%.5f" % prism, grip,
                         "%.4f" % robot_xy[0], "%.4f" % robot_xy[1],
                         obj[0] if obj[0] != "" else "", obj[1] if obj[1] != "" else "",
                         obj[2] if obj[2] != "" else ""])
        self.f.flush()

    # ---------- 结果 ----------
    def result(self, **kw):
        kw["t_wall"] = round(time.time() - self.t0, 2)
        self.results.append(kw)

    # ---------- 错误 ----------
    def error(self, msg, phase=""):
        rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "phase": phase, "message": msg}
        self.errors.append(rec)
        self._ef.write("[%s] (%s) %s\n" % (rec["time"], phase, msg))
        self._ef.flush()
        print("    !! ERROR: %s" % msg, flush=True)

    # ---------- 安全违规 ----------
    def violation(self, kind, detail):
        rec = {"time": round(time.time() - self.t0, 2), "kind": kind, "detail": detail}
        self.violations.append(rec)

    def close(self, extra=None):
        self.f.close()
        with open(self.res_path, "w", encoding="utf-8") as f:
            json.dump({"tag": self.tag, "results": self.results,
                       "errors": self.errors, "violations": self.violations},
                      f, ensure_ascii=False, indent=2)
        attempts = [r for r in self.results if not r.get("skipped")]
        n_ok = sum(1 for r in attempts if r.get("success"))
        n_all = len(attempts)
        summary = {
            "tag": self.tag,
            "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(time.time() - self.t0, 1),
            "grasp_attempts": n_all,
            "grasp_success": n_ok,
            "success_rate": round(n_ok / n_all, 3) if n_all else 0.0,
            "errors": len(self.errors),
            "violations": len(self.violations),
            "files": {"trajectory": os.path.basename(self.csv_path),
                      "results": os.path.basename(self.res_path),
                      "errors": os.path.basename(self.err_path),
                      "summary": os.path.basename(self.sum_path)},
        }
        if extra:
            summary.update(extra)
        with open(self.sum_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        self._ef.write("# 结束：抓取 %d 次，成功 %d 次，错误 %d 条，安全违规 %d 条\n"
                       % (n_all, n_ok, len(self.errors), len(self.violations)))
        self._ef.close()
        return summary
