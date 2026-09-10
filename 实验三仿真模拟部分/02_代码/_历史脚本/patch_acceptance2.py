# -*- coding: utf-8 -*-
"""patch_acceptance2.py —— 接入 不可达/料盒满 中止、结果记录、日志汇总"""
import io

P = "vision_pipeline_recorded.py"
s = io.open(P, encoding="utf-8").read()

REP = [
    # 1) 料盒已满 → 明确报错并中止
    ("""        cls = prim["class"]
        bin_name = VC.CLASS_BIN[cls]
        slot_y = bin_slots[bin_name][bin_used[bin_name]]
        bin_used[bin_name] += 1""",
     """        cls = prim["class"]
        bin_name = VC.CLASS_BIN[cls]
        if bin_used[bin_name] >= len(bin_slots[bin_name]):
            msg = ("cannot place object at slot %d: bin %s is full (%d/%d slots used) "
                   "- task aborted" % (g, bin_name, bin_used[bin_name], len(bin_slots[bin_name])))
            LOG.error(msg, "place")
            label("ERROR: " + msg, sub)
            nap(2.5)
            aborted = True
            break
        slot_y = bin_slots[bin_name][bin_used[bin_name]]
        bin_used[bin_name] += 1"""),
    # 2) 夹取失败（夹爪始终未接触物体）→ 判定不可达，重试一次后中止
    ("""        label(T["s_grasp"], sub)
        touched = gripper_close_until_touch(grid)""",
     """        label(T["s_grasp"], sub)
        touched = gripper_close_until_touch(grid)
        if not touched:
            print("    第 1 次闭合未接触物体 → 重试一次", flush=True)
            gripper(ST_OPEN); time.sleep(0.3)
            touched = gripper_close_until_touch(grid)
        if not touched:
            msg = ("target at slot %d is unreachable: gripper never touched an object "
                   "after 2 attempts - task aborted" % g)
            LOG.error(msg, "grasp")
            label("ERROR: unreachable target at slot %d - task aborted" % g, sub)
            nap(3.0)
            aborted = True
            break"""),
    # 3) 结果记录
    ("""        results["Grid%d" % g] = (pos(grid), bin_name, cls, slot_y)
        done += 1""",
     """        results["Grid%d" % g] = (pos(grid), bin_name, cls, slot_y)
        _p = pos(grid)
        _err = [abs(_p[0] - SLOT_X), abs(_p[1] - slot_y), abs(_p[2] - SLOT_Z)]
        LOG.result(slot=g, grid="Grid%d" % g, cls=cls, class_name=prim["class_name"],
                   confidence=round(prim["confidence"], 3), bin=bin_name,
                   target=[SLOT_X, slot_y, SLOT_Z], actual=[round(v, 4) for v in _p],
                   error=[round(v, 4) for v in _err], success=max(_err) < 0.02)
        done += 1"""),
    # 4) 初始化 aborted
    ("""    total = VC.GRID_N
    done = 0
    skipped = []""",
     """    total = VC.GRID_N
    done = 0
    aborted = False
    skipped = []"""),
    # 5) 返回时带上 aborted
    ("    return done, skipped", "    return done, skipped, aborted"),
    ("    done_cnt, skipped_pos = main_loop()",
     "    done_cnt, skipped_pos, was_aborted = main_loop()"),
    # 6) 结束后写日志汇总
    ("""    print("视频: %s  帧数=%d  大小=%.1fMB" %
          (OUT, state["frames"], os.path.getsize(OUT)/1024/1024), flush=True)""",
     """    print("视频: %s  帧数=%d  大小=%.1fMB" %
          (OUT, state["frames"], os.path.getsize(OUT)/1024/1024), flush=True)
    if LOG is not None:
        try:
            summ = LOG.close({"view": VIEW, "lang": LANG, "video": os.path.basename(OUT)})
            print("\\n=== 执行结果汇总 ===", flush=True)
            print("  抓取次数=%d  成功=%d  成功率=%.0f%%  错误=%d  安全违规=%d"
                  % (summ["grasp_attempts"], summ["grasp_success"],
                     summ["success_rate"] * 100, summ["errors"], summ["violations"]), flush=True)
            print("  轨迹: %s" % summ["files"]["trajectory"], flush=True)
            print("  结果: %s" % summ["files"]["results"], flush=True)
            print("  错误: %s" % summ["files"]["errors"], flush=True)
            print("  汇总: %s" % summ["files"]["summary"], flush=True)
        except Exception as e:
            print("日志汇总写入失败:", e, flush=True)"""),
]

n = 0
for a, b in REP:
    if a in s:
        s = s.replace(a, b, 1)
        n += 1
    else:
        print("未匹配:", a.splitlines()[0][:70])
io.open(P, "w", encoding="utf-8").write(s)
print("阶段二替换：%d / %d" % (n, len(REP)))
