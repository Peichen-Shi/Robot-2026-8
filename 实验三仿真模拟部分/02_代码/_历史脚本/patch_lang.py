# -*- coding: utf-8 -*-
"""patch_lang.py —— 把录制脚本画面文字改为按语言字典输出（一次性补丁）"""
import io

P = "vision_pipeline_recorded.py"
s = io.open(P, encoding="utf-8").read()

REP = [
    ('state = {"label": "准备中 …", "sub": "", "last": 0.0, "frames": 0,',
     'state = {"label": "ready ...", "sub": "", "last": 0.0, "frames": 0,\n         "pip_lines": [],'),
    ('state["pip"] = annotate_cn(frame, dets, box_note="夹爪摄像头：正前方目标")',
     'state["pip"] = annotate_cn(frame, dets, box_note=T["note_cam"])'),
    ('ann = annotate_cn(frame, dets, box_note="夹爪相机：正前方目标")',
     'ann = annotate_cn(frame, dets, box_note=T["note_cam"])'),
    ('label("① 启动：仿真与夹爪相机就绪（左上＝夹爪摄像头·全程实时）", "位置 0/6")',
     'label(T["s_start"], T["prog"] % (0, VC.GRID_N, 0))'),
    ('label("② 识别方式：逐个位置用【夹爪相机】判断有无物体与类别", "位置 0/6")',
     'label(T["s_mode"], T["prog"] % (0, VC.GRID_N, 0))'),
    ('label("③ 后退：进入安全横向通道 x=%.3f" % X_SAFE, "位置 0/6")',
     'label(T["s_safe"] % X_SAFE, T["prog"] % (0, VC.GRID_N, 0))'),
    ('        sub = "位置 %d/%d · 已放置 %d" % (g, total, done)',
     '        sub = T["prog"] % (g, total, done)'),
    ('label("④ 对准：平移到 位置%d 前方" % g, sub)', 'label(T["s_align"] % g, sub)'),
    ('label("⑤ 就位：前移到抓取位（位置%d）" % g, sub)', 'label(T["s_approach"] % g, sub)'),
    ('label("⑥ 夹爪相机判断：位置%d 是否有物体（右上＝夹爪相机识别）" % g, sub)',
     'label(T["s_check"] % g, sub)'),
    ('            label("位置%d：夹爪相机未看到物体 → 跳过不抓取，前往下一个位置" % g, sub)',
     '            label(T["s_skip"] % g, sub)'),
    ('label("⑦ 抓取完成：关闭夹爪视角，抬升物体", sub)', 'label(T["s_lift"], sub)'),
    ('label("⑧ 后退：直线退回安全通道", sub)', 'label(T["s_back"], sub)'),
    ('label("⑨ 搬运：平移到 %s 前方" % ("左料盒" if bin_name == "LeftBin" else "右料盒"), sub)',
     'label(T["s_carry"], sub)'),
    ('label("⑩ 伸入：小车前移到放置位，机械臂前伸送料", sub)', 'label(T["s_insert"], sub)'),
    ('label("⑪ 放置：机械臂前伸，把物体放到箱内水平线 x=0.380", sub)', 'label(T["s_place"], sub)'),
    ('label("⑫ 松开：夹爪张开，物体留在网格位置", sub)', 'label(T["s_release"], sub)'),
    ('label("⑬ 收回：手臂退出箱体", sub)', 'label(T["s_retract"], sub)'),
    ('label("⑭ 全部完成：小车直角走位返回原位", "已放置 %d/%d" % (done, total))',
     'label(T["s_done"], T["prog_done"] % (done, total))'),
    ('label("结果：A类→左料盒　B类→右料盒　已放置 %d 个，跳过 %d 个（无物体）" %\n          (done, total - done), "%d / %d" % (done, total))',
     'label(T["r_sum"] % (done, total - done), "%d / %d" % (done, total))'),
    ('label("运行出错: %s" % e)', 'label("ERROR: %s" % e)'),
    # 跳过分支：使用 pip_no_object() 显示“无物体/不夹取”
    ('            state["pip"] = None\n            capture(force=True)\n            drive_x(X_SAFE)',
     '            pip_no_object(); nap(0.6)\n            state["pip"] = None\n            capture(force=True)\n            drive_x(X_SAFE)'),
]

n = 0
for a, b in REP:
    if a in s:
        s = s.replace(a, b)
        n += 1
    else:
        print("未匹配:", a.splitlines()[0][:70])
io.open(P, "w", encoding="utf-8").write(s)
print("替换成功 %d / %d 处" % (n, len(REP)))
