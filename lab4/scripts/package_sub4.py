"""Package the final code and real evidence only after a complete 3/3 qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('root',type=Path)
parser.add_argument('summary',type=Path)
args=parser.parse_args()
root=args.root.resolve()
summary=json.loads(args.summary.read_text())
assert summary['qualified'] is True and len(summary['attempts'])==3
assert all(a['qualified'] and a['actual_race_ms']<4000 and a['stable_terminal_60s']
           for a in summary['attempts'])
manifest=json.loads((root/'results/sub4_frozen_manifest.json').read_text())
for name,digest in manifest.items():
    path=Path.home()/'vnav-builds/lab4/lab4.x86_64' if name=='simulator_binary' else root/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name

evidence=root/'evidence/sub4_qualified'
evidence.mkdir(exist_ok=True)
for attempt in summary['attempts']:
    name=attempt['run_id'];destination=evidence/name
    assert not destination.exists(),destination
    shutil.copytree(root/'results'/name,destination)
    startup=name[:-3]+'_startup'
    (evidence/startup).mkdir(exist_ok=True)
    for filename in ('startup.json','feedback_ready.json'):
        src=root/'results'/startup/filename
        if src.exists():shutil.copyfile(src,evidence/startup/filename)
shutil.copyfile(args.summary,evidence/'qualification_summary.json')
shutil.copyfile(root/'results/sub4_frozen_manifest.json',evidence/'frozen_manifest.json')
history=evidence/'history';history.mkdir(exist_ok=True)
for src in (root/'results').glob('qualify_sub4_*_summary.json'):
    if src!=args.summary:shutil.copyfile(src,history/src.name)
for filename in ('inertia_identification.json','trial_summary.json','trial_summary.csv'):
    src=root/'results'/filename
    if src.exists():shutil.copyfile(src,history/filename)
validation=root/'evidence/validation'
for filename in ('build_corner_bias.log','tests_final.log'):
    if (root/filename).exists():shutil.copyfile(root/filename,validation/filename)

table='\n'.join('| {} | {:.6f} s | 12/12 | 无 | {:.3f} m | 通过 |'.format(
    i+1,a['actual_race_ms']/1000,a['minimum_clearance_m']) for i,a in enumerate(summary['attempts']))
peak_error=max(a['terminal_tail_position_error_peak_m'] for a in summary['attempts'])
peak_speed=max(a['terminal_tail_speed_peak_mps'] for a in summary['attempts'])
peak_rate=max(a['terminal_tail_body_rate_peak_rad_s'] for a in summary['attempts'])
report=f'''# VNAV Lab4：4 秒以内稳定版

已在当前 Ubuntu 虚拟机的原 Unity 场景中完成三次独立重启验证，三次均小于 4 秒、通过全部 12 门、无任务内碰撞/控制故障，并连续稳定悬停至少 60 秒。

计时口径：**实际正向通过第 1 门至实际正向通过第 12 门的 ROS 仿真时间**。进场准备和最后一门后的减速不计入比赛用时，但减速及后续悬停仍检查碰撞、故障和稳定性。参考轨迹共 6.65 秒，包括 0.95 秒进场、3.9 秒门间计划时间和 1.8 秒减速。

| 独立重启 | 实际比赛用时 | 过门 | 碰撞 | 最小剩余门框余量 | 连续 60 秒悬停 |
|---|---:|---:|---|---:|---|
{table}

表中余量已扣除 0.18 m 机体余量，物理门孔仍为 2.4 m × 2.4 m。60 秒悬停窗口中，三次最坏位置误差为 {100*peak_error:.2f} cm、最坏速度为 {peak_speed:.4f} m/s、最坏机体角速度为 {peak_rate:.4f} rad/s。

## 当前虚拟机直接运行

```bash
bash ~/lab4_sub4_20261029/run_sub4_race.sh
```

自动打开 Unity 三维窗口、准备悬停、完成比赛并保持终点。等到录包分析输出 ACCEPTED 后才算该次验收通过；按 Ctrl+C 结束。已启动比赛时再次运行会被运行锁拦截。

单独停止本目录的飞行进程：

```bash
bash ~/lab4_sub4_20261029/stop_sub4.sh
```

保留的 Fast61 基线：

```bash
bash ~/lab4_sub4_20261029/run_fast61_baseline.sh
```

重新做三次独立重启、每次 65 秒终点观察：

```bash
bash ~/lab4_sub4_20261029/verify_sub4.sh ~/lab4_sub4_20261029
```

## 主要改进

- 联合调整分段时间和门内目标点；原始门位置、尺寸、顺序和参考偏航约束保留。门内偏移仍限制在 ±0.70 m。
- 根据真实录包修正控制器惯量模型，补偿指令延迟，并在高速飞行阶段优先建立推力方向。
- 独立设置高度控制增益；保留水平制动力，同时限制急转弯时的额外升力。
- 用小幅末段前馈补偿减少重复出现的转弯偏差，并提前建立最后一门附近的刹车姿态。
- 悬停阶段恢复已验证的控制增益和力矩上限；状态、参考、旋翼看门狗及碰撞遥测保持开启。

质量、重力、Unity 场景、仿真时钟和仿真器二进制保持原状。控制器的推力/力矩指令上限和允许倾角有提高；控制器惯量模型是估计值，未改写 Unity 内部物理参数。当前验证针对这台虚拟机和该固定赛道。

## 证据与配置

- `optimization/planner_profiles/sub4.yaml`：最终分段时间与门内目标点。
- `optimization/profiles/sub4.yaml`：最终控制参数。
- `evidence/sub4_qualified/qualification_summary.json`：本次完整 3/3 验证结果。
- `evidence/sub4_qualified/*_01/`：原始 bag、参数、实际过门 CSV、独立数值复核和长稳检查。
- `evidence/sub4_qualified/frozen_manifest.json`：源码、配置、构建及仿真器哈希。
- `evidence/sub4_qualified/history/`：探索过程和旧版未通过的资格测试汇总，失败用时不作为完整成绩。
- `evidence/validation/`：编译和测试日志；42 项 Python 测试及 4 项 C++ 测试通过。
- `PACKAGE_SHA256SUMS.txt`：此包完整性校验清单。

原工作空间 `~/vnav_ws` 作为只读依赖使用，改进代码和构建位于独立的 `~/lab4_sub4_20261029`。压缩包不含 Unity 二进制和编译缓存。重新构建可在解压目录执行 `LAB4_UNDERLAY="$HOME/vnav_ws/devel" bash install.sh`；冻结的构建哈希用于本机本路径复测，迁移或重新配置后应重新构建、记录哈希并重新验收。

历史 Fast61 说明和可靠性改进说明归档在 `HISTORY_Fast61_README.md` 与 `HISTORY_Fast61_Improvements.md`；原始 evidence 中的 6.1 秒图表属于历史版本，本次结果以 `evidence/sub4_qualified/` 为准。
'''
old_readme=root/'HISTORY_Fast61_README.md'
if not old_readme.exists():
    old_readme.write_text('# 历史记录：Sub4 调速前的 Fast61 说明\n\n以下内容为旧版记录，当前成绩与验证见 README_zh.md。\n\n'+(root/'README_zh.md').read_text(),encoding='utf-8')
old_notes=root/'IMPROVEMENTS_20261029.md'
if old_notes.exists():
    (root/'HISTORY_Fast61_Improvements.md').write_text('# 历史记录：Sub4 调速前的可靠性改进\n\n以下描述对应旧版阶段，当前结果见 README_zh.md。\n\n'+old_notes.read_text(),encoding='utf-8')
    old_notes.write_text('历史可靠性改进说明见 HISTORY_Fast61_Improvements.md。当前 Sub4 配置、实测和启动方法见 README_zh.md。\n',encoding='utf-8')
(root/'README_zh.md').write_text(report,encoding='utf-8')
(root/'SUB4_RESULTS.md').write_text(report,encoding='utf-8')

excluded={'ros_ws','results','controller_experiment','.git','__pycache__'}
files=[]
for path in root.rglob('*'):
    if not path.is_file():continue
    rel=path.relative_to(root)
    if any(part in excluded for part in rel.parts):continue
    if path.suffix in ('.pyc','.baseline'):continue
    if rel.parts[0].startswith('build_') or rel.parts[0].startswith('run_vm_'):continue
    if len(rel.parts)==1 and path.suffix=='.log':continue
    if rel.as_posix()=='PACKAGE_SHA256SUMS.txt':continue
    files.append((rel.as_posix(),path))
# Preserve the qualification input manifest where the verifier expects it.
files.append(('results/sub4_frozen_manifest.json',root/'results/sub4_frozen_manifest.json'))
files.sort()
checks=''.join(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+name+'\n' for name,path in files)
(root/'PACKAGE_SHA256SUMS.txt').write_text(checks,encoding='utf-8')
files.append(('PACKAGE_SHA256SUMS.txt',root/'PACKAGE_SHA256SUMS.txt'))
output=Path.home()/'VNAV_Lab4_Sub4_Stable_20261029.zip'
assert not output.exists(),output
with zipfile.ZipFile(output,'x',compression=zipfile.ZIP_DEFLATED) as archive:
    for name,path in files:archive.write(path,'VNAV_Lab4_Sub4_Stable_20261029/'+name)
with zipfile.ZipFile(output) as archive:assert archive.testzip() is None
print(json.dumps(dict(package=str(output),files=len(files),bytes=output.stat().st_size,
                      sha256=hashlib.sha256(output.read_bytes()).hexdigest()),indent=2))
