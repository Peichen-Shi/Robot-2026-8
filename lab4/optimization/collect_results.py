#!/usr/bin/env python3
"""Summarize recorded physical flights; a planned duration is never a race time."""
import argparse, csv, json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument("root",type=Path,nargs="?",default=Path(__file__).resolve().parents[1]/"results");a=p.parse_args()
rows=[]
for f in sorted(a.root.glob("*/race_acceptance.json")):
 d=json.loads(f.read_text());meta=json.loads((f.parent/"run_metadata.json").read_text())
 checks=d["checks"];valid=d["passed"] is True and d["passed_gates"]==12
 rows.append(dict(run_id=d["run_id"],valid=valid,under_7s=valid and d["first_gate_to_last_gate_ms"]<7000,
   race_time_ms=d["first_gate_to_last_gate_ms"] if valid else None,
   passed_gates=d["passed_gates"],planned_duration_s=meta.get("target_duration_s"),
   minimum_clearance_m=d.get("minimum_clearance_m"),collision=not checks["no_reported_mission_collision"],
   stable_exit=checks["stable_exit_last_second"],failure_reasons="; ".join(d["failure_reasons"])))
(a.root/"trial_summary.json").write_text(json.dumps(rows,indent=2)+"\n")
if rows:
 with (a.root/"trial_summary.csv").open("w",newline="") as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
for row in rows:
 print(row)
