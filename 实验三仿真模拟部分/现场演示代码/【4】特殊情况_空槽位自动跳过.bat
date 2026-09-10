@echo off
chcp 65001 >nul
title Experiment 3 - Empty slot is skipped
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 4 : SPECIAL - slot 4 has NO object (the only empty slot)
echo  Robot goes to slot 4, gripper camera sees nothing, SKIPS it,
echo  then continues to slot 5 and grasps normally.  Time: about 1 minute
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --fresh --slots=4,5
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
