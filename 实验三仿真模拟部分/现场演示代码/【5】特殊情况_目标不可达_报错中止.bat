@echo off
chcp 65001 >nul
title Experiment 3 - Unreachable target -> abort
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 5 : SPECIAL - target is unreachable (slot 6 object moved 17cm away)
echo  Gripper camera sees it, the gripper cannot touch it after 2 attempts,
echo  then an ERROR is logged and the whole task is ABORTED.  Time: about 1 min
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --fresh --unreachable --slots=5,6
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
