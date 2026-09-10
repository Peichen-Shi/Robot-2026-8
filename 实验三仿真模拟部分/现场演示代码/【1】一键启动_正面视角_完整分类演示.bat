@echo off
chcp 65001 >nul
title Experiment 3 - Launch + FRONT full demo
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 1 : LAUNCH the whole simulation + FRONT view full demo
echo  Expected: 5 grasps, 5 success, 0 collision, 0 joint-limit violation
echo  Time: about 2.5 minutes (includes simulator cold start)
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --fresh
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
