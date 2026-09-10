@echo off
chcp 65001 >nul
title Experiment 3 - TOP view full demo
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 2 : TOP view - same full classification run seen from above
echo  Best view for: vehicle path (right-angle moves) and bin clearance
echo  Time: about 2.5 minutes
echo ================================================================
echo.
python -u "code\launch_sim.py" top auto zh --fresh
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
