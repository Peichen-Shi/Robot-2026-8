@echo off
chcp 65001 >nul
title Experiment 3 - SIDE view full demo
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 3 : SIDE view - same full run seen from the side
echo  Best view for: arm lift / extend / place motion details
echo  Time: about 2.5 minutes
echo ================================================================
echo.
python -u "code\launch_sim.py" side auto zh --fresh
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
