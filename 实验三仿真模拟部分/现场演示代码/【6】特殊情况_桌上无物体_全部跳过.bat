@echo off
chcp 65001 >nul
title Experiment 3 - No object on table
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 6 : SPECIAL - all objects removed from the table
echo  Every slot is judged as 'no object' by the gripper camera,
echo  so the robot skips all of them and finishes safely.  Time: about 1 min
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --fresh --noobj
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
