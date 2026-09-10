@echo off
chcp 65001 >nul
title Experiment 3 - QUICK two grasps (top view)
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 9 : QUICK - grasp slot 1 (class A) and slot 2 (class B), TOP view
echo  Shows both classes going to the correct left / right bin
echo  Time: about 1 minute
echo ================================================================
echo.
python -u "code\launch_sim.py" top auto zh --fresh --slots=1,2
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
