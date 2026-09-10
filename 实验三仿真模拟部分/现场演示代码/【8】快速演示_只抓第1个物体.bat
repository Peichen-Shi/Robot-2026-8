@echo off
chcp 65001 >nul
title Experiment 3 - QUICK single grasp
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 8 : QUICK - grasp and place only the object at slot 1
echo  Shortest complete pick-and-place cycle (vision + grasp + place)
echo  Time: about 40 seconds
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --fresh --slots=1
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
