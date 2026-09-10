@echo off
chcp 65001 >nul
title Experiment 3 - Reuse running simulation
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 0 : re-run the full demo on the simulation already open
echo  Use this for a FAST second run (no simulator restart needed)
echo  Requires: CoppeliaSim already open with scene/second_imitation.ttt
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --no-start
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
