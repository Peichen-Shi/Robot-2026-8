@echo off
chcp 65001 >nul
title Experiment 3 - Bin full -> abort
cd /d "%~dp0"
echo ================================================================
echo  Experiment 3 : Desktop Object Auto-Sorting
echo  CoppeliaSim Edu 4.10 + RoboMaster EP + Python (ZMQ Remote API)
echo ----------------------------------------------------------------
echo  DEMO 7 : SPECIAL - each bin is limited to 1 slot (capacity demo)
echo  Slot 1 -> left bin, slot 2 -> right bin, slot 3 -> LEFT BIN FULL
echo  so an ERROR is logged and the task is aborted.  Time: about 1.5 min
echo ================================================================
echo.
python -u "code\launch_sim.py" front auto zh --fresh --bincap=1 --slots=1,2,3
echo.
echo ----------------------------------------------------------------
echo  Video   : videos\   (newest .mp4 in that folder)
echo  Logs    : logs\     (trajectory / results / errors / summary)
echo  Guide   : 00_demo_guide.md   (Chinese, full instructions)
echo ----------------------------------------------------------------
pause
