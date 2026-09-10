@echo off
chcp 65001 >nul
REM ============================================================
REM  start_sim.bat —— 实验三 完整仿真系统 一键启动（Windows 双击运行）
REM  可传参：  start_sim.bat front 输出.mp4 en
REM  默认   ：  front 视角 + 中文界面 + 输出到桌面“实验三_演示视频”文件夹
REM ============================================================
setlocal
cd /d "%~dp0"
set VIEW=%1
set OUT=%2
set LANG=%3
if "%VIEW%"=="" set VIEW=front
if "%OUT%"=="" set OUT=C:\Users\39562\Desktop\实验三_演示视频\实验三_视觉分类演示_正面_最终版.mp4
if "%LANG%"=="" set LANG=zh

echo ============================================================
echo   实验三 桌面物体自动分类整理 - 一键启动
echo   视角: %VIEW%    界面语言: %LANG%
echo   输出: %OUT%
echo ============================================================
python -u launch_sim.py "%VIEW%" "%OUT%" "%LANG%"
echo.
echo 结束。日志见 logs\ 目录（launch_*.log / trajectory_*.csv / results_*.json / errors_*.log）
pause
