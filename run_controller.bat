@echo off
title 3D Delta Robot Controller - NEMA 23
cd /d "%~dp0"
echo ========================================================
echo    Starting 3D Delta Robot Controller
echo    Actuators: 3x NEMA 23 (57BYG250C)
echo    Bicep: 150mm | Forearm: 400mm
echo ========================================================
echo.
python main.py
pause
