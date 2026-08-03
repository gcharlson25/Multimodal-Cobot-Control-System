@echo off
cd /d "%~dp0"
echo Starting Full System (teleop + vision + voice)...
echo.

echo [1] Starting robot client (Python 3.7)...
start "Robot Client" cmd /k python "robot_client.py"

timeout /t 3 /nobreak >nul

echo [2] Starting vision server (Python 3.12)...
start "Vision Server" cmd /k py -3.12 "vision_alignment.py"

timeout /t 3 /nobreak >nul

echo [3] Starting voice control (Python 3.14)...
start "Voice Control" cmd /k py -3.14 "voice_control.py"

echo.
echo All three processes launched. Close the windows to stop.
