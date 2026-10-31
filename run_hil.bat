@echo off
title TurtleBot HIL bench
echo =================================================================
echo   HIL  -  charge the TurtleBot battery
echo   Laptop webcam + Burger kinematic plant. Real motors stay off.
echo   Dashboard  http://localhost:8000
echo   Pose map   http://127.0.0.1:5001
echo =================================================================
py -3.11 hil\run_hil.py
pause
