@echo off
title IMU DAQ server (close this window to stop)
rem Starts the acquisition server and opens the GUI in the browser.
rem Python is taken from .venv, then the conda env "daq", then the PATH.
cd /d "%~dp0host"
set "PY="
if exist "%~dp0.venv\Scripts\python.exe" set "PY=%~dp0.venv\Scripts\python.exe"
if not defined PY if exist "%USERPROFILE%\miniconda3\envs\daq\python.exe" set "PY=%USERPROFILE%\miniconda3\envs\daq\python.exe"
if not defined PY if exist "%USERPROFILE%\anaconda3\envs\daq\python.exe" set "PY=%USERPROFILE%\anaconda3\envs\daq\python.exe"
if not defined PY set "PY=python"
"%PY%" server.py %*
echo.
pause
