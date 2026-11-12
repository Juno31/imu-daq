@echo off
rem One-time setup of the Python environment for IMU DAQ.
rem Uses conda (env "daq") when available, otherwise a local .venv with pip.
cd /d "%~dp0"
where conda >nul 2>nul
if %errorlevel%==0 (
  echo Creating / updating conda env "daq" ...
  call conda env update -n daq -f environment.yml --prune || goto :fail
) else (
  echo conda not found - creating .venv with the system Python ...
  python -m venv .venv || goto :fail
  ".venv\Scripts\python.exe" -m pip install --upgrade pip || goto :fail
  ".venv\Scripts\python.exe" -m pip install -r requirements.txt || goto :fail
)
echo.
echo Done. Install the NI-DAQmx driver too if you use an NI DAQ (see README), then run start.bat.
pause
exit /b 0
:fail
echo Setup failed - see the messages above.
pause
exit /b 1
