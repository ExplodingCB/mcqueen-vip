@echo off
setlocal
rem Always resolve tracks and configuration relative to this checkout.
pushd "%~dp0"
if errorlevel 1 exit /b 1

if exist ".venv\Scripts\python.exe" goto check_environment
rem Prefer the version used in the simulator guide over the newest installed Python.
py -3.12 -c "import sys" >nul 2>&1
if not errorlevel 1 goto create_with_py312
py -3 -c "import sys; sys.exit(sys.version_info < (3, 10))" >nul 2>&1
if not errorlevel 1 goto create_with_py
python -c "import sys; sys.exit(sys.version_info < (3, 10))" >nul 2>&1
if not errorlevel 1 goto create_with_python
echo Python 3.10 or newer is required. Install Python from https://www.python.org/downloads/windows/
echo Enable the Python launcher or add Python to PATH, then run this file again.
goto failed

:create_with_py312
py -3.12 -m venv .venv
if errorlevel 1 goto failed
goto install

:create_with_py
py -3 -m venv .venv
if errorlevel 1 goto failed
goto install

:create_with_python
python -m venv .venv
if errorlevel 1 goto failed
goto install

:check_environment
".venv\Scripts\python.exe" -c "import sys; sys.exit(sys.version_info < (3, 10))"
if errorlevel 1 (
    echo The existing .venv needs Windows Python 3.10 or newer.
    goto failed
)
".venv\Scripts\python.exe" -c "import mcq_sim, numpy, scipy, yaml, PIL" >nul 2>&1
if not errorlevel 1 goto launch

:install
echo Installing simulator dependencies. First-time setup requires internet access.
".venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 goto failed
".venv\Scripts\python.exe" -m pip install -e "src\mcq_sim"
if errorlevel 1 goto failed

:launch
echo Starting the simulator. Open the localhost URL printed below in your browser.
echo Press Ctrl+C in this window to stop the simulator.
".venv\Scripts\python.exe" -u -m mcq_sim view --port 0 %*
set "sim_exit=%errorlevel%"
if not "%sim_exit%"=="0" goto failed
popd
exit /b 0

:failed
echo.
echo Simulator startup failed. Review the error above and run start_sim.bat again.
pause
popd
exit /b 1
