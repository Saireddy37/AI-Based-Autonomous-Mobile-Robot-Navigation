@echo off
setlocal
cd /d "%~dp0"
set PORT=5050

echo ============================================================
echo Robotics Project 2 - FINAL VERIFIED BUILD
echo ============================================================
echo Installing/updating Python dependencies...
python -m pip install -r requirements.txt
if errorlevel 1 (
  echo.
  echo ERROR: Dependency installation failed.
  pause
  exit /b 1
)

echo Running self-test before starting the dashboard...
python main.py --self-test
if errorlevel 1 (
  echo.
  echo ERROR: Self-test failed. See the messages above.
  pause
  exit /b 1
)

echo.
echo Starting dashboard at http://localhost:%PORT%
python main.py --port %PORT%
pause
