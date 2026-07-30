@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv-yolo\Scripts\python.exe" (
    echo Run setup_yolo.ps1 first.
    pause
    exit /b 1
)
set SDL_VIDEODRIVER=dummy
".venv-yolo\Scripts\python.exe" tools\generate_yolo_dataset.py
if errorlevel 1 goto :failed
".venv-yolo\Scripts\python.exe" tools\train_yolo.py
if errorlevel 1 goto :failed
echo Custom YOLO model trained successfully.
exit /b 0

:failed
echo YOLO training failed.
pause
exit /b 1
