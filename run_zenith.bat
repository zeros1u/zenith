@echo off
setlocal
cd /d "%~dp0"
if exist ".venv-yolo\Scripts\python.exe" (
    ".venv-yolo\Scripts\python.exe" app.py
) else (
    python app.py
)
if errorlevel 1 pause
endlocal
