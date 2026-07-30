$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot

if (-not (Test-Path -LiteralPath ".venv-yolo\Scripts\python.exe")) {
    python -m venv .venv-yolo
}

$yoloPython = Join-Path $PSScriptRoot ".venv-yolo\Scripts\python.exe"
& $yoloPython -m pip install `
    torch==2.12.1 torchvision==0.27.1 `
    --index-url https://download.pytorch.org/whl/cu126
& $yoloPython -m pip install -r requirements-yolo.txt
& $yoloPython -c "import torch, ultralytics; print('CUDA:', torch.cuda.is_available()); print('Ultralytics:', ultralytics.__version__)"

Write-Host ""
Write-Host "ZENITH YOLO environment is ready." -ForegroundColor Green
Write-Host "Start the application with run_zenith.bat."
