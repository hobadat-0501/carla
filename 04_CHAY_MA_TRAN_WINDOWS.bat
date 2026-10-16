@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
if not exist ".venv_0915\Scripts\python.exe" (
  echo Chay 00_CAI_DAT_WINDOWS.bat truoc.
  pause
  exit /b 1
)
.venv_0915\Scripts\python.exe -u chay_quy_trinh.py matrix %*
set "CARLA_RUN_CODE=%errorlevel%"
if not "%CARLA_RUN_CODE%"=="0" echo Chua hoan thanh. Doc dong LOI va cac tep nhat ky.
pause
exit /b %CARLA_RUN_CODE%
