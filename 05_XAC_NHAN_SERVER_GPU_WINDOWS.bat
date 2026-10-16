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
set "CARLA_EXE=%~1"
if defined CARLA_EXE goto have_path
if exist "C:\CARLA_0.9.15\CarlaUE4.exe" set "CARLA_EXE=C:\CARLA_0.9.15\CarlaUE4.exe"
if defined CARLA_EXE goto have_path
set /p "CARLA_EXE=Nhap duong dan day du toi CarlaUE4.exe: "
set "CARLA_EXE=%CARLA_EXE:"=%"
:have_path
if not exist "%CARLA_EXE%" (
  echo Khong tim thay CarlaUE4.exe tai duong dan da nhap.
  pause
  exit /b 1
)
echo Dong CARLA server cu va cac client khac truoc khi chay phep thu nay.
echo Chuong trinh se mo server rieng, thay the the gioi va thu minh chung.
.venv_0915\Scripts\python.exe -u xac_nhan_server_gpu.py --server-exe "%CARLA_EXE%"
set "CARLA_VERIFY_CODE=%errorlevel%"
if "%CARLA_VERIFY_CODE%"=="2" echo Da co mo phong/camera, nhung GPU chua du minh chung.
if "%CARLA_VERIFY_CODE%"=="1" echo Chua hoan thanh. Doc KET_QUA_XAC_NHAN.txt va nhat ky.
pause
exit /b %CARLA_VERIFY_CODE%
