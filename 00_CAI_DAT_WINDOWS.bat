@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
if exist ".venv_0915\Scripts\python.exe" goto validate
if exist ".venv_0915" (
  echo Thu muc .venv_0915 chua hoan chinh. Doi ten .venv_0915 cu roi chay lai.
  pause
  exit /b 1
)
where py >nul 2>&1
if errorlevel 1 goto plain_python
for %%V in (3.10) do (
  py -%%V -c "import struct,sys; sys.exit(struct.calcsize('P')!=8)" >nul 2>&1
  if not errorlevel 1 (
    py -%%V -m venv .venv_0915
    if errorlevel 1 goto failed
    goto validate
  )
)
:plain_python
python -c "import sys,struct; sys.exit(not (sys.version_info[:2]==(3,10) and struct.calcsize('P')==8))" >nul 2>&1
if errorlevel 1 goto no_python
python -m venv .venv_0915
if errorlevel 1 goto failed
:validate
.venv_0915\Scripts\python.exe -c "import sys,struct; sys.exit(not (sys.version_info[:2]==(3,10) and struct.calcsize('P')==8))"
if errorlevel 1 goto no_python
if not exist "logs" mkdir logs
echo Dang cai thu vien. Nhat ky: logs\cai_dat.log
.venv_0915\Scripts\python.exe -m pip install --disable-pip-version-check --only-binary=:all: -r requirements.txt > logs\cai_dat.log 2>&1
if errorlevel 1 goto failed
.venv_0915\Scripts\python.exe kiem_tra_moi_truong.py --offline
if errorlevel 1 goto failed
echo CAI DAT XONG. Mo CARLA 0.9.15 roi chay 01_KIEM_TRA_WINDOWS.bat
pause
exit /b 0
:no_python
echo Can CPython 3.10 ban 64 bit cho bo ma CARLA 0.9.15 nay.
echo Python 3.11/3.12/3.14 tren may khong duoc dung cho bo ma nay.
echo Cai Python phu hop, doi ten .venv_0915 cu neu co, roi chay lai.
pause
exit /b 1
:failed
echo CAI DAT CHUA XONG. Doc logs\cai_dat.log neu tep nay da duoc tao.
echo Neu pip khong tai duoc: kiem tra Internet/proxy. Khong cai CARLA server bang pip.
pause
exit /b 1
