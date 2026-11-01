@echo off
rem Run a script with the "emio" conda environment (Python 3.12 + emioapi), whatever
rem Python is active. Usage:
rem     emio.bat cabledriven_sliders.py
rem     emio.bat camera_feedback.py run trajectories\test1.json --no-camera
rem     emio.bat cabledriven.py --check
setlocal
set "PY=%USERPROFILE%\anaconda3\envs\emio\python.exe"
if not exist "%PY%" (
    echo Could not find %PY%
    echo Create it with:  conda create -n emio python=3.12 -y  then  conda activate emio  and  pip install emioapi opencv-python
    exit /b 1
)
cd /d "%~dp0"
"%PY%" %*
