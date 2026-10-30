@echo off
rem Launch the cable-driven rod simulation with the SOFA bundled in Emio Labs.
rem Usage: run_sim.bat            (manual: sliders in the "Cable rod: motors" window, press Play first)
rem        run_sim.bat bend       (scripted: pull each cable in turn)
rem        run_sim.bat circle     (scripted: circular sweep)
rem        run_sim.bat path       (scripted: keyframes from trajectory.py)
rem        run_sim.bat weave      (hardcoded animation: S-bend through the bars, weave.py)
rem        run_sim.bat play       (replay the newest recording made with the Record button)
rem        run_sim.bat play:NAME  (replay sofa_sim\recordings\NAME.json)
setlocal

set "SOFA=%LOCALAPPDATA%\Programs\emio-labs\resources\sofa"
if not exist "%SOFA%\bin\runSofa.exe" (
    echo Could not find SOFA at "%SOFA%". Is Emio Labs installed?
    exit /b 1
)

rem Same environment the Emio Labs app sets before starting SOFA.
set "SOFA_ROOT=%SOFA%"
set "PYTHONHOME=%SOFA%\bin\python"
set "PATH=%PYTHONHOME%;%PATH%"
set "PYTHONPATH=%PYTHONHOME%\Lib\site-packages;%SOFA%\plugins\SofaPython3\lib\python3\site-packages;%PYTHONPATH%"

set "SCENE=%~dp0cable_rod_scene.py"
if "%~1"=="" (
    "%SOFA%\bin\runSofa.exe" -l SofaPython3,SofaImGui -g imgui "%SCENE%"
) else (
    rem Scripted modes start playing straight away (-a)
    "%SOFA%\bin\runSofa.exe" -l SofaPython3,SofaImGui -g imgui -a "%SCENE%" --argv --demo=%~1
)
