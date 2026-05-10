@echo off
set "APP=%~dp0G-code_post-processing_system.exe"

if exist "%APP%" (
    start "" "%APP%" --license-request
    exit /b
)

for %%F in ("%~dp0*.exe") do (
    start "" "%%~fF" --license-request
    exit /b
)

echo Cannot find application exe in current directory.
pause
