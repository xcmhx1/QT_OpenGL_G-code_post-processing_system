@echo off
if "%~1"=="" (
    echo Drag machine code txt file onto this bat file.
    pause
    exit /b 1
)

powershell -ExecutionPolicy Bypass -File "%~dp0Generate-License.ps1" "%~1"
pause
