@echo off
set "APP=%~dp0G-code_post-processing_system.exe"
set "CODE=%~dp0机器码.txt"

if exist "%APP%" (
    start "" /wait "%APP%" --license-request
    if exist "%CODE%" (
        echo Machine code file generated:
        echo %CODE%
    ) else (
        echo Failed to generate machine code file.
    )
    exit /b
)

for %%F in ("%~dp0*.exe") do (
    start "" /wait "%%~fF" --license-request
    if exist "%CODE%" (
        echo Machine code file generated:
        echo %CODE%
    ) else (
        echo Failed to generate machine code file.
    )
    exit /b
)

echo Cannot find application exe in current directory.
pause
