@echo off
setlocal

if not exist "%~dp0arduino-cli.exe" (
    echo ERROR: arduino-cli.exe was not found beside this CMD file.
    exit /b 2
)

"%~dp0arduino-cli.exe" %*
exit /b %ERRORLEVEL%
