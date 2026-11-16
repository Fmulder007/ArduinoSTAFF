@echo off
setlocal
"%~dp0arduino-cli-offline.exe" %*
exit /b %ERRORLEVEL%
