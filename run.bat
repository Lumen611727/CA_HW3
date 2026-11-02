@echo off
if exist build\Release\hw3.exe (build\Release\hw3.exe & exit /b)
if exist build\hw3.exe (build\hw3.exe & exit /b)
echo hw3.exe not found. Run build.bat first.
exit /b 1
