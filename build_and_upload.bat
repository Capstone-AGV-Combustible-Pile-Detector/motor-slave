@echo off
setlocal enabledelayedexpansion

cd /d "%~dp0"

echo ===================================================
echo     ESP32 BUILD ^& UPLOAD TOOL (Dhuro-Omnibot-ROS2)
echo ===================================================
echo.

:: 1. Locate arduino-cli
set "ARDUINO_CLI=arduino-cli"
where arduino-cli >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    if exist "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" (
        set "ARDUINO_CLI=C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    ) else if exist "%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" (
        set "ARDUINO_CLI=%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    ) else (
        echo [ERROR] arduino-cli.exe not found!
        echo Please make sure Arduino IDE 2.x is installed.
        pause
        exit /b 1
    )
)

:: 2. Set Board FQBN and Port
set "FQBN=esp32:esp32:esp32"
set "PORT=%~1"
if "!PORT!"=="" set "PORT=COM22"

echo [CONFIG]
echo   Arduino CLI : "!ARDUINO_CLI!"
echo   Board (FQBN): !FQBN!
echo   Target Port : !PORT!
echo   Working Dir : %cd%
echo.

echo ---------------------------------------------------
echo Select an option:
echo   [1] Build only (Compile)
echo   [2] Build and Upload to !PORT!
echo   [3] Upload to a custom COM Port
echo   [4] Exit
echo ---------------------------------------------------
set /p "CHOICE=Enter choice (1-4) [default: 2]: "
if "!CHOICE!"=="" set "CHOICE=2"

if "!CHOICE!"=="1" goto do_build
if "!CHOICE!"=="2" goto do_upload
if "!CHOICE!"=="3" goto do_custom_port
if "!CHOICE!"=="4" exit /b 0

:do_custom_port
set /p "PORT=Enter COM Port (e.g. COM22): "
if "!PORT!"=="" set "PORT=COM22"
goto do_upload

:do_build
echo.
echo [INFO] Compiling sketch...
"!ARDUINO_CLI!" compile --fqbn !FQBN! .
goto finish

:do_upload
echo.
echo ===================================================
echo [TIP] If it gets stuck at 'Connecting...',
echo       PRESS and HOLD the 'BOOT' button on ESP32
echo       until writing starts!
echo ===================================================
echo.
echo [INFO] Compiling and uploading to !PORT!...
"!ARDUINO_CLI!" compile --fqbn !FQBN! --upload --port !PORT! .
goto finish

:finish
echo.
if %ERRORLEVEL% EQU 0 (
    echo ===================================================
    echo [SUCCESS] Operation completed successfully!
    echo ===================================================
) else (
    echo ===================================================
    echo [ERROR] Failed with exit code %ERRORLEVEL%
    echo ===================================================
)
echo.
pause
