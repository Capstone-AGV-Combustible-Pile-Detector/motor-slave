@echo off
setlocal enabledelayedexpansion

cd /d "%~dp0"

echo ===================================================
echo     ESP32 BUILD ^& UPLOAD TOOL (Dhuro-Omnibot-ROS2)
echo ===================================================

set "ARDUINO_CLI=arduino-cli"
where arduino-cli >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    if exist "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" (
        set "ARDUINO_CLI=C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    ) else if exist "%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" (
        set "ARDUINO_CLI=%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    ) else (
        echo [ERROR] arduino-cli.exe not found!
        echo Please ensure Arduino IDE 2.x or arduino-cli is installed.
        exit /b 1
    )
)

set "FQBN=%~1"
if "%FQBN%"=="" (
    set "FQBN=esp32:esp32:esp32"
)

set "PORT=%~2"

echo CLI : "!ARDUINO_CLI!"
echo FQBN: !FQBN!

if "%PORT%"=="" (
    echo.
    echo [INFO] Compiling sketch only (no port specified)...
    "!ARDUINO_CLI!" compile --fqbn !FQBN! .
) else (
    echo Target Port: !PORT!
    echo.
    echo ===================================================
    echo [TIP] If it gets stuck at 'Connecting...',
    echo       PRESS and HOLD the 'BOOT' button on ESP32
    echo       until writing starts!
    echo ===================================================
    echo.
    echo [INFO] Compiling and Uploading to !PORT!...
    "!ARDUINO_CLI!" compile --fqbn !FQBN! --upload --port !PORT! .
)

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ===================================================
    echo [SUCCESS] Operation completed successfully!
    echo ===================================================
) else (
    echo.
    echo ===================================================
    echo [ERROR] Failed with error code %ERRORLEVEL%
    echo ===================================================
    exit /b %ERRORLEVEL%
)
