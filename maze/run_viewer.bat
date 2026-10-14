@echo off
rem Bấm đúp để mở chương trình vẽ mê cung, nhận bản đồ từ xe qua Bluetooth HC-05 (COM8, 9600 baud).
rem Đổi cổng: sửa COM8 bên dưới (hoặc DEFAULT_PORT trong maze_viewer.py).
chcp 65001 >nul
cd /d "%~dp0"
python maze_viewer.py --port COM8 --baud 9600 %*
if errorlevel 1 (
    echo.
    echo Loi: kiem tra xe da bat, HC-05 da ghep doi, va dung cong COM8.
    pause
)
