@echo off
echo Shutting down Cow Tracker processes...

:: Kill Python processes running the dashboard (flask/socketio on port 5000)
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :5000 ^| findstr LISTENING') do (
    echo Killing process on port 5000 (PID: %%a)
    taskkill /PID %%a /F >nul 2>&1
)

:: Kill any python processes running app.py
wmic process where "commandline like '%%app.py%%'" call terminate >nul 2>&1

:: Kill any ESP-IDF monitor processes
wmic process where "commandline like '%%idf.py%%monitor%%'" call terminate >nul 2>&1

:: Kill any esptool serial connections
wmic process where "commandline like '%%esptool%%'" call terminate >nul 2>&1

echo Done. All tracker processes stopped.
pause
