#!/bin/bash
# Shutdown all Cow Tracker processes

echo "Shutting down Cow Tracker processes..."

# Kill dashboard on port 5000
if command -v lsof &>/dev/null; then
    lsof -ti:5000 2>/dev/null | xargs -r kill -9
elif command -v fuser &>/dev/null; then
    fuser -k 5000/tcp 2>/dev/null
fi

# Kill python processes running app.py
pkill -f "app.py" 2>/dev/null

# Kill ESP-IDF monitor
pkill -f "idf.py.*monitor" 2>/dev/null

# Kill esptool
pkill -f "esptool" 2>/dev/null

# Windows fallback (Git Bash)
if command -v powershell.exe &>/dev/null; then
    powershell.exe -NoProfile -Command "
        Get-NetTCPConnection -LocalPort 5000 -ErrorAction SilentlyContinue | ForEach-Object { Stop-Process -Id \$_.OwningProcess -Force -ErrorAction SilentlyContinue }
        Get-Process python -ErrorAction SilentlyContinue | Where-Object { \$_.CommandLine -like '*app.py*' } | Stop-Process -Force
    " 2>/dev/null
fi

echo "Done. All tracker processes stopped."
