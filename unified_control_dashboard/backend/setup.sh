#!/bin/bash
# One-click setup for Unified Control Backend (Windows/Git Bash)

echo "--- Setting up Virtual Environment ---"
# Check for 'py' (Windows Launcher) or fallback to 'python'
if command -v py &> /dev/null; then
    PY_CMD="py"
else
    PY_CMD="python"
fi

$PY_CMD -m venv venv
echo "--- Activating Environment ---"
source venv/Scripts/activate
echo "--- Installing Dependencies ---"
pip install -r requirements.txt

echo "--- Setup Complete! ---"
echo "To run the server:"
echo "source venv/Scripts/activate"
echo "$PY_CMD -m app.main"
