#!/bin/bash
# Starts the road safety app on the Raspberry Pi screen.
cd "$(dirname "$0")"

export DISPLAY=:0
export XAUTHORITY="$HOME/.Xauthority"

# Use the virtual environment if there is one
if [ -f venv/bin/activate ]; then
    source venv/bin/activate
fi

exec python3 main.py
