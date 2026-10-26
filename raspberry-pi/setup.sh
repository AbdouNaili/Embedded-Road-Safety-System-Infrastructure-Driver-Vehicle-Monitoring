#!/bin/bash
# Installs the app as a service that starts at boot.
# Run once from this folder:  bash setup.sh
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
USER_NAME="$(whoami)"

echo "=== Road Safety System - installation ==="

echo "[1/4] Making launch.sh executable..."
chmod +x "$DIR/launch.sh"

echo "[2/4] Installing the systemd service..."
sudo tee /etc/systemd/system/road-safety.service > /dev/null <<SERVICE
[Unit]
Description=Road Safety System
After=graphical.target

[Service]
User=$USER_NAME
WorkingDirectory=$DIR
ExecStart=$DIR/launch.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=graphical.target
SERVICE

echo "[3/4] Giving $USER_NAME access to the serial port (dialout group)..."
sudo usermod -a -G dialout "$USER_NAME"

echo "[4/4] Enabling the service..."
sudo systemctl daemon-reload
sudo systemctl enable road-safety.service

echo ""
echo "Done. Reboot to start the app automatically:  sudo reboot"
