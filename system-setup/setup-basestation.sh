#!/bin/bash

set -e

sudo cp ~/basestation/system-setup/*.rules /etc/udev/rules.d/
sudo cp ~/basestation/system-setup/*.link /etc/systemd/network/
sudo cp ~/basestation/system-setup/*.network /etc/systemd/network/
sudo cp ~/basestation/system-setup/basestation.service /etc/systemd/system/

sudo udevadm control --reload-rules
sudo systemctl restart systemd-udevd
sudo systemctl restart systemd-networkd
sudo systemctl daemon-reload
sudo systemctl enable basestation.service
sudo systemctl start basestation.service

echo "udev rules, network settings, and system services configured"
