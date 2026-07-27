#!/bin/bash

set -e

sudo cp ~/basestation/systemd/*.rules /etc/udev/rules.d/
sudo cp ~/basestation/systemd/*.link /etc/systemd/network/
sudo cp ~/basestation/systemd/*.network /etc/systemd/network/

sudo udevadm control --reload-rules
sudo systemctl restart systemd-udevd
sudo systemctl restart systemd-networkd

echo "udev rules and network settings configured"
