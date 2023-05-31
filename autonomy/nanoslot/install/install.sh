#!/bin/sh
# Install udev and systemd rules for nanoslot slot handler,
# which automatically runs nanoboot and then the slot programs
# as soon as an Arduino Nano is plugged in.
#
# Requires a user named "robot".  Creates a directory /nanoslot
# (an alternative would be tailoring the paths in the scripts).

sudo mkdir /nanoslot
sudo chown robot /nanoslot

cd `dirname "$0"`
dir=`realpath ..`
(
cd /nanoslot
sudo touch log
sudo chown robot log
sudo ln -s "$dir"/install/nanoslot.sh .
sudo ln -s "$dir" dir
)

sudo cp 90-nanoslot.rules /etc/udev/rules.d
sudo cp nanoslot@.service /etc/systemd/system


sudo udevadm control --reload
sudo systemctl daemon-reload


