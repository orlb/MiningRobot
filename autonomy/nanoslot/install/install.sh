#!/bin/sh
# Install udev and systemd rules for nanoslot slot handler

sudo mkdir /nanoslot
sudo chown robot /nanoslot

cd `dirname "$0"`
dir=`realpath ..`
(
cd /nanoslot
touch log
sudo chown robot log
ln -s "$dir"/nanohandler/nanoslot.sh .
ln -s "$dir" dir
)

sudo cp 90-nanoslot.rules /etc/udev/rules.d
sudo cp nanoslot@.service /etc/systemd/system


sudo udevadm control --reload
sudo systemctl daemon-reload


