#!/bin/sh
# Suitable for init scripts or login scripts (~/.config/autostart on gnome)
export DISPLAY=:0
#export BEACON=10.10.10.100
DIR="$( dirname $0)"
cd "${DIR}"
#(cd ../kend; date >> log; ./kend >> log) &
(date >> log; ./backend "$@" >> log) &


