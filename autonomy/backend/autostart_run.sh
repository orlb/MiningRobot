#!/bin/sh
# Suitable for init scripts or login scripts (~/.config/autostart on gnome)
export DISPLAY=:0
#export BEACON=10.10.10.100
DIR="$( dirname $0)"
cd "${DIR}"
#(cd ../kend; date >> log; ./kend >> log) &

# no gui: way less battery, but harder to debug
(date >> log; ./backend --nogui "$@" >> log) &

# with GUI: more energy and CPU waste, but shows what's happening
#(date >> log; ./backend "$@" >> log) &


