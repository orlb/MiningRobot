#!/bin/sh
#  Gets called by systemd with serial device name

log=/nanoslot/log

echo >> $log
date >> $log
echo "nanoslot $@" >> $log

cd /nanoslot/dir
exec nanoboot/nanoboot "/dev/$@" >> $log 2>&1
echo "closed $@" >> $log

