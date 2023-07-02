#!/bin/sh
#  Reboot the USB stack on a Raspberry Pi 4, which makes all the USB devices visible again.
# Basically unloads/reloads the kernel driver like: https://www.iram.fr/~blanchet/ethercat/unbind_manually_pci_device.html
# Trick discovered by the amazing John Pender in 2014. 

sudo su << EOF
echo -n "0000:01:00.0" > /sys/bus/pci/drivers/xhci_hcd/unbind
echo -n "0000:01:00.0" > /sys/bus/pci/drivers/xhci_hcd/bind
EOF
