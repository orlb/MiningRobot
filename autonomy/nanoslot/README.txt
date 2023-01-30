This service responds to an Arduino being plugged in by firing off a handler program, allowing hotplug of running robot hardware.

Idea:
    - An Arduino is plugged into a USB port
    - udev recognizes the correct Arduino, fires up nanoslot systemd service
    - systemd service starts nanoslot.sh shell script
    - nanoslot.sh does some logging and execs nanoboot program
    - nanoboot connects to the Arduino and asks its slot ID
    - nanoboot execs the slot_<ID>/main program, which talks to that Arduino

Limitations:
    - Arduinos are identified by ID, which is coded into the firmware.
    - Need to test if udev->systemd->nanoboot handoff works at startup.


To flash an Arduino in the Arduino IDE, first kill off that slot program:

	sudo killall slot_A0

Flash the Arduino and re-plug or manually run the slot program with --dev /dev/ttyUSB0.
Be sure to close the Arduino serial monitor before running a slot program,
the Arduino seems to mess up slot serial communication. 


To install the udev and systemd support, run install/install.sh as root.
The scripts are coded to expect a user named "robot" for running the slot programs.


To see what the slot programs are doing,

	tail -f /nanoslot/log &

and hotplug.

