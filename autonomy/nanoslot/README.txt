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


