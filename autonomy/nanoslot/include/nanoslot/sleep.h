/*
 Utility functions for gentle-on-CPU sleep function.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-22 (Public Domain)
*/
#ifndef __NANOSLOT_SLEEP_H
#define __NANOSLOT_SLEEP_H


#define NANO_TO_MILLI 1000000UL
/* Sleep for this many milliseconds.
    1ms sleep -> about 1% CPU used.
    10ms sleep -> under 0.1% CPU used.
*/
void data_exchange_sleep(int millisec=1) {
    // Don't hog the CPU, give up our timeslice
    struct timespec sleeptime;
    sleeptime.tv_sec=millisec/1000;
    sleeptime.tv_nsec=(millisec%1000)*NANO_TO_MILLI;
    nanosleep(&sleeptime,NULL);
}

#endif

