/*
 Function definitions for sanity checking nanoslot data.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-29 (Public Domain)
*/
#ifndef __NANOSLOT_SANITY_H
#define __NANOSLOT_SANITY_H 
#include <stdio.h>

// Sanity-check this value, exit if we got something we didn't expect
void nanoslot_expected_value(int got,int expected,const char *what) {
    if (got!=expected) {
        printf("  ERROR: %s on Arduino is %d (0x%x), but we wanted %d (0x%x) (mismatch!)\n",
            what, got,got, expected,expected);
        fflush(stdout);
        exit(1);
    }
}

void nanoslot_exchange::sanity_check_size(void)
{
    if (size==0) size=sizeof(nanoslot_exchange);
    nanoslot_expected_value(size,sizeof(nanoslot_exchange),"nanoslot_exchange::size");
}



#endif

