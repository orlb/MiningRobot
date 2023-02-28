/*
 Interface the lunatic data exchange with slot F0 arm nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-02-27 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xF0 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_F0  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_lunatic comm(&argc,&argv);
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p);

            if (comm.got_sensor) 
            {
            /*
                if (comm.my_sensor.stop) {
                    printf(" F0 STOP requested\n");
                    fflush(stdout);
                }
            */
            }
            
            if (comm.need_command)
            {
                comm.send_command(comm.my_command);
                if (comm.verbose) {
                    printf("  F0 motors: %3d %3d %3d %3d\n",comm.my_command.motor[0],comm.my_command.motor[1],comm.my_command.motor[2],comm.my_command.motor[3]); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

