/*
 Interface the lunatic data exchange with slot C0 mining head tool.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-04-12 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xC0 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_C0  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_lunatic comm(&argc,&argv);
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p,comm.my_sensor);

            if (comm.got_sensor) 
            {
                const float voltScale=5.0*(1.0/1023);
                comm.my_state.load=voltScale*(comm.my_sensor.cell0);
                comm.my_state.cell=voltScale*(comm.my_sensor.cell1 - comm.my_sensor.cell0);
            }
            
            if (comm.lunatic_post_packet(p))
            {
                comm.send_command(comm.my_command);
                if (comm.verbose) {
                    printf("  C0 mining motor: %3d\n",comm.my_command.mine); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(20);
    }
    
    return 0;
}

