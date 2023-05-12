/*
 Example where we interface the data exchange with slot serial comms.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xEE /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_EE  /* my exchange struct */
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
                printf("  Arduino latency: %d ms, heartbeat %02x\n", comm.my_sensor.latency, comm.my_sensor.heartbeat);
            }
            
            if (comm.lunatic_post_packet(p)) 
            {
                comm.send_command(comm.my_command);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

